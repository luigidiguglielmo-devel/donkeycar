#!/usr/bin/env python3
import time
SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'

DLP_SETTING_DISABLED = 0
CONFIG_REGISTER = 0x1A

class IMU:
    '''
    Installation:
    
    - MPU6050
    sudo apt install python3-smbus
    or
    sudo apt-get install i2c-tools libi2c-dev python-dev python3-dev
    git clone https://github.com/pimoroni/py-smbus.git
    cd py-smbus/library
    python setup.py build
    sudo python setup.py install

    pip install mpu6050-raspberrypi
    
    - MPU9250
    pip install mpu9250-jmdev
    
    '''

    def __init__(self, addr=0x68, poll_delay=0.0166, sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
        self.sensortype = sensor
        self.abias = [0, 0, 0]
        self.gbias = [0, 0, 0]

        if self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            self.sensor = MPU6050(addr)
            self.sensor.set_accel_range(MPU6050.ACCEL_RANGE_4G)
            self.sensor.set_gyro_range(MPU6050.GYRO_RANGE_1000DEG)
        
            if(dlp_setting > 0):
                self.sensor.bus.write_byte_data(self.sensor.address, CONFIG_REGISTER, dlp_setting)
            self.calibrate()
        
        else:
            from mpu9250_jmdev.registers import AK8963_ADDRESS, GFS_1000, AFS_4G, AK8963_BIT_16, AK8963_MODE_C100HZ
            from mpu9250_jmdev.mpu_9250 import MPU9250

            self.sensor = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=addr,  # In 0x68 Address
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_1000,
                afs=AFS_4G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ)
            
            if(dlp_setting > 0):
                self.sensor.writeSlave(CONFIG_REGISTER, dlp_setting)
            self.calibrate()
            self.sensor.configure()

        
        self.accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.mag = {'x': 0., 'y': 0., 'z': 0.}
        self.temp = 0.
        self.poll_delay = poll_delay
        self.on = True

    def calibrate(self):
        if self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            # compute sensor biases
            print('Calibrating IMU...')
            samples = 0
            for i in range(0, 200):
                adata = self.sensor.get_accel_data()
                self.abias[0] += adata[0]
                self.abias[1] += adata[1]
                self.abias[2] += adata[2]

                gdata = self.sensor.get_gyro_data()
                self.gbias[0] += gdata[0]
                self.gbias[1] += gdata[1]
                self.gbias[2] += gdata[2]
                time.sleep(0.01)
                samples += 1
            
            self.abias[0] /= samples
            self.abias[1] /= samples
            self.abias[2] /= samples
            self.abias[2] -= MPU6050.GRAVITIY_MS2 

            self.gbias[0] /= samples
            self.gbias[1] /= samples
            self.gbias[2] /= samples
            print(' done')
        else:
            self.sensor.calibrateMPU6500()

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
                
    def poll(self):
        try:
            if self.sensortype == SENSOR_MPU6050:
                self.accel = self.sensor.get_accel_data()
                self.accel[0] -= self.abias[0]
                self.accel[1] -= self.abias[1]
                self.accel[2] -= self.abias[2]

                self.gyro = self.sensor.get_gyro_data()
                self.gyro[0] -= self.gbias[0]
                self.gyro[1] -= self.gbias[1]
                self.gyro[2] -= self.gbias[2]

                self.temp = self.sensor.get_temp()

            else:
                from mpu9250_jmdev.registers import GRAVITY
                ret = self.sensor.getAllData()
                self.accel = { 'x' : ret[1] * GRAVITY, 'y' : ret[2] * GRAVITY, 'z' : ret[3] * GRAVITY }
                self.gyro = { 'x' : ret[4], 'y' : ret[5], 'z' : ret[6] }
                self.mag = { 'x' : ret[13], 'y' : ret[14], 'z' : ret[15] }
                self.temp = ret[16]
        except:
            print('failed to read imu!!')
            
    def run_threaded(self):
        return self.accel['x'], self.accel['y'], self.accel['z'], self.gyro['x'], self.gyro['y'], self.gyro['z'], self.temp

    def run(self):
        self.poll()
        return self.accel['x'], self.accel['y'], self.accel['z'], self.gyro['x'], self.gyro['y'], self.gyro['z'], self.temp

    def shutdown(self):
        self.on = False


if __name__ == "__main__":
    iter = 0
    import sys
    sensor_type = SENSOR_MPU6050 
    dlp_setting = DLP_SETTING_DISABLED
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    if len(sys.argv) > 2:
        dlp_setting = int(sys.argv[2])

    p = IMU(sensor=sensor_type)
    while iter < 100:
        data = p.run()
        print(data)
        time.sleep(0.1)
        iter += 1
     
