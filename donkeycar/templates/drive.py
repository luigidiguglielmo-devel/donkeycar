#!/usr/bin/env python3
"""
Scripts to drive a donkey car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical)] [--camera=(single|stereo)] [--meta=<key:value> ...] [--myconfig=<filename>]

Options:
    -h --help               Show this screen.
    --js                    Use physical joystick.
    -f --file=<file>        A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value>      Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
    --myconfig=filename     Specify myconfig file to use. 
                            [default: myconfig.py]
"""
from docopt import docopt

#
# import cv2 early to avoid issue with importing after tensorflow
# see https://github.com/opencv/opencv/issues/14884#issuecomment-599852128
#
try:
    import cv2
except:
    pass


import donkeycar as dk
from donkeycar.parts.tub_v2 import TubWriter
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.pipe import Pipe
from donkeycar.utils import *

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def drive(cfg, model_path=None, use_joystick=False, model_type=None, meta=[]):
    """
    Construct a working robotic vehicle from many parts. Each part runs as a
    job in the Vehicle loop, calling either it's run or run_threaded method
    depending on the constructor flag `threaded`. All parts are updated one
    after another at the framerate given in cfg.DRIVE_LOOP_HZ assuming each
    part finishes processing in a timely manner. Parts may have named outputs
    and inputs. The framework handles passing named outputs to parts
    requesting the same named input.
    """
    logger.info(f'PID: {os.getpid()}')

   # Initialize logging before anything else to allow console logging
    if cfg.HAVE_CONSOLE_LOGGING:
        logger.setLevel(logging.getLevelName(cfg.LOGGING_LEVEL))
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter(cfg.LOGGING_FORMAT))
        logger.addHandler(ch)

    # Initialize car
    V = dk.vehicle.Vehicle()
    tub_inputs = []
    tub_types = []
    
    # ------------------
    # - Adding sensors -
    # ------------------
    # 1. Camera
    # 2. IMU
    # 3. RC Controller

    # ----------------------------
    # - Camera and related topic -
    # ----------------------------
    # input topic: []
    # output topic: ['cam/image_array']
    # threaded: True
    # run condition: N/A
    # ----------------------------
    from donkeycar.parts.camera import MockCamera
    cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    V.add(cam, 
          inputs = [], 
          outputs  =['cam/image_array'], 
          threaded = True,
          run_condition = None)
    
    tub_inputs += ['cam/image_array']
    tub_types += ['image_array']

    # ----------------------------
    # - IMU and related topic -
    # ----------------------------
    # input topic: []
    # output topic: ['imu/acl_xyz', 'imu/gyr_xyz']
    # threaded: True
    # run condition: N/A
    # ----------------------------
    if cfg.HAVE_IMU:
        from donkeycar.parts.imu import IMU
        imu = IMU(sensor=cfg.IMU_SENSOR, 
                addr=cfg.IMU_ADDRESS,
                dlp_setting=cfg.IMU_DLP_CONFIG)
        V.add(imu,
            inputs = [], 
            outputs = ['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                        'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'], 
            threaded=True,
            run_condition = None)

        tub_inputs += ['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                    'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']
        tub_types += ['float', 'float', 'float',
                    'float', 'float', 'float']

    # -----------------------------------
    # - RC Controller and related topic -
    # -----------------------------------
    # input topic: []
    # output topic: ['TBD']
    # threaded: True
    # run condition: N/A
    # ----------------------------
    from donkeycar.parts.controller import PS4JoystickController
    ctr = PS4JoystickController(throttle_dir=cfg.JOYSTICK_THROTTLE_DIR,
                                throttle_scale=cfg.JOYSTICK_MAX_THROTTLE,
                                steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                                auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE,
                                dev_fn=cfg.JOYSTICK_DEVICE_FILE)
    ctr.set_deadzone(cfg.JOYSTICK_DEADZONE)
    V.add(ctr,
          inputs = ['user/mode', 'recording'],
          outputs = ['user/steering', 'user/throttle',
                     'user/mode', 'recording'],
          threaded = True,
          run_condition = None)

    tub_inputs +=['user/streering', 'user/throttle']
    tub_types +=['float', 'float']

    #
    # This web controller will create a web server that is capable
    # of managing steering, throttle, and modes, and more.
    #
    #ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
    #V.add(ctr,
    #      inputs=['cam/image_array', 'tub/num_records', 'user/mode', 'recording'],
    #      outputs=['user/steering', 'user/throttle', 'user/mode', 'recording', 'web/buttons'],
    #      threaded=True)

    # -------------------------------
    # converting sensor inputs into
    #       actuators commands
    # -------------------------------

    V.add(Pipe(), inputs=['user/steering'], outputs=['streering'])
    V.add(Pipe(), inputs=['user/throttle'], outputs=['throttle'])

    tub_inputs +=['steering', 'throttle']
    tub_types +=['float', 'float']

    # ------------------
    # - Adding actuators -
    # ------------------
    # 1. Steering
    # 2. Throttle
    # ------------------
    if not(cfg.DRIVE_TRAIN_TYPE == "MOCK"):
        from donkeycar.parts import pins
        #
        # drivetrain for RC car with servo and ESC.
        # using a PwmPin for steering (servo)
        # and as second PwmPin for throttle (ESC)
        #
        from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PulseController
        dt = cfg.PWM_STEERING_THROTTLE

        steering_controller = PulseController(
                pwm_pin=pins.pwm_pin_by_id(dt["PWM_STEERING_PIN"]),
                pwm_scale =    dt["PWM_STEERING_SCALE"],
                pwm_inverted = dt["PWM_STEERING_INVERTED"])
        steering = PWMSteering(controller=steering_controller,
                                left_pulse =  dt["STEERING_LEFT_PWM"],
                                right_pulse = dt["STEERING_RIGHT_PWM"])

        throttle_controller = PulseController(
                pwm_pin = pins.pwm_pin_by_id(dt["PWM_THROTTLE_PIN"]),
                pwm_scale =    dt["PWM_THROTTLE_SCALE"],
                pwm_inverted = dt['PWM_THROTTLE_INVERTED'])
        throttle = PWMThrottle(controller=throttle_controller,
                                max_pulse=  dt['THROTTLE_FORWARD_PWM'],
                                zero_pulse= dt['THROTTLE_STOPPED_PWM'],
                                min_pulse=  dt['THROTTLE_REVERSE_PWM'])
        
        V.add(steering, inputs=['steering'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)

    #
    # Create data storage part
    #
    tub_path = TubHandler(path=cfg.DATA_PATH).create_tub_path() if \
        cfg.AUTO_CREATE_NEW_TUB else cfg.DATA_PATH
    meta += getattr(cfg, 'METADATA', [])

    tub_writer = TubWriter(tub_path, inputs=tub_inputs, types=tub_types, metadata=meta)
    V.add(tub_writer, inputs=tub_inputs, outputs=["tub/num_records"], run_condition='recording')

    # run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config(myconfig=args['--myconfig'])

    if args['drive']:
        model_type = args['--type']
        camera_type = args['--camera']
        drive(cfg, model_path=args['--model'], use_joystick=args['--js'],
              model_type=model_type, camera_type=camera_type,
              meta=args['--meta'])
    elif args['train']:
        print('Use python train.py instead.\n')
