"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXAMPLE
-----------
import dk
cfg = dk.load_config(config_path='~/mycar/config.py')
print(cfg.CAMERA_RESOLUTION)

"""

import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#LOGGING
HAVE_CONSOLE_LOGGING = True
LOGGING_LEVEL = 'INFO'          # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects

# -------------
# -- VEHICLE --
# -------------
DRIVE_LOOP_HZ = 30      # the vehicle loop will pause if faster than this speed.
MAX_LOOPS = None        # the vehicle loop can abort after this many iterations, when given a positive integer.

#RECORD OPTIONS
RECORD_DURING_AI = False            #normally we do not record during ai mode. Set this to true to get image and steering records for your Ai. Be careful not to use them to train.
AUTO_RECORD_ON_THROTTLE = True      #if true, we will record whenever throttle is not zero. if false, you must manually toggle recording with some other trigger. Usually circle button on joystick.
AUTO_CREATE_NEW_TUB = False         #create a new tub (tub_YY_MM_DD) directory when recording or append records to data directory directly

#CAMERA
HAVE_CAMERA = False
CAMERA_TYPE = "CVCAM"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|D435|MOCK|IMAGE_LIST)
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
CAMERA_FRAMERATE = 24
CAMERA_VFLIP = False
CAMERA_HFLIP = False
CAMERA_INDEX = 0  # used for 'WEBCAM' and 'CVCAM' when there is more than one camera connected 

BGR2RGB = False  # true to convert from BRG format to RGB format; requires opencv

SHOW_PILOT_IMAGE = False  # show the image used to do the inference when in autopilot mode
USE_FPV = False           # send camera data to FPV webserver

# For IMAGE_LIST camera
# PATH_MASK = "~/mycar/data/tub_1_20-03-12/*.jpg"

#IMU
HAVE_IMU = False                #when true, this add a Mpu6050 part and records the data. Can be used with a
IMU_SENSOR = 'mpu6050'          # (mpu6050|mpu9250)
IMU_ADDRESS = 0x68              # if AD0 pin is pulled high them address is 0x69, otherwise it is 0x68
IMU_DLP_CONFIG = 0              # Digital Lowpass Filter setting (0:250Hz, 1:184Hz, 2:92Hz, 3:41Hz, 4:20Hz, 5:10Hz, 6:5Hz)
IMU_SAMPLERATE = 60             # sample rate (Hz) - frequency the imu value is read/updated

#WEB CONTROL
WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))  # which port to listen on when making a web controller
WEB_INIT_MODE = "user"              # which control mode to start in. one of user|local_angle|local. Setting local will start in ai mode.

#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = False     #when starting the manage.py, when True, will not require a --js option to use the joystick
JOYSTICK_MAX_THROTTLE = 0.5         #this scalar is multiplied with the -1 to 1 throttle value to limit the maximum throttle. This can help if you drop the controller or just don't need the full speed available.
JOYSTICK_STEERING_SCALE = 1.0       #some people want a steering that is less sensitve. This scalar is multiplied with the steering -1 to 1. It can be negative to reverse dir.
CONTROLLER_TYPE = 'web'             #(web|ps3|ps4|xbox|pigpio_rc|nimbus|wiiu|F710|rc3|MM1|custom) custom will run the my_joystick.py controller written by the `donkey createjs` command
JOYSTICK_DEADZONE = 0.01            # when non zero, this is the smallest throttle before recording triggered.
JOYSTICK_THROTTLE_DIR = -1.0        # use -1.0 to flip forward/backward, use 1.0 to use joystick's natural forward/backward
JOYSTICK_DEVICE_FILE = "/dev/input/js0" # this is the unix file use to access the joystick.


#PIGPIO RC control
STEERING_RC_GPIO = 26
THROTTLE_RC_GPIO = 20
DATA_WIPER_RC_GPIO = 19
PIGPIO_STEERING_MID = 1500         # Adjust this value if your car cannot run in a straight line
PIGPIO_MAX_FORWARD = 2000          # Max throttle to go fowrward. The bigger the faster
PIGPIO_STOPPED_PWM = 1500
PIGPIO_MAX_REVERSE = 1000          # Max throttle to go reverse. The smaller the faster
PIGPIO_SHOW_STEERING_VALUE = False
PIGPIO_INVERT = False
PIGPIO_JITTER = 0.025   # threshold below which no signal is reported


#9865, over rides only if needed, ie. TX2..
PCA9685_I2C_ADDR = 0x40     #I2C address, use i2cdetect to validate this number
PCA9685_I2C_BUSNUM = None   #None will auto detect, which is fine on the pi. But other platforms should specify the bus num.

#
# DRIVE_TRAIN_TYPE
# These options specify which chasis and motor setup you are using.
# See Actuators documentation https://docs.donkeycar.com/parts/actuators/
# for a detailed explanation of each drive train type and it's configuration.
# Choose one of the following and then update the related configuration section:
#
# "PWM_STEERING_THROTTLE" uses two PWM output pins to control a steering servo and an ESC, as in a standard RC car.
# "MOCK" no drive train.  This can be used to test other features in a test rig.
#
DRIVE_TRAIN_TYPE = "PWM_STEERING_THROTTLE"

#
# PWM_STEERING_THROTTLE
#
# Drive train for RC car with a steering servo and ESC.
# Uses a PwmPin for steering (servo) and a second PwmPin for throttle (ESC)
# Base PWM Frequence is presumed to be 60hz; use PWM_xxxx_SCALE to adjust pulse with for non-standard PWM frequencies
#
PWM_STEERING_THROTTLE = {
    "PWM_STEERING_PIN": "PCA9685.1:40.1",   # PWM output pin for steering servo
    "PWM_STEERING_SCALE": 1.0,              # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
    "PWM_STEERING_INVERTED": False,         # True if hardware requires an inverted PWM pulse
    "PWM_THROTTLE_PIN": "PCA9685.1:40.0",   # PWM output pin for ESC
    "PWM_THROTTLE_SCALE": 1.0,              # used to compensate for PWM frequence differences from 60hz; NOT for increasing/limiting speed
    "PWM_THROTTLE_INVERTED": False,         # True if hardware requires an inverted PWM pulse
    "STEERING_LEFT_PWM": 460,               #pwm value for full left steering
    "STEERING_RIGHT_PWM": 290,              #pwm value for full right steering
    "THROTTLE_FORWARD_PWM": 500,            #pwm value for max forward throttle
    "THROTTLE_STOPPED_PWM": 370,            #pwm value for no movement
    "THROTTLE_REVERSE_PWM": 220,            #pwm value for max reverse throttle
}

#When racing, to give the ai a boost, configure these values.
AI_LAUNCH_DURATION = 0.0            # the ai will output throttle for this many seconds
AI_LAUNCH_THROTTLE = 0.0            # the ai will output this throttle value
AI_LAUNCH_ENABLE_BUTTON = 'R2'      # this keypress will enable this boost. It must be enabled before each use to prevent accidental trigger.
AI_LAUNCH_KEEP_ENABLED = False      # when False ( default) you will need to hit the AI_LAUNCH_ENABLE_BUTTON for each use. This is safest. When this True, is active on each trip into "local" ai mode.

#Scale the output of the throttle of the ai pilot for all model types.
AI_THROTTLE_MULT = 1.0              # this multiplier will scale every throttle value for all output from NN models

# --------------
# -- TRAINING --
# --------------
# The default AI framework to use. Choose from (tensorflow|pytorch)
DEFAULT_AI_FRAMEWORK = 'tensorflow'

# The DEFAULT_MODEL_TYPE will choose which model will be created at training
# time. This chooses between different neural network designs. You can
# override this setting by passing the command line parameter --type to the
# python manage.py train and drive commands.
# tensorflow models: (linear|categorical|tflite_linear|tensorrt_linear)
# pytorch models: (resnet18)
DEFAULT_MODEL_TYPE = 'imu'
BATCH_SIZE = 128                #how many records to use when doing one pass of gradient decent. Use a smaller number if your gpu is running out of memory.
TRAIN_TEST_SPLIT = 0.8          #what percent of records to use for training. the remaining used for validation.
MAX_EPOCHS = 100                #how many times to visit all records of your data
SHOW_PLOT = True                #would you like to see a pop up display of final loss?
VERBOSE_TRAIN = True            #would you like to see a progress bar with text during training?
USE_EARLY_STOP = True           #would you like to stop the training if we see it's not improving fit?
EARLY_STOP_PATIENCE = 5         #how many epochs to wait before no improvement
MIN_DELTA = .0005               #early stop will want this much loss change before calling it improved.
PRINT_MODEL_SUMMARY = True      #print layers and weights to stdout
OPTIMIZER = None                #adam, sgd, rmsprop, etc.. None accepts default
LEARNING_RATE = 0.001           #only used when OPTIMIZER specified
LEARNING_RATE_DECAY = 0.0       #only used when OPTIMIZER specified
SEND_BEST_MODEL_TO_PI = False   #change to true to automatically send best model during training
CREATE_TF_LITE = True           # automatically create tflite model in training
CREATE_TENSOR_RT = False        # automatically create tensorrt model in training

PRUNE_CNN = False               #This will remove weights from your model. The primary goal is to increase performance.
PRUNE_PERCENT_TARGET = 75       # The desired percentage of pruning.
PRUNE_PERCENT_PER_ITERATION = 20 # Percenge of pruning that is perform per iteration.
PRUNE_VAL_LOSS_DEGRADATION_LIMIT = 0.2 # The max amout of validation loss that is permitted during pruning.
PRUNE_EVAL_PERCENT_OF_DATASET = .05  # percent of dataset used to perform evaluation of model.

#
# Augmentations and Transformations
#
# - Augmentations are changes to the image that are only applied during
#   training and are applied randomly to create more variety in the data.
#   Available augmentations are:
#   - BRIGHTNESS  - modify the image brightness. See [albumentations](https://albumentations.ai/docs/api_reference/augmentations/transforms/#albumentations.augmentations.transforms.RandomBrightnessContrast)
#   - BLUR        - blur the image. See [albumentations](https://albumentations.ai/docs/api_reference/augmentations/blur/transforms/#albumentations.augmentations.blur.transforms.Blur)
#
# - Transformations are changes to the image that apply both in
#   training and at inference.  They are always applied and in
#   the configured order.  Available image transformations are:
#   - Apply a mask to the image:
#     - 'CROP'      - apply rectangular mask to borders of image
#     - 'TRAPEZE'   - apply a trapezoidal mask to image
#   - Apply an enhancement to the image
#     - 'CANNY'     - apply canny edge detection
#     - 'BLUR'      - blur the image
#   - resize the image
#     - 'RESIZE'    - resize to given pixel width and height
#     - 'SCALE'     - resize by given scale factor
#   - change the color space of the image
#     - 'RGB2BGR'   - change color model from RGB to BGR
#     - 'BGR2RGB'   - change color model from BGR to RGB
#     - 'RGB2HSV'   - change color model from RGB to HSV
#     - 'HSV2RGB'   - change color model from HSV to RGB
#     - 'BGR2HSV'   - change color model from BGR to HSV
#     - 'HSV2BGR'   - change color model from HSV to BGR
#     - 'RGB2GRAY'  - change color model from RGB to greyscale
#     - 'BGR2GRAY'  - change color model from BGR to greyscale
#     - 'HSV2GRAY'  - change color model from HSV to greyscale
#     - 'GRAY2RGB'  - change color model from greyscale to RGB
#     - 'GRAY2BGR'  - change color model from greyscale to BGR
#
# You can create custom tranformations and insert them into the pipeline.
# - Use a tranformer label that beings with `CUSTOM`, like `CUSTOM_CROP`
#   and add that to the TRANSFORMATIONS or POST_TRANFORMATIONS list.
#   So for the custom crop example, that might look like this;
#   `POST_TRANSFORMATIONS = ['CUSTOM_CROP']`
# - Set configuration properties for the module and class that
#   implement your custom transformation.
#   - The module config will begin with the transformer label
#     and end with `_MODULE`, like `CUSTOM_CROP_MODULE`.  It's value is
#     the absolute file path to the python file that has the transformer
#     class.  For instance, if you called the file
#     `my_custom_transformer.py` and put in in the root of
#     your `mycar` folder, next to `myconfig.py`, then you would add 
#     the following to your myconfig.py file (keeping with the crop example);
#     `CUSTOM_CROP_MODULE = "/home/pi/mycar/my_custom_transformer.py"`
#     The actual path will depend on what OS you are using and what
#     your user name is.
#   - The class config will begin with the transformer label and end with `_CLASS`,
#     like `CUSTOM_CROP_CLASS`.  So if your class is called `CustomCropTransformer`
#     the you would add the following property to your `myconfig.py` file:
#     `CUSTOM_CROP_CLASS = "CustomCropTransformer"`
# - Your custom class' constructor will take in the Config object to
#   it it's constructor.  So you can add whatever configuration properties
#   you need to your myconfig.py, then read them in the constructor.
#   You can name the properties anything you want, but it is good practice
#   to prefix them with the custom tranformer label so they don't conflict
#   with any other config and so it is way to see what they go with.
#   For instance, in the custom crop example, we would want the border
#   values, so that could look like;
#   ```
#   CUSTOM_CROP_TOP = 45    # rows to ignore on the top of the image
#   CUSTOM_CROP_BOTTOM = 5  # rows ignore on the bottom of the image
#   CUSTOM_CROP_RIGHT = 10  # pixels to ignore on the right of the image
#   CUSTOM_CROP_LEFT = 10   # pixels to ignore on the left of the image
#   ```
# - Your custom class must have a `run` method that takes an image and
#   returns an image.  It is in this method where you will implement your
#   transformation logic.
# - For example, a custom crop that did a blur after the crop might look like;
#   ```
#   from donkeycar.parts.cv import ImgCropMask, ImgSimpleBlur
#
#   class CustomCropTransformer:
#       def __init__(self, config) -> None:
#           self.top = config.CUSTOM_CROP_TOP
#           self.bottom = config.CUSTOM_CROP_BOTTOM
#           self.left = config.CUSTOM_CROP_LEFT
#           self.right = config.CUSTOM_CROP_RIGHT
#           self.crop = ImgCropMask(self.left, self.top, self.right, self.bottom)
#           self.blur = ImgSimpleBlur()
#
#       def run(self, image):
#           image = self.crop.run(image)
#           return self.blur.run(image)
#   ```
#
AUGMENTATIONS = []         # changes to image only applied in training to create
                           # more variety in the data.
TRANSFORMATIONS = []       # changes applied _before_ training augmentations,
                           # such that augmentations are applied to the transformed image,
POST_TRANSFORMATIONS = []  # transformations applied _after_ training augmentations,
                           # such that changes are applied to the augmented image

# Settings for brightness and blur, use 'MULTIPLY' and/or 'BLUR' in
# AUGMENTATIONS
AUG_BRIGHTNESS_RANGE = 0.2  # this is interpreted as [-0.2, 0.2]
AUG_BLUR_RANGE = (0, 3)

# "CROP" Transformation
# Apply mask to borders of the image
# defined by a rectangle.
# If these crops values are too large, they will cause the stride values to
# become negative and the model with not be valid.
# # # # # # # # # # # # #
# xxxxxxxxxxxxxxxxxxxxx #
# xxxxxxxxxxxxxxxxxxxxx #
# xx                 xx # top
# xx                 xx #
# xx                 xx #
# xxxxxxxxxxxxxxxxxxxxx # bottom
# xxxxxxxxxxxxxxxxxxxxx #
# # # # # # # # # # # # #
ROI_CROP_TOP = 45               # the number of rows of pixels to ignore on the top of the image
ROI_CROP_BOTTOM = 0             # the number of rows of pixels to ignore on the bottom of the image
ROI_CROP_RIGHT = 0              # the number of rows of pixels to ignore on the right of the image
ROI_CROP_LEFT = 0               # the number of rows of pixels to ignore on the left of the image

# "TRAPEZE" tranformation
# Apply mask to borders of image
# defined by a trapezoid.
# # # # # # # # # # # # # #
# xxxxxxxxxxxxxxxxxxxxxxx #
# xxxx ul     ur xxxxxxxx # min_y
# xxx             xxxxxxx #
# xx               xxxxxx #
# x                 xxxxx #
# ll                lr xx # max_y
# # # # # # # # # # # # # #
ROI_TRAPEZE_LL = 0
ROI_TRAPEZE_LR = 160
ROI_TRAPEZE_UL = 20
ROI_TRAPEZE_UR = 140
ROI_TRAPEZE_MIN_Y = 60
ROI_TRAPEZE_MAX_Y = 120

# "CANNY" Canny Edge Detection tranformation
CANNY_LOW_THRESHOLD = 60    # Canny edge detection low threshold value of intensity gradient
CANNY_HIGH_THRESHOLD = 110  # Canny edge detection high threshold value of intensity gradient
CANNY_APERTURE = 3          # Canny edge detect aperture in pixels, must be odd; choices=[3, 5, 7]

# "BLUR" transformation (not this is SEPARATE from the blur augmentation)
BLUR_KERNEL = 5        # blur kernel horizontal size in pixels
BLUR_KERNEL_Y = None   # blur kernel vertical size in pixels or None for square kernel
BLUR_GAUSSIAN = True   # blur is gaussian if True, simple if False

# "RESIZE" transformation
RESIZE_WIDTH = 160     # horizontal size in pixels
RESIZE_HEIGHT = 120    # vertical size in pixels

# "SCALE" transformation
SCALE_WIDTH = 1.0      # horizontal scale factor
SCALE_HEIGHT = None    # vertical scale factor or None to maintain aspect ratio

#Model transfer options
#When copying weights during a model transfer operation, should we freeze a certain number of layers
#to the incoming weights and not allow them to change during training?
FREEZE_LAYERS = False               #default False will allow all layers to be modified by training
NUM_LAST_LAYERS_TO_TRAIN = 7        #when freezing layers, how many layers from the last should be allowed to train?

#For the categorical model, this limits the upper bound of the learned throttle
#it's very IMPORTANT that this value is matched from the training PC config.py and the robot.py
#and ideally wouldn't change once set.
MODEL_CATEGORICAL_MAX_THROTTLE_RANGE = 0.8

#RNN or 3D
SEQUENCE_LENGTH = 3             #some models use a number of images over time. This controls how many.
