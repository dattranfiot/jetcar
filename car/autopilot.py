#!/usr/bin/env python3

from components.actuator import PCA9685, PWMSteering, PWMThrottle
from components.camera import CSICamera
from components.joystick import PS4JoystickController
from components.tub_v2 import TubWriter
from components.web import WebFpv
import getpass
from car import vehicle
import datetime


class DriveMode:
    """ Helper class to dispatch between ai and user driving"""

    def __init__(self):
        self.AI_THROTTLE_MULT = 0.5
        pass

    def run(self, mode, user_angle, user_throttle, pilot_angle, pilot_throttle):
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle if pilot_angle else 0.0, user_throttle
        else:
            return pilot_angle if pilot_angle else 0.0, \
                   pilot_throttle * self.AI_THROTTLE_MULT if \
                       pilot_throttle else 0.0


class PilotCondition:
    """ Helper class to determine how is in charge of driving"""

    def run(self, mode):
        return mode != 'user'


def drive(tub_path=None, model_path=None, model_type=None):
    from os import listdir, makedirs
    from os.path import isfile, join, isdir, exists
    """
    Construct a minimal robotic vehicle from many parts. Here, we use a
    single camera, web or joystick controller, autopilot and tubwriter.

    Each part runs as a job in the Vehicle loop, calling either it's run or
    run_threaded method depending on the constructor flag `threaded`. All
    parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely
    manner. Parts may have named outputs and inputs. The framework handles
    passing named outputs to parts requesting the same named input.
    """

    car = vehicle.Vehicle()
    # add camera
    inputs = []
    cam = CSICamera(image_w=224, image_h=224,
                    capture_width=1080, capture_height=720,
                    image_d=3, framerate=30,
                    gstreamer_flip=0)

    car.add(cam,
            inputs=inputs,
            outputs=['cam/image_array'],
            threaded=True)

    # add controller
    # JOYSTICK
    JOYSTICK_MAX_THROTTLE = 0.7         # this scalar is multiplied with the -1 to 1 throttle value to limit the maximum throttle. This can help if you drop the controller or just don't need the full speed available.
    JOYSTICK_STEERING_SCALE = 1.0       #some people want a steering that is less sensitve. This scalar is multiplied with the steering -1 to 1. It can be negative to reverse dir.
    AUTO_RECORD_ON_THROTTLE = False     # if true, we will record whenever throttle is not zero. if false, you must manually toggle recording with some other trigger. Usually circle button on joystick.
    JOYSTICK_DEADZONE = 0.0             # when non zero, this is the smallest throttle before recording triggered.
    JOYSTICK_THROTTLE_DIR = -1.0        # use -1.0 to flip forward/backward, use 1.0 to use joystick's natural forward/backward
    ctr = PS4JoystickController(throttle_dir=JOYSTICK_THROTTLE_DIR,
                                throttle_scale=JOYSTICK_MAX_THROTTLE,
                                steering_scale=JOYSTICK_STEERING_SCALE,
                                auto_record_on_throttle=AUTO_RECORD_ON_THROTTLE)

    ctr.set_deadzone(JOYSTICK_DEADZONE)
    car.add(ctr,
            inputs=['cam/image_array'],
            outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
            threaded=True)

    # pilot condition to determine if user or ai are driving
    car.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])

    # adding the auto-pilot

    # Choose what inputs should change the car.
    car.add(DriveMode(),
            inputs=['user/mode',
                    'user/angle',
                    'user/throttle',
                    'pilot/angle',
                    'pilot/throttle'],
            outputs=['angle', 'throttle'])

    # Drive train setup
    PCA9685_I2C_ADDR = 0x40
    PCA9685_I2C_BUSNUM = 0x01
    # STEERING
    STEERING_CHANNEL = 0  # channel on the 9685 pwm board 0-15
    STEERING_LEFT_PWM = 475  # pwm value for full left steering
    STEERING_RIGHT_PWM = 320  # pwm value for full right steering

    # THROTTLE
    THROTTLE_CHANNEL = 1  # channel on the 9685 pwm board 0-15
    THROTTLE_FORWARD_PWM = 420  # pwm value for max forward throttle
    THROTTLE_STOPPED_PWM = 360  # pwm value for no movement
    THROTTLE_REVERSE_PWM = 290  # pwm value for max reverse throttle

    steering_controller = PCA9685(STEERING_CHANNEL, PCA9685_I2C_ADDR,
                                  busnum=PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=STEERING_LEFT_PWM,
                           right_pulse=STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(THROTTLE_CHANNEL, PCA9685_I2C_ADDR,
                                  busnum=PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=THROTTLE_FORWARD_PWM,
                           zero_pulse=THROTTLE_STOPPED_PWM,
                           min_pulse=THROTTLE_REVERSE_PWM)

    car.add(steering, inputs=['angle'])
    car.add(throttle, inputs=['throttle'])

    # add tub to save data
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
    types = ['image_array', 'float', 'float', 'str']
    # do we want to store new records into own dir or append to existing
    if tub_path is None:
        tub_path = '/home/{}/datastore/{}/'.format(getpass.getuser(),
                                                   datetime.datetime.now().strftime('%y-%m-%d'))

        if not exists(tub_path):
            makedirs(tub_path)
            num = 0
        else:
            onlydirs = [f for f in listdir(tub_path) if isdir(join(tub_path, f))]
            num = len(onlydirs) + 1

        tub_path += '{}/'.format(num)

    tub_writer = TubWriter(base_path=tub_path, inputs=inputs, types=types)
    car.add(tub_writer,
            inputs=inputs,
            outputs=["tub/num_records"],
            run_condition='recording')

    # Use the FPV preview, which will show the cropped image output, or the full frame.
    USE_FPV = True
    if USE_FPV:
        car.add(WebFpv(), inputs=['cam/image_array'], threaded=True)

    # start the car
    # VEHICLE
    DRIVE_LOOP_HZ = 20  # the vehicle loop will pause if faster than this speed.
    car.start(rate_hz=DRIVE_LOOP_HZ)


if __name__ == '__main__':
    drive()
