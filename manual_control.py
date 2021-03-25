import os
import array
import time
import struct
import logging


class Joystick(object):
    '''
    An interface to a physical joystick
    '''

    def __init__(self, dev_fn='/dev/input/js0'):
        self.axis_states = {}
        self.button_states = {}
        self.axis_names = {}
        self.button_names = {}
        self.axis_map = []
        self.button_map = []
        self.jsdev = None
        self.dev_fn = dev_fn

    def init(self):
        try:
            from fcntl import ioctl
        except ModuleNotFoundError:
            self.num_axes = 0
            self.num_buttons = 0
            print("no support for fnctl module. joystick not enabled.")
            return False

        if not os.path.exists(self.dev_fn):
            print(self.dev_fn, "is missing")
            return False

        '''
        call once to setup connection to device and map buttons
        '''
        # Open the joystick device.
        print('Opening %s...' % self.dev_fn)
        self.jsdev = open(self.dev_fn, 'rb')

        # Get the device name.
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
        self.js_name = buf.tobytes().decode('utf-8')
        print('Device name: %s' % self.js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
        self.num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        self.num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:self.num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:self.num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
            # print('btn', '0x%03x' % btn, 'name', btn_name)

        return True

    def show_map(self):
        '''
        list the buttons and axis found on this joystick
        '''
        print('%d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
        print('%d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

    def poll(self):
        '''
        query the state of the joystick, returns button which was pressed, if any,
        and axis which was moved, if any. button_state will be None, 1, or 0 if no changes,
        pressed, or released. axis_val will be a float from -1 to +1. button and axis will
        be the string label determined by the axis map in init.
        '''
        button = None
        button_state = None
        axis = None
        axis_val = None

        if self.jsdev is None:
            return button, button_state, axis, axis_val

        # Main event loop
        evbuf = self.jsdev.read(8)

        if evbuf:
            tval, value, typev, number = struct.unpack('IhBB', evbuf)

            if typev & 0x80:
                # ignore initialization event
                return button, button_state, axis, axis_val

            if typev & 0x01:
                button = self.button_map[number]
                # print(tval, value, typev, number, button, 'pressed')
                if button:
                    self.button_states[button] = value
                    button_state = value
                    logging.info("button: %s state: %d" % (button, value))

            if typev & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axis_states[axis] = fvalue
                    axis_val = fvalue
                    logging.debug("axis: %s val: %f" % (axis, fvalue))

        return button, button_state, axis, axis_val


class PS4Joystick(Joystick):
    '''
    An interface to a physical PS4 joystick available at /dev/input/js0
    '''

    def __init__(self, *args, **kwargs):
        super(PS4Joystick, self).__init__(*args, **kwargs)

        self.axis_names = {
            0x00: 'left_stick_horz',
            0x01: 'left_stick_vert',
            0x02: 'right_stick_horz',
            0x05: 'right_stick_vert',

            0x03: 'left_trigger_axis',
            0x04: 'right_trigger_axis',

            0x10: 'dpad_leftright',
            0x11: 'dpad_updown',

            0x19: 'tilt_a',
            0x1a: 'tilt_b',
            0x1b: 'tilt_c',

            0x06: 'motion_a',
            0x07: 'motion_b',
            0x08: 'motion_c',
        }

        self.button_names = {

            0x130: 'square',
            0x131: 'cross',
            0x132: 'circle',
            0x133: 'triangle',

            0x134: 'L1',
            0x135: 'R1',
            0x136: 'L2',
            0x137: 'R2',
            0x13a: 'L3',
            0x13b: 'R3',

            0x13d: 'pad',
            0x138: 'share',
            0x139: 'options',
            0x13c: 'PS',
        }


if __name__ == "__main__":
    # Testing the XboxOneJoystickController
    js = PS4Joystick('/dev/input/js0')
    js.init()

    while True:
        button, button_state, axis, axis_val = js.poll()
        if button is not None or axis is not None:
            print(button, button_state, axis, axis_val)
            time.sleep(0.01)
