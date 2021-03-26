import time

class CSICamera:
    '''
    Camera for Jetson Nano IMX219 based camera
    Credit: https://github.com/feicccccccc/donkeycar/blob/dev/donkeycar/parts/camera.py
    gstreamer init string from https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/camera.py
    '''

    def gstreamer_pipeline(self, capture_width=3280, capture_height=2464, output_width=224, output_height=224,
                           framerate=21, flip_method=0):
        return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), ' \
               'width=%d, ' \
               'height=%d, ' \
               'format=(string)NV12, ' \
               'framerate=(fraction)%d/1 ! nvvidconv ' \
               'flip-method=%d ! nvvidconv ! video/x-raw, ' \
               'width=(int)%d, ' \
               'height=(int)%d, ' \
               'format=(string)BGRx ! videoconvert ! appsink' % \
               (capture_width,
                capture_height,
                framerate,
                flip_method,
                output_width,
                output_height)

    def __init__(self, image_w=160, image_h=120, image_d=3, capture_width=3280, capture_height=2464, framerate=60,
                 gstreamer_flip=0):
        '''
        gstreamer_flip = 0 - no flip
        gstreamer_flip = 1 - rotate CCW 90
        gstreamer_flip = 2 - flip vertically
        gstreamer_flip = 3 - rotate CW 90
        '''
        self.w = image_w
        self.h = image_h
        self.running = True
        self.frame = None
        self.flip_method = gstreamer_flip
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.framerate = framerate

    def init_camera(self):
        import cv2

        # initialize the camera and stream
        self.camera = cv2.VideoCapture(
            self.gstreamer_pipeline(
                capture_width=self.capture_width,
                capture_height=self.capture_height,
                output_width=self.w,
                output_height=self.h,
                framerate=self.framerate,
                flip_method=self.flip_method),
            cv2.CAP_GSTREAMER)

        self.poll_camera()
        print('CSICamera loaded.. .warming camera')
        time.sleep(2)

    def update(self):
        self.init_camera()
        while self.running:
            self.poll_camera()

    def poll_camera(self):
        import cv2
        _, frame = self.camera.read()
        if frame is not None:
            self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def run(self):
        self.poll_camera()

        return self.frame

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        self.running = False
        print('stopping CSICamera')
        time.sleep(.5)
        del self.camera
