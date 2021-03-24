from jetracer.nvidia_racecar import NvidiaRacecar
import numpy as np
import time

STEERING_OFFSET = 0.1
STEERING_GAIN = -0.35


def main():
    car = NvidiaRacecar()
    car.steering_offset = STEERING_OFFSET
    car.steering_gain = STEERING_GAIN
    for s in np.arange(-1.0, 1.1, 0.1):
        car.steering = s
        print(s)
        time.sleep(1)


if __name__ == "__main__":
    main()
