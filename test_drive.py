from jetracer.nvidia_racecar import NvidiaRacecar
import time

STEERING_OFFSET = 0.1
STEERING_GAIN = -0.35
THROTTLE_GAIN = -0.2


def main():
    car = NvidiaRacecar()
    car.throttle_gain = THROTTLE_GAIN
    car.steering_offset = STEERING_OFFSET
    car.steering_gain = STEERING_GAIN
    car.throttle = 0
    car.steering = 0
    while True:
        time.sleep(0.05)


if __name__ == "__main__":
    main()
