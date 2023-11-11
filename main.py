from time import time, sleep
import cv2
import threading
from math import inf
import RPi.GPIO as GPIO


class Drone:

    def __init__(self, ip_drone, ip_cam):

        self.cap = cv2.VideoCapture()#f'rtsp://hackaton1:hackaton2023@{ip_cam}/stream2'
        self.ip = ip_drone

    def get_frame(self):

        _, frame = self.cap.read()
        return frame

    def down(self):
        pass

    def up(self):
        pass

    def takeoff(self):
        pass


class Camera:

    def __init__(self, cam_id):

        self.cap = cv2.VideoCapture(cam_id)

    def get_frame(self):

        _, frame = self.cap.read()
        return frame

    def get_road_segmentation(self):

        frame = self.get_frame()

        # TODO: Прикрутить YOLO, UNET или CV2

        return 0

    def get_aruco_markers(self):

        frame = self.get_frame()

        # TODO: Тут поиск маркеров

        return 0


class Motor:    # Управление мотором

    def __init__(self, forward_pin, backward_pin):  # Инициализируем порты

        GPIO.setup((forward_pin, backward_pin), GPIO.OUT, initial=GPIO.LOW)

        self.__forward_pin = forward_pin
        self.__backward_pin = backward_pin

        self.__forward_speed = 0
        self.__backward_speed = 0
        self.__motor_pwm_period = .001

        self.__max_speed = 1

        self.__pwm_forward = threading.Thread(target=self.pwm_on_pin, args=(self.__forward_pin, 1))  # Сохраняем ШИМ в отдельный поток для управления
        self.__pwm_backward = threading.Thread(target=self.pwm_on_pin, args=(self.__backward_pin, -1))  # Сохраняем ШИМ в отдельный поток для управления

        self.__pwm_forward.start()
        self.__pwm_backward.start()

    def pwm_on_pin(self, pin, direction):  # Подаем нулевой ШИМ


        while True:

            speed = self.__forward_speed if direction > 0 else self.__backward_speed

            print(f'forward_speed: {self.__forward_speed} | backward_speed: {self.__backward_speed}')

            if speed > 0:
                GPIO.output(pin, GPIO.HIGH)
                sleep(max(self.__motor_pwm_period * speed, 0))

                GPIO.output(pin, GPIO.LOW)
                sleep(max(self.__motor_pwm_period * (1 - speed), 0))
            else:
                sleep(self.__motor_pwm_period)

    def set_forward_speed(self, speed=1):
        self.__forward_speed = speed

    def set_backward_speed(self, speed=1):
        self.__backward_speed = speed

    def forward(self, speed):
        self.set_forward_speed(speed)
        self.set_backward_speed(0)

    def backward(self, speed):
        self.set_forward_speed(0)
        self.set_backward_speed(speed)

    def stop(self):
        self.set_forward_speed(0)
        self.set_backward_speed(0)

    def get_pwm(self):
        return self.__pwm


class Rower:

    def __init__(self, *args, speed=1, left_motor, right_motor, camera, drone):

        self.speed = speed
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.camera = camera
        self.drone = drone

    def forward(self, speed, duration=inf):

        start_time = time()

        self.left_motor.forward(speed)
        self.right_motor.forward(speed)

        i = 0
        while start_time + duration > time():

            self.camera.get_frame()

            if i % 5 == 0:
                # TODO: Обработчик кадров
                pass

            i += 1

    def turn(self, degrees):

        start_time = time()
        while start_time + (3 * (abs(degrees) / 90)) > time():

            if degrees < 0:
                self.left_motor.backward(self.speed)
                self.right_motor.formard(self.speed)

            else:
                self.left_motor.formard(self.speed)
                self.right_motor.backward(self.speed)

    def backward(self):
        pass

    def ask_drone(self):
        pass

    def give_container(self):
        self.drone.takeoff()


if __name__ == '__main__':

    GPIO.setmode(GPIO.BOARD)

    l_motor = Motor(forward_pin=11, backward_pin=13)
    r_motor = Motor(forward_pin=15, backward_pin=19)

    cam = Camera(0)

    copter = Drone('192.168.0.0', '192.168.88.244')

    rower = Rower(left_motor=l_motor, right_motor=r_motor, camera=cam, drone=copter)

    rower.forward(.5, 5)
    rower.turn(10)
    rower.forward(.9, 10)

    while True:
        pass
        break

    GPIO.cleanup()


# TODO: Для мотора (шим и продолжительность)







