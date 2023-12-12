import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 17
ECHO = 27
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, GPIO.LOW)


def dist_check():
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
    stop = 0
    start = 0
    while GPIO.input(ECHO) == GPIO.LOW:
        start = time.time()
    while GPIO.input(ECHO) == GPIO.HIGH:
        stop = time.time()
    duration = stop - start
    distance = (duration * 340 * 100) / 2
    return distance


try:
    while True:
        result_dis = dist_check()
        print('distance = %.2f cm' % (result_dis))
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanuup()



