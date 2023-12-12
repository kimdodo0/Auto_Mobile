import RPi.GPIO as GPIO
import time

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 모터 핀 설정
motor1 = 17
motor2 = 18
motor3 = 22
motor4 = 23

# PWM 핀 설정
pwm1_pin = 12
pwm2_pin = 13

# 초음파 센서 핀 설정
TRIG_left = 9
ECHO_left = 25

TRIG_front = 19
ECHO_front = 16

TRIG_right = 11
ECHO_right = 8

# 모터 및 초음파 센서 핀 초기화
GPIO.setup(motor1, GPIO.OUT)  # in1
GPIO.setup(motor2, GPIO.OUT)  # in2
GPIO.setup(motor3, GPIO.OUT)  # in4
GPIO.setup(motor4, GPIO.OUT)  # in3

GPIO.setup(pwm1_pin, GPIO.OUT)
pwm_L = GPIO.PWM(pwm1_pin, 50)
pwm_L.start(3.0)

GPIO.setup(pwm2_pin, GPIO.OUT)
pwm_R = GPIO.PWM(pwm2_pin, 50)
pwm_R.start(3.0)

GPIO.setup(TRIG_front, GPIO.OUT)
GPIO.setup(ECHO_front, GPIO.IN)
GPIO.setup(TRIG_left, GPIO.OUT)
GPIO.setup(ECHO_left, GPIO.IN)
GPIO.setup(TRIG_right, GPIO.OUT)
GPIO.setup(ECHO_right, GPIO.IN)

GPIO.output(TRIG_front, GPIO.LOW)
GPIO.output(TRIG_left, GPIO.LOW)
GPIO.output(TRIG_right, GPIO.LOW)


# 초음파 거리 측정 함수
def dist_check(TRIG, ECHO):
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


# 모터 제어 함수
def motor_control(dist_func_front, dist_func_left, dist_func_right):
    try:
        GPIO.output(motor1, GPIO.HIGH)
        GPIO.output(motor2, GPIO.LOW)  # 오른쪽 바퀴 정방향
        GPIO.output(motor3, GPIO.LOW)
        GPIO.output(motor4, GPIO.HIGH)  # 왼쪽 바퀴 정방향

        pwm_L.ChangeDutyCycle(30.0)
        pwm_R.ChangeDutyCycle(30.0)

        if dist_func_front < 10:
            if dist_func_left < 10:
                pwm_L.ChangeDutyCycle(15)
                pwm_R.ChangeDutyCycle(70)
            elif dist_func_left < 5:
                pwm_R.ChangeDutyCycle(90)
            elif dist_func_right < 10:
                pwm_L.ChangeDutyCycle(70)
                pwm_R.ChangeDutyCycle(15)
            elif dist_func_right < 5:
                pwm_L.ChangeDutyCycle(90)

        else:
            pwm_L.ChangeDutyCycle(30)
            pwm_R.ChangeDutyCycle(30)

        time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()


while 1:
    dist_L = dist_check(TRIG_left, ECHO_left)
    dist_R = dist_check(TRIG_right, ECHO_right)
    dist_F = dist_check(TRIG_front, ECHO_front)

    motor_control(dist_F, dist_L, dist_R)