from gpiozero import Motor
import RPi.GPIO as GPIO
import time
import threading

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

dist_result = [30, 30, 30]
# 종료 이벤트
exit_event = threading.Event()

# Lock 객체 생성
gpio_lock = threading.Lock()

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

def front():
    # print('front')
    GPIO.output(motor1, GPIO.HIGH)
    GPIO.output(motor2, GPIO.LOW)  # 오른쪽 바퀴 정방향
    GPIO.output(motor3, GPIO.LOW)
    GPIO.output(motor4, GPIO.HIGH)  # 왼쪽 바퀴 정방향
    pwm_L.ChangeDutyCycle(30.0)
    pwm_R.ChangeDutyCycle(30.0)
    # time.sleep(0.5)

def reverse():
    print('reverse')
    GPIO.output(motor2, GPIO.HIGH)
    GPIO.output(motor1, GPIO.LOW)  # 오른쪽 바퀴 역방향
    GPIO.output(motor4, GPIO.LOW)
    GPIO.output(motor3, GPIO.HIGH)  # 왼쪽 바퀴 역방향
    pwm_L.ChangeDutyCycle(30)
    pwm_R.ChangeDutyCycle(30)
    time.sleep(0.5)

def right():
    print('right')
    GPIO.output(motor2, GPIO.HIGH)
    GPIO.output(motor1, GPIO.LOW)  # 오른쪽 바퀴 역방향
    GPIO.output(motor3, GPIO.LOW)
    GPIO.output(motor4, GPIO.HIGH)  # 왼쪽 바퀴 정방향
    pwm_L.ChangeDutyCycle(30)
    pwm_R.ChangeDutyCycle(30)
    time.sleep(0.5)

def left():
    print('left')
    GPIO.output(motor1, GPIO.HIGH)
    GPIO.output(motor2, GPIO.LOW)  # 오른쪽 바퀴 정방향
    GPIO.output(motor4, GPIO.LOW)
    GPIO.output(motor3, GPIO.HIGH)  # 왼쪽 바퀴 역방향
    pwm_L.ChangeDutyCycle(30)
    pwm_R.ChangeDutyCycle(30)
    time.sleep(0.5)
  
# 모터 제어 함수
def motor_control(dist_result):
    try:
        while not exit_event.is_set():
            with gpio_lock:

                # GPIO.output(motor1, GPIO.HIGH)
                # GPIO.output(motor2, GPIO.LOW)  # 오른쪽 바퀴 정방향
                # GPIO.output(motor3, GPIO.LOW)
                # GPIO.output(motor4, GPIO.HIGH)  # 왼쪽 바퀴 정방향

                # pwm_L.ChangeDutyCycle(20.0)
                # pwm_R.ChangeDutyCycle(20.0)
                if dist_result[0] > 200:
                    dist_result[0] = 40
                
                # if dist_result[0]< 350 and dist_result[1] < 350 and dist_result[2]< 350:
                front()
                # dist_result = [front, left, right]
                # if  0 < dist_result[0] < 20:
                if (dist_result[0] < 10 and dist_result[1] < 10) or (dist_result[0] < 10 and dist_result[2] < 10)  : 
                    reverse()                  

                elif dist_result[1] > dist_result[2]:
                    left()
                # elif dist_result[1] < 10:
                #     pwm_R.ChangeDutyCycle(50)
                elif dist_result[2] > dist_result[1]:
                    right()
                # elif dist_result[2] < 10:
                #     pwm_L.ChangeDutyCycle(50)


                # else:
                #     front()
                    # pwm_L.ChangeDutyCycle(30)
                    # pwm_R.ChangeDutyCycle(30)

            # time.sleep(1)


    except KeyboardInterrupt:
        GPIO.cleanup()

# 거리 측정 함수
def distance_measurement_front():
    try:
        while not exit_event.is_set():
            result_front = dist_check(TRIG_front, ECHO_front)
            print('front distance = %.2f cm' % (result_front))
            dist_result[0] = result_front
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass

def distance_measurement_left():
    try:
        while not exit_event.is_set():
            result_left = dist_check(TRIG_left, ECHO_left)
            print('left distance = %.2f cm' % (result_left))
            dist_result[1] = result_left
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
        
def distance_measurement_right():
    try:
        while not exit_event.is_set():
            result_right = dist_check(TRIG_right, ECHO_right)
            print('right distance = %.2f cm' % (result_right))
            dist_result[2] = result_right
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass

# 스레드 생성 및 실행
distance_thread_front = threading.Thread(target=distance_measurement_front)
distance_thread_left = threading.Thread(target=distance_measurement_left)
distance_thread_right = threading.Thread(target=distance_measurement_right)
distance_thread_front.daemon = True
distance_thread_left.daemon = True
distance_thread_right.daemon = True
# motor_thread = threading.Thread(target=motor_control,
#                                 args=(distance_measurement_front,
#                                        distance_measurement_left,
#                                        distance_measurement_right))


# motor_thread.start()
distance_thread_front.start()
distance_thread_left.start()
distance_thread_right.start()

while True:
    motor_control(dist_result)

    try:
        # 메인 스레드는 종료 이벤트를 대기
        while True:
            time.sleep(1)
            # 종료 이벤트 설정 후 스레드 종료 대기
            exit_event.set()
            # motor_thread.join()
            distance_thread_right.join()
            distance_thread_front.join()
            distance_thread_left.join()

    except KeyboardInterrupt:
        GPIO.cleanup()
        break

