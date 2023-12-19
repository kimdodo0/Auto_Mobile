# 속도 감속 후 회전

from threading import Thread, Lock
import time

import RPi.GPIO as GPIO

forward_limit = 30
side_limit = 30
lock = Lock()

GPIO.setwarnings(False)
LEFT_MOTOR = 12
RIGHT_MOTOR = 13
LEFT_TRIG = 9
LEFT_ECHO = 25
FRONT_TRIG = 19
FRONT_ECHO = 16
RIGHT_TRIG = 11
RIGHT_ECHO = 8
motor3 = 17
motor4 =18
motor1 =23 
motor2 =22
LEFT_DIST_IDX = 0
FRONT_DIST_IDX = 1
RIGHT_DIST_IDX = 2

dist = [0 for i in range(3)]

def read_dist(TRIG, ECHO, dist_list, dist_idx):
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, GPIO.LOW)
    
    dist_hist = []
    
    while True:
        time.sleep(0.01)

        ### 교수님 코드
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

        if distance > 200 or distance < 0:
            continue

        
        if len(dist_hist) < 5:
            dist_hist.append(distance)
        else:
            dist_hist.pop(0)
            dist_hist.append(distance)
        
            sum = 0
            for dist in dist_hist:
                sum += dist

            min_1 = min(dist_hist)
            max_1 = max(dist_hist)
            
            avg = (sum - min_1 - max_1) / 3
            dist_list[dist_idx] = avg
            

def forward(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.LOW)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.LOW)

def turn_left(left_pwm,right_pwm):
    # left_pwm.start(20)
    # right_pwm.start(20)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.HIGH)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.HIGH)
    time.sleep(0.011)
    GPIO.output(motor1,GPIO.LOW)
    GPIO.output(motor2,GPIO.HIGH)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.LOW)
    left_pwm.start(55)
    right_pwm.start(55)
    
def turn_right(left_pwm,right_pwm):
    # left_pwm.start(20)
    # right_pwm.start(20)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.HIGH)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.HIGH)
    time.sleep(0.011)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.LOW)
    GPIO.output(motor3,GPIO.LOW)
    GPIO.output(motor4,GPIO.HIGH)
    left_pwm.start(52)
    right_pwm.start(52)
    
def backward(left_pwm,right_pwm):
    left_pwm.start(40)
    right_pwm.start(65)
    GPIO.output(motor1,GPIO.LOW)
    GPIO.output(motor2,GPIO.HIGH)
    GPIO.output(motor3,GPIO.LOW)
    GPIO.output(motor4,GPIO.HIGH)
    
if __name__ == '__main__':
    PWM_FREQ=1000
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor1,GPIO.OUT)
    GPIO.setup(motor2,GPIO.OUT)
    GPIO.setup(motor3,GPIO.OUT)
    GPIO.setup(motor4,GPIO.OUT)
    GPIO.setup(LEFT_MOTOR,GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR,GPIO.OUT)
    GPIO.output(motor1,GPIO.LOW)
    GPIO.output(motor3,GPIO.LOW)
    GPIO.output(motor2,GPIO.LOW)
    GPIO.output(motor4,GPIO.LOW)
    
    leftSonic_thread = Thread(target=read_dist, args=(LEFT_TRIG, LEFT_ECHO, dist, LEFT_DIST_IDX))
    frontSonic_thread = Thread(target=read_dist, args=(FRONT_TRIG, FRONT_ECHO, dist, FRONT_DIST_IDX))
    rightSonic_thread = Thread(target=read_dist, args=(RIGHT_TRIG, RIGHT_ECHO, dist, RIGHT_DIST_IDX))

    leftSonic_thread.start()
    frontSonic_thread.start()
    rightSonic_thread.start()


    GPIO.setup(LEFT_MOTOR, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR, GPIO.OUT)
    left_pwm = GPIO.PWM(LEFT_MOTOR, PWM_FREQ)
    right_pwm = GPIO.PWM(RIGHT_MOTOR, PWM_FREQ)
    

    
    left_pwm.start(50)
    right_pwm.start(50)
    
    try:   
        forward(left_pwm,right_pwm)
        time.sleep(0.3)
        while True:
            time.sleep(0.03)
            
           
            forward(left_pwm,right_pwm)

            if dist[FRONT_DIST_IDX]<50:
                if dist[LEFT_DIST_IDX] < dist[RIGHT_DIST_IDX] and dist[LEFT_DIST_IDX] < 15:
                    turn_right(left_pwm,right_pwm)
                    print('2')
                    time.sleep(0.12)
                elif dist[LEFT_DIST_IDX] > dist[RIGHT_DIST_IDX] and dist[RIGHT_DIST_IDX] < 15:
                    turn_left(left_pwm,right_pwm)
                    print('1')
                    time.sleep(0.12)
                

            if dist[FRONT_DIST_IDX]< 47 and dist[LEFT_DIST_IDX] < 40 and dist[RIGHT_DIST_IDX] <35:
                GPIO.output(motor1,GPIO.HIGH)
                GPIO.output(motor2,GPIO.HIGH)
                GPIO.output(motor3,GPIO.HIGH)
                GPIO.output(motor4,GPIO.HIGH)
                print('break')
                while dist[FRONT_DIST_IDX] < 30:
                    backward(left_pwm, right_pwm)
                    print('3')
                    turn_right(left_pwm, right_pwm)
                    time.sleep(0.1)
                    if dist[FRONT_DIST_IDX] < 5:
                        print('while break')
                        break
                    
                

            # if dist[LEFT_DIST_IDX] < 5 and dist[RIGHT_DIST_IDX] < 5:
            #         turn_right(left_pwm, right_pwm)
            #         print('3')
            #         time.sleep(0.5)
                

            elif dist[LEFT_DIST_IDX] <29:
                turn_right(left_pwm,right_pwm)
                print('4')
                time.sleep(0.15)

            elif dist[RIGHT_DIST_IDX]<23:
                turn_left(left_pwm,right_pwm)
                print('5')
                time.sleep(0.15)

                    
    except KeyboardInterrupt:
        GPIO.cleanup()
        exit(0)
    
    
    
    