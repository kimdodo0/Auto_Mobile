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

        if distance > 300:
            distance = 100

        
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
    left_pwm.start(70)
    right_pwm.start(70)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.LOW)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.LOW)

def turn_left(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(motor1,GPIO.LOW)
    GPIO.output(motor2,GPIO.HIGH)
    GPIO.output(motor3,GPIO.HIGH)
    GPIO.output(motor4,GPIO.LOW)
    
def turn_right(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(motor1,GPIO.HIGH)
    GPIO.output(motor2,GPIO.LOW)
    GPIO.output(motor3,GPIO.LOW)
    GPIO.output(motor4,GPIO.HIGH)
    
def backward(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
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
        while True:
            time.sleep(0.03)
            
           
            forward(left_pwm,right_pwm)

            if dist[FRONT_DIST_IDX]<45:
                if dist[LEFT_DIST_IDX] > dist[RIGHT_DIST_IDX]:
                    turn_left(left_pwm,right_pwm)
                    time.sleep(0.3)
                elif dist[LEFT_DIST_IDX] < dist[RIGHT_DIST_IDX]:
                    turn_right(left_pwm,right_pwm)
                    time.sleep(0.3)

            if dist[LEFT_DIST_IDX] <1:
                turn_left(left_pwm,right_pwm)
                time.sleep(0.1)

            if dist[RIGHT_DIST_IDX]<1:
                turn_right(left_pwm,right_pwm)
                time.sleep(0.1)

                    
    except KeyboardInterrupt:
        GPIO.cleanup()
        exit(0)
    
    
    
    