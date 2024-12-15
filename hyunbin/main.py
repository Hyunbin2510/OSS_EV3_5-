#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S1)
ser = UARTDevice(Port.S2, baudrate=115200)

#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)
robot.settings(straight_speed = 200)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    angle = gyro.angle()%360
    print('robot turn')
    robot.straight(-70)
    if 0 <= anlge < 180:
        robot.drive(0, power)
    else:
        robot.drive(0,-power)

    while True:
        
        angle = gyro.angle()%360
        print(angle)
        if angle <= target_angle+2:
            robot.stop()
            break

#==========[camera_chase]==========
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
                
        # 1. 각 리스트를 추출하기 위해 "]["를 기준으로 분리
        split_lists = data_str.split('][')

        # 2. 첫 번째와 마지막 리스트의 대괄호를 보정
        split_lists[0] = split_lists[0].lstrip('[')
        split_lists[-1] = split_lists[-1].rstrip(']')

        # 3. 각 리스트의 문자열을 정수형으로 변환하여 리스트로 생성
        result = [list(map(int, lst.split(','))) for lst in split_lists]

        parsed_list = result[0]

        return parsed_list
    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

#==========[shooting positions]==========
def grab(command):
    if command == 'motion3':
        #close
        grab_motor.run_until_stalled(500,Stop.COAST,duty_limit=30)
        #set_zero point
        grab_motor.reset_angle(0)
    elif command == 'motion1':
        #open1
        grab_motor.run_until_stalled(-500,Stop.COAST,duty_limit=30)
    elif command == 'motion2':
        #open2
        grab_motor.run_target(500,-100)
    elif command == 'motion4':
        grab_motor.run_until_stalled(500,Stop.COAST,duty_limit=30)

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-500,Stop.COAST,duty_limit=30)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(6000)
        time.sleep(0.4)
        shooting_motor.stop()



#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0
gyro.reset_angle(0)
#==========[zero set position setting]==========
shoot('zero') #shoot 모터가 안쪽이고,
grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(0.1)
grab('motion1') #공을 잡기 위한 높이로 열기
grab('motion2')

print("Zero set postion completed")

#==========[test loop]==========

#==========[main loop]==========
iteration = 0
while True:
    data = ser.read_all()
    print(data)
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data)
        print(filter_result)
        #filter_result[0] : x, filter_result[1] : y
        if filter_result[0]!= -1 and filter_result[1]!= -1:
        # if filter_result[0]!= -1 and filter_result[1]!= -1:
            if filter_result[1] > 90:
                print(filter_result)
                robot.straight(85) #강제로 앞으로 이동
                grab('motion4') 
                robot.straight(-70)
                robot.turn(20)
                time.sleep(0.2) #동작간 딜레이
                grab('motion1') #슛을 위한 열기
                time.sleep(0.1) #동작간 딜레이
                shoot('shoot') #공 날리기
                time.sleep(0.1) #동작간 딜레이
                shoot('zero')
                grab('motion2')
                robot.turn(-20) 
            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.5, kd=0.1, power=150)
                
        else: # 센서가 공을 보지 못했을 경우의 움직임.
            # todo 알고리즘 추가하기
            if iteration%5 == 0:
                robot.straight(100)
            elif iteration%5 == 1:
                robot.turn(30)
                time.sleep(0.1)
            elif iteration%5 == 2:
                robot.turn(-30)
                
            elif iteration%5 == 3:
                robot.turn(-30)
                time.sleep(0.1)
            elif iteration%5 == 4:
                robot.turn(30)
            iteration += 1
            print(iteration)
        time.sleep_ms(50)
    except:
        pass

        

