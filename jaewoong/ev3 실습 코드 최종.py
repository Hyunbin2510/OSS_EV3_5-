#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time


wheel_diameter = 5.6
axle_track = 115

ev3 = EV3Brick()

ultra = UltrasonicSensor(Port.S3)
# color_sensor = ColorSensor(Port.S2)

shooting_motor = Motor(Port.C)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor,right_motor,wheel_diameter,axle_track)
ev3.speaker.beep()





##################### 1안 #######################
robot.drive_time(1, 0, abs(55 / 25 * 1000))
robot.stop() 
time.sleep(1)
# shooting_motor.dc(100)
# time.sleep(1)
# shooting_motor.dc(-100)
# time.sleep(2)
robot.stop()
while True:
    # 후진 300mm
    robot.drive(-1,0)  # 후진 300mm
    
    # 초음파 센서로 거리 측정
    distance = ultra.distance()  # 센서로 측정된 거리(mm)

    # 벽과 5cm 이하로 가까워지면 멈추기
    if distance < 60:  # 50mm = 5cm
        robot.stop()  # 로봇 멈추기
        time.sleep(2)
        robot.stop()
        break
        
        


while True:
    robot.drive_time(100, 0, abs(55 / 25 * 1000)) # 원래 값은 58 
    robot.stop() 
    time.sleep(1)
    shooting_motor.dc(100)
    robot.stop()
    time.sleep(1)
    shooting_motor.dc(-100)
    robot.stop()
    time.sleep(2)
    robot.stop()
    while True:
        # 후진 300mm
        robot.drive(-20,0)  # 후진 300mm
        
        # 초음파 센서로 거리 측정
        distance = ultra.distance()  # 센서로 측정된 거리(mm)

        # 벽과 5cm 이하로 가까워지면 멈추기
        if distance < 100:  # 50mm = 5cm
            robot.stop()  # 로봇 멈추기
            time.sleep(2)
            robot.stop()
            break

    time.sleep(10)





##################### 2안 #######################
num=0
while num<5:
    robot.drive_time(100, 0, abs(57 / 25 * 1000)) # 원래 값은 58 
    robot.stop() 
    time.sleep(3)
    shooting_motor.dc(100)
    robot.stop()
    time.sleep(1)
    shooting_motor.dc(-100)
    robot.stop()
    time.sleep(2)
    robot.stop()
    while True:
        # 후진 300mm
        robot.drive(-20,0)  # 후진 300mm
        
        # 초음파 센서로 거리 측정
        distance = ultra.distance()  # 센서로 측정된 거리(mm)

        # 벽과 5cm 이하로 가까워지면 멈추기
        if distance < 100:  # 50mm = 5cm
            robot.stop()  # 로봇 멈추기
            time.sleep(1)
            robot.stop()
            break

    time.sleep(10) # 계산시 한번왓다갓다 대기까지 해서 사이클 생각하면 20초 고 이를 5번 왓다갓다하면 100초 
    num+=1

robot.drive_time(100, 0, abs(57 / 25 * 1000)) # 원래 값은 58 
robot.stop()
while True: # 제한 시간 끝날때까지 가까이서 슈팅 계속함함
    time.sleep(3)
    shooting_motor.dc(100)
    robot.stop()
    time.sleep(1)
    shooting_motor.dc(-100)
    time.sleep(1)

# robot.drive_time(100, 0, abs(55 / 25 * 1500)) # 속도 높이고 직진 거리 감소시키는게 나을듯. 첫발에 너무 나가는 경향이 있음.
# robot.stop() 
# time.sleep(0.5) # 이부분은 0.1 ~ 1정도 고려. 공이 최대한 안정화 된후 공을 쏘게 하기 위해서
# shooting_motor.dc(100)
# time.sleep(0.5)
# shooting_motor.dc(-100)
# time.sleep(0.5)
# robot.drive_time(-100, 0, abs(90 / 25 * 600))
# robot.drive_time(-10, 0, abs(90 / 25 * 1000))

# # 속도가 빠른 버전
# while True:
#     robot.drive_time(100, 0, abs(40 / 25 * 1500)) # 속도 높이고 직진 거리 감소시키는게 나을듯. 첫발에 너무 나가는 경향이 있음.
#     robot.stop() 
#     time.sleep(0.5) # 이부분은 0.1 ~ 1정도 고려. 공이 최대한 안정화 된후 공을 쏘게 하기 위해서
#     shooting_motor.dc(100)
#     time.sleep(0.5)
#     shooting_motor.dc(-100)
#     time.sleep(0.5)
#     robot.drive_time(-100, 0, abs(90 / 25 * 600))
#     robot.drive_time(-10, 0, abs(90 / 25 * 1000))

    
