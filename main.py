#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.robotics import DriveBase

# 11111 
# run_motor = Motor(Port.A)

# run_motor.run_time(30)
# wait(1000)
# run_motor.run_time(-30)
# wait(1000)



# run_motor.reset_angle(0)
# run_motor.run(100)

# if <720



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# 맨 윗줄 주석 아니니 지우면 안됨. 지우면 실행이 되지 않음.

# Create your objects here.
# ev3 = EV3Brick()

# class Motor(port,positive_direction = Direction.CLOCKWISE,gears = NONE)

# run_motor = Motor(Port.A)
# ev3.run_motor(Port.A)

# run_motor.run(200)


# ts = TouchSensor(Port.S1)

# while True:
#     if ts.pressed():
#         while ts.pressed():
#             pass
#     print("터치센서 눌림")

# ev3 = EV3Brick()

# # ts = TouchSensor(Port.S1)
# # while True:
# #     if ts.pressed():
# #         while ts.pressed():
# #             pass
# #     ev3.speaker.beep()


# ultra = UltrasonicSensor(Port.S3)

# distance = ultra.distance()
# print(distance)

# presence = ultra.presence()
# print(presence)


ev3 = EV3Brick()
ev3.speaker.beep()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

wheel_diameter = 5.6
axle_track = 115

robot = DriveBase(left_motor,right_motor,wheel_diameter,axle_track)

shooting_motor = Motor(Port.C)
grab_motor = Motor(Port.B)

grab_motor.run_until_stalled(500, stop_type=Stop.HOLD, duty_limit=75) 
grab_motor.reset_angle()  # 현재 각도를 기준으로 0으로 초기화

robot.straight(100)

grab_motor.run_target(speed=500, target_angle=-90, wait=True)  

robot.straight(-50)

robot.turn(90)

shooting_motor.run(1000)  # 빠르게 모터 회전
wait(500)                # 0.5초 동안 동작
shooting_motor.stop()    # 모터 정지
