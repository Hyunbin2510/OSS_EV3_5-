#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()



# Write your program here.
'''
모터 정의
'''
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_arm = Motor(Port.B)
shooting_arm = Motor(Port.C)

wheel_diameter = 5.6
axle_track = 115
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

def motion1():
    '''
    팔을 최대한 위로 옮기기
    '''
    grab_arm.reset_angle(0)
    grab_arm.run_until_stalled(-200,duty_limit=30)
    grab_high_angle = grab_arm.angle()


def motion2():
    '''
    중간높이 올리기 (벽에 걸리지 않게)
    '''
    motion3()
    grab_arm.reset_angle(0)
    grab_arm.run_angle(100,-45)
def motion3():
    '''
    공을 잡기위해 팔을 최대한 아래로 옮기기
    '''
    grab_arm.run_until_stalled(200,duty_limit=30)
    grab_low_angle = grab_arm.angle()

motion1()
motion2()
# robot.straight(100)

