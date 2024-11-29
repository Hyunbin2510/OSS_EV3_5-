#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
#모터 % 센서 선언
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_arm = Motor(Port.B)
shooting_arm = Motor(Port.C)
gyro = GyroSensor(Port.S1)

#바퀴크기와 지름?
wheel_diameter = 5.6
axle_track = 115
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
gyro.reset_angle(0)

def motion1():
    grab_arm.run_until_stalled(-180, duty_limit=30)
def motion3():
    grab_arm.run_until_stalled(180, duty_limit=50)
def motion2():
    grab_arm.reset_angle(0)
def motion4():
    shooting_arm.run_until_stalled(-100, duty_limit=50)
    shooting_arm.reset_angle(0)
def motion5():
    shooting_arm.run(2000)
    time.sleep(0.25)
    shooting_arm.stop()




motion3()
motion2()
motion1()

motion4()
motion5()
motion4()

