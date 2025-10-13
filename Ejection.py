#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math


# Brain should be defined by default
brain=Brain()


# Robot configuration code
motor_6 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
potentiometer_a = Potentiometer(brain.three_wire_port.a)
bumper_b = Bumper(brain.three_wire_port.b)




# wait for rotation sensor to fully initialize
wait(30, MSEC)




# Make random actually random
def initializeRandomSeed():
   wait(100, MSEC)
   random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
   urandom.seed(int(random))
    
# Set random seed
initializeRandomSeed()




def play_vexcode_sound(sound_name):
   # Helper to make playing sounds from the V5 in VEXcode easier and
   # keeps the code cleaner by making it clear what is happening.
   print("VEXPlaySound:" + sound_name)
   wait(5, MSEC)


# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")


#endregion VEXcode Generated Robot Configuration


# ------------------------------------------
#
#   Project:      VEXcode Project
#   Author:       VEX
#   Created:
#   Description:  VEXcode V5 Python Project
#
# ------------------------------------------


# Library imports
from vex import *


# Begin project


def move_to_pos(setpoint):
   error = potentiometer_a.angle(PERCENT) - setpoint


   while (error != 0):
       motor_6.set_velocity(10, PERCENT)
       motor_6.spin(FORWARD)
       error = potentiometer_a.angle(PERCENT) - setpoint


bumperToggle = False


def eject():
   if (bumper_b.pressing()):
       bumperToggle = True
  
   if(bumperToggle):
       move_to_pos(0.001)
       bumperToggle = False


while True:
   move_to_pos(0.8)
   motor_6.set_velocity(0.01, PERCENT)
   eject()
   wait(2, SECONDS)

