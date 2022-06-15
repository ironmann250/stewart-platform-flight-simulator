#test arduino stewart platfform control
#flash arduino with code at ./arduino/platform_control/servo_only.ino
#if you add Serial.println in arduino you can read it as in this test code
import serial_interface,time,utils
from ctypes import *

#inverse kinematics model
inverse_kinematics_solver = CDLL("./libs/RSS6RBT_InverseDLL.dll")
inverse_kinematics_results = (c_double*6)()

#arduino serial com
arduino_com=serial_interface.Com_serial("COM6",115200,timeout=0.2)
arduino_com.start() #this will delay for 2 sec to sync

arduino_com.send("180,180,180,180,180,180") #go to lowest
some_String=arduino_com.device.read() #read data from the Arduino and skip if nothing is sent
time.sleep(5) #wait 5 sec 
#some_string=arduino_com.read_position_blocking() #read from the arduino and wait for data to be sent
x=0
y=0
OPERATING_Z=194
yaw=0
pitch=0
roll=0
ret=inverse_kinematics_solver.InK6RSS(c_double(x), c_double(y), c_double(OPERATING_Z), c_double(yaw), c_double(pitch), c_double(roll), pointer(inverse_kinematics_results))    
command=utils.make_command(inverse_kinematics_results,ret)
if command: #if the command is valid (not None)
    arduino_com.send(command)
