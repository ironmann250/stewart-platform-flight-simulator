from ctypes import *
import time,serial_interface,utils,flightgear_interface
import joystick_controller 
from multiprocessing import Process, freeze_support
#define and initialize devices
inverse_kinematics_solver = CDLL("./libs/RSS6RBT_InverseDLL.dll")
inverse_kinematics_results = (c_double*6)()
arduino_com=serial_interface.Com_serial("COM6",115200,timeout=0.2)
arduino_com.start()
arduino_com.send("test")
fg_com=flightgear_interface.FG_com()
fg_com.start()
fg_com.connect_and_wait_until_ready()
#user_inputs=joystick_controller.XboxController() #no need pass it to fg


#define constants and global vars
LIMIT_YAW=[-17,17]
LIMIT_PITCH=[-13,13]
LIMIT_ROLL=[-13,13]
OPERATING_Z=194
DELAY_TIME=1
newpitch=0
errortolerance=0.7

time.sleep(15) #wait for everything to sync

#main loop
# for i in range(15):
#     #print(arduino_com.read_position_blocking())
#     print(arduino_com.device.readline())
#     time.sleep(0.2)

#put platform at 0
ret=inverse_kinematics_solver.InK6RSS(c_double(0), c_double(0), c_double(OPERATING_Z), c_double(0), c_double(0), c_double(0), pointer(inverse_kinematics_results))    
command=utils.make_command(inverse_kinematics_results,ret)
arduino_com.send(command)
sign_flag2=1
pitch=-13
'''
give new and old angle
then move 0.5 deg to new angle
'''

while True:
    #read plane pitch
    freeze_support()
    # if pitch > 13:
    #     sign_flag2=-1
    # if pitch <-13:
    #     sign_flag2=1
    # pitch=pitch+sign_flag2
    plane_pitch=fg_com.get_param("pitch")
    plane_pitch=float(plane_pitch) if plane_pitch != None else 0
    print(plane_pitch)
    #plane_pitch=pitch
    oldpitch=newpitch
    newpitch=plane_pitch
    target=oldpitch
    # plane_roll=fg_com.get_param("roll")
    # plane_roll=float(plane_roll) if plane_roll != None else 0
    
    

    
    while (abs(abs(newpitch)-abs(target))>errortolerance):
        if newpitch<oldpitch:
            sign_flag=-0.25
        else:
            sign_flag=0.25
        target+=sign_flag
    #calculate inverse kinematics solution
        ret=inverse_kinematics_solver.InK6RSS(c_double(0), c_double(0), c_double(OPERATING_Z), c_double(0), c_double(target), c_double(0), pointer(inverse_kinematics_results))    
        #if results are valid send it to the arduino
        command=utils.make_command(inverse_kinematics_results,ret)
        #put platform at plane pitch
        if command:
                arduino_com.send(command)
                
        #print(arduino_com.read_position_blocking())
        #print(arduino_com.device.readline())
        print(command)
        print(pitch,target,newpitch,oldpitch,sign_flag,abs(abs(newpitch)-abs(target))>errortolerance)
        time.sleep(0.04)
    newpitch=target

# while True:
#     #get user input
#     yaw=1#user_inputs.LeftJoystickX*LIMIT_YAW[1]
#     pitch=1#user_inputs.LeftJoystickY*LIMIT_PITCH[1]
#     roll=1#user_inputs.RightJoystickX*LIMIT_ROLL[1]
#     for i in range(-100,100,20):
#         y=i
#         #calculate inverse kinematics solution
#         ret=inverse_kinematics_solver.InK6RSS(c_double(1), c_double(y), c_double(OPERATING_Z), c_double(yaw), c_double(pitch), c_double(roll), pointer(inverse_kinematics_results))
        
#         #if results are valid send it to the arduino
#         command=utils.make_command(inverse_kinematics_results,ret)

#         if command:
#             arduino_com.send(command)

#         #print(yaw,pitch,roll)
#         #print(command)
#         print(arduino_com.device.readline())
#         #time.sleep(DELAY_TIME)
    






