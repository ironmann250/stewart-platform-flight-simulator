from ctypes import *
import time,com_serial,utils
import joystick_controller 
#define and initialize devices
inverse_kinematics_solver = CDLL("./inverse_kinematics_lib/RSS6RBT_InverseDLL.dll")
inverse_kinematics_results = (c_double*6)()
arduino_com=com_serial.Com_serial("COM6",9600)
arduino_com.start()
user_inputs=joystick_controller.XboxController()

#define constants and global vars
LIMIT_YAW=[-17,17]
LIMIT_PITCH=[-13,13]
LIMIT_ROLL=[-13,13]
OPERATING_Z=194
DELAY_TIME=1
#main loop

while True:
    #get user input
    yaw=1#user_inputs.LeftJoystickX*LIMIT_YAW[1]
    pitch=1#user_inputs.LeftJoystickY*LIMIT_PITCH[1]
    roll=1#user_inputs.RightJoystickX*LIMIT_ROLL[1]
    for i in range(-100,100,20):
        y=i
        #calculate inverse kinematics solution
        ret=inverse_kinematics_solver.InK6RSS(c_double(1), c_double(y), c_double(OPERATING_Z), c_double(yaw), c_double(pitch), c_double(roll), pointer(inverse_kinematics_results))
        
        #if results are valid send it to the arduino
        command=utils.make_command(inverse_kinematics_results,ret)

        if command:

            arduino_com.send(command)
        
        print(yaw,pitch,roll)
        print(command)
        time.sleep(DELAY_TIME)
    






