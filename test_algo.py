import visualize_3d_rotation
from time import sleep
sign_flag2=1
pitch=-13
newpitch=0
newroll=0
newyaw=0
errortolerance=0.3
plane_pitch=0
plane_roll=0
plane_yaw=0
while 1:
    if plane_pitch > 13:
        sign_flag2=-1
    if plane_pitch <-13:
        sign_flag2=1
        
    plane_pitch=plane_pitch+sign_flag2
    plane_roll=plane_roll+sign_flag2
    plane_yaw=plane_yaw+sign_flag2
    
    oldpitch=newpitch
    newpitch=plane_pitch
    targetpitch=oldpitch

    oldroll=newroll
    newroll=plane_roll
    targetroll=oldroll

    oldyaw=newyaw
    newyaw=plane_yaw
    targetyaw=oldyaw

    while ((abs(abs(newpitch)-abs(targetpitch))>errortolerance) and
           (abs(abs(newroll)-abs(targetroll))>errortolerance) and
           (abs(abs(newyaw)-abs(targetyaw))>errortolerance)):#see if we reached the target yaw #still testing yaw
        if newpitch<oldpitch:
            sign_flag=-0.5
        else:
            sign_flag=0.5
        targetpitch+=sign_flag

        if newroll<oldroll:
            sign_flag=-0.5
        else:
            sign_flag=0.5
        targetroll+=sign_flag

        if newyaw<oldyaw:
            sign_flag=-0.5
        else:
            sign_flag=0.5
        targetyaw+=sign_flag
        print(plane_pitch,targetpitch,plane_roll,targetroll,plane_yaw,targetyaw)
        visualize_3d_rotation.update(targetroll,targetpitch,targetyaw,debug=False)
        sleep(0.05)
    newpitch=targetpitch
    newroll=targetroll
    newyaw=targetyaw
