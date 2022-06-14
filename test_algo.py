from time import sleep
sign_flag2=1
pitch=-13
newpitch=0
errortolerance=0.1
while 1:
    if pitch > 13:
        sign_flag2=-1
    if pitch <-13:
        sign_flag2=1
    pitch=pitch+sign_flag2
    plane_pitch=pitch
    oldpitch=newpitch
    newpitch=plane_pitch
    target=oldpitch

    while (abs(abs(newpitch)-abs(target))>errortolerance):
        if newpitch<oldpitch:
            sign_flag=-0.1
        else:
            sign_flag=0.1
        target+=sign_flag

        print(pitch,target,newpitch,oldpitch,sign_flag,abs(abs(newpitch)-abs(target))>errortolerance)

        sleep(0.05)
    newpitch=target
