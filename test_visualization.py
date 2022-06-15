import visualize_3d,time
pitch=-13
sign_flag2=1
while True:
    if pitch > 13:
        sign_flag2=-1
    if pitch <-13:
        sign_flag2=1
    pitch=pitch+sign_flag2
    visualize_3d.update(pitch,0,0)
    time.sleep(0.1)



