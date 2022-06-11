ANGLE_OFFSET=135 #offset add to inversek results refer to inK6RSSV1.h motor range limits

def make_command(inverse_kinematics_results,ret):
    command=""
    if  (inverse_kinematics_results[0] == -360) or \
        (inverse_kinematics_results[1] == -360) or \
        (inverse_kinematics_results[2] == -360) or \
        (inverse_kinematics_results[3] == -360) or \
        (inverse_kinematics_results[4] == -360) or \
        (inverse_kinematics_results[5] == -360) or \
        (ret == 0 ) :
        return None

    for i in range(6):
        if i != 5:
            command+=str(inverse_kinematics_results[i]+ANGLE_OFFSET)+","
        else:
            command+=str(inverse_kinematics_results[i]+ANGLE_OFFSET)+"\n"
    return command