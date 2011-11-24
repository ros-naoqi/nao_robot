import rospy


def scanPoseBending(motionProxy,head_pitch,head_yaw, t):
    #print "This is scanPoseBending" 
    names = list()
    times = list()
    keys = list()

    #t=2.0000

    names.append("HeadYaw")
    times.append([ t])
    keys.append([ [ head_yaw, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("HeadPitch")
    times.append([ t])
    keys.append([ [ head_pitch, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LShoulderPitch")
    times.append([ t])
    keys.append([ [ 2.00489, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LShoulderRoll")
    times.append([ t])
    keys.append([ [ 0.35585, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LElbowYaw")
    times.append([ t])
    keys.append([ [ -1.42666, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LElbowRoll")
    times.append([ t])
    keys.append([ [ -0.24540, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LWristYaw")
    times.append([ t])
    keys.append([ [ 0.00609, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("LHand")
    #times.append([ t])
    #keys.append([ [ 0.00840, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RShoulderPitch")
    times.append([ t])
    keys.append([ [ 2.02645, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RShoulderRoll")
    times.append([ t])
    keys.append([ [ -0.34673, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RElbowYaw")
    times.append([ t])
    keys.append([ [ 1.42351, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RElbowRoll")
    times.append([ t])
    keys.append([ [ 0.24702, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RWristYaw")
    times.append([ t])
    keys.append([ [ -0.00464, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("RHand")
    #times.append([ t])
    #keys.append([ [ 0.01469, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("LHipYawPitch")
    #times.append([ t])
    #keys.append([ [ 0.0, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("LHipRoll")
    #times.append([ t])
    #keys.append([ [ -0.01683, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #hp = -1.32533
    #kp = 1.13205
    
    hp = -1.32533*0.85
    kp = 1.13205*0.85
    names.append("LHipPitch")
    times.append([ t])
    keys.append([ [ hp, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("LKneePitch")
    times.append([ t])
    keys.append([ [ kp, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("LAnklePitch")
    #times.append([ t])
    #keys.append([ [ -0.30531, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("LAnkleRoll")
    #times.append([ t])
    #keys.append([ [ 0.01078, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("RHipRoll")
    #times.append([ t])
    #keys.append([ [ 0.00004, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RHipPitch")
    times.append([ t])
    keys.append([ [ hp, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    names.append("RKneePitch")
    times.append([ t])
    keys.append([ [ kp, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("RAnklePitch")
    #times.append([ t])
    #keys.append([ [ -0.25460, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])

    #names.append("RAnkleRoll")
    #times.append([ t])
    #keys.append([ [ -0.01837, [ 2, -1.00000, 0.00000], [ 2, 0.00000, 0.00000]]])


    check = True
    if check:
        numJoints = len(motionProxy.getJointNames('Body'))
        if (numJoints == 26):
            names2 = names
            times2 = times
            keys2 = keys
        elif (numJoints == 22):  # no hands (e.g. simulator)
            names2 = names[0:6] + names[8:24]
            times2 = times[0:6] + times[8:24]
            keys2  = keys[0:6] + keys[8:24]
        else:
            rospy.logerr("Unkown number of joints: %d", numJoints)
            return 
    else:
        names2 = names
        times2 = times
        keys2 = keys
    try:
        #motion = ALProxy("ALMotion")
        moveId = motionProxy.post.angleInterpolationBezier(names2, times2, keys2);
    except BaseException, err:
        print "An error has been caught"
        print err
        pass

