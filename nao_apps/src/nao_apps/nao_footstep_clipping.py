#!/usr/bin/env python

import almath


# Struct with clipping params initialized with clipping values for the NAO robot
class Params(object):
    # Various parameters describing step characteristics.
    min_step_x = -0.04
    min_step_y = 0.088
    max_step_x = 0.08
    max_step_y = 0.16
    max_step_theta = 0.5 # 0.5xxx
    min_step_theta = -max_step_theta

    # Bounding boxes of NAO's feet.
    foot_box_r_fl = almath.Pose2D(0.11, 0.038, 0.0)
    foot_box_r_fr = almath.Pose2D(0.11, -0.050, 0.0)
    foot_box_r_rr = almath.Pose2D(-0.047, -0.050, 0.0)
    foot_box_r_rl = almath.Pose2D(-0.047, 0.038, 0.0)
    foot_box_r = almath.vectorPose2D([foot_box_r_fl, foot_box_r_fr,
                                      foot_box_r_rr, foot_box_r_rl])

    foot_box_l_fl = almath.Pose2D(0.11, 0.050, 0.0)
    foot_box_l_fr = almath.Pose2D(0.11, -0.038, 0.0)
    foot_box_l_rr = almath.Pose2D(-0.047, -0.038, 0.0)
    foot_box_l_rl = almath.Pose2D(-0.047,  0.050, 0.0)
    foot_box_l = almath.vectorPose2D([foot_box_l_fl, foot_box_l_fr,
                                      foot_box_l_rr, foot_box_l_rl])


def clip_data(min, max, value):
    ''' Clip value between two extremes. '''
    clipped = value

    if (clipped < min):
        clipped = min
    if (clipped > max):
        clipped = max

    return clipped


def clip_footstep_on_gait_config(foot, is_left_support):
    ''' Clip the foot move so that it does not exceed the maximum
        size of steps.
        foot is an almath.Pose2D (x, y, theta position).
        is_left_support must be set to True if the move is on the right leg
        (the robot is supporting itself on the left leg).
    '''

    # Clip X.
    clipped_x = clip_data(Params.min_step_x, Params.max_step_x, foot.x)
    foot.x = clipped_x

    # Clip Y.
    if not is_left_support:
        clipped_y = clip_data(Params.min_step_y, Params.max_step_y, foot.y)
    else:
        clipped_y = clip_data(-Params.max_step_y, -Params.min_step_y, foot.y)
    foot.y = clipped_y

    # Clip Theta.
    clipped_theta = clip_data(Params.min_step_theta, Params.max_step_theta, foot.theta)
    foot.theta = clipped_theta


def clip_footstep_with_ellipse(foot):
    ''' Clip the foot move inside an ellipse defined by the foot's dimansions.
        foot is an almath.Pose2D (x, y, theta position).
    '''

    # Apply an offset to have Y component of foot move centered on 0.
    if (foot.y < -Params.min_step_y):
        foot.y = foot.y + Params.min_step_y
    elif (foot.y > Params.min_step_y):
        foot.y = foot.y - Params.min_step_y
    else:
      return

    # Clip the foot move to an ellipse using ALMath method.
    if foot.x >= 0:
        almath.clipFootWithEllipse(Params.max_step_x, Params.max_step_y - Params.min_step_y, foot)
    else:
        almath.clipFootWithEllipse(Params.min_step_x, Params.max_step_y - Params.min_step_y, foot)

    # Correct the previous offset on Y component.
    if foot.y >=0:
        foot.y  = foot.y + Params.min_step_y
    else:
        foot.y = foot.y - Params.min_step_y


def clip_footstep_to_avoid_collision(foot, is_left_support):
    ''' Clip the foot move to avoid collision with the other foot.
        foot is an almath.Pose2D (x, y, theta position).
        is_left_support must be set to True if the move is on the right leg
        (the robot is supporting itself on the left leg).
    '''

    # Use ALMath method.
    almath.avoidFootCollision(Params.foot_box_l, Params.foot_box_r, is_left_support, foot)


def clip_footstep(foot, is_left_support):
    ''' Clipping functions to avoid any warnings and undesired effects
        when sending the footsteps to ALMotion.
        foot is an almath.Pose2D (x, y, theta position)
        is_left_support must be set to True if the move is on the right leg
        (the robot is supporting itself on the left leg).
    '''

    # First clip the foot move with gait config.
    clip_footstep_on_gait_config(foot, is_left_support)
    # Then clip it on an ellipse.
    clip_footstep_with_ellipse(foot)
    # Then make sure you do not have any collision.
    clip_footstep_to_avoid_collision(foot, is_left_support)


def clip_footstep_tuple(foot, is_left_support):
    """
    Wrapper of clip_footstep to handle 'foot' as geometry_msgs.Pose2D.
    'foot' is a tuple (x, y, theta). A tuple (x_clip, y_clip, theta_clip) is
    returned.
    """

    foot = almath.Pose2D(*foot)
    clip_footstep(foot, is_left_support)
    return foot.x, foot.y, foot.theta
