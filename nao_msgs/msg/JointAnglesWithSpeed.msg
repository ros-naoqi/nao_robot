Header header

# A list of joint names, corresponding to their names in the Nao docs.
# This must be either the same lenght of joint_angles or 1 if it's a
# keyword such as 'Body' (for all angles)
string[] joint_names
float32[] joint_angles

#fraction of max joint velocity [0:1]
float32 speed

# Absolute angle(=0, default) or relative change
uint8 relative
