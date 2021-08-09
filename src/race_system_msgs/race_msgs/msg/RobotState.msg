# Represents the PR2 state
Header header
string[] name # left_arm, right_arm, torso_pose, head
string[] pose 
# arm: 'ArmCarryPosture', 'ArmMovingPosture', 'ArmPointingPosture', 'ArmToSidePosture', 'ArmTuckedPosture', 'ArmUnnamedPosture', 'ArmUntuckedPosture'
# torso: 'TorsoDownPosture', 'TorsoUpPosture', 'TorsoMiddlePosture', 'TorsoMovingPosture', 'TorsoUnnamedPosture';
#head: 'straight', 'down', 'left', 'right'
