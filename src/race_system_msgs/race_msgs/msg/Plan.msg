float32 cost
Operator[] operators
# decomposition in yaml format
# methods have 'method:' (string for the name of the method), 
#              'args:' (list of strings)
#              'subtasks:' (nested list of methods or operators)
# operators have 'operator:' (string for the name of the operator)
#                'args:' (list of strings)
#e.g.,
#- method: 'assume_manipulation_pose'
#  args: ['manipulationareasouthtable1', 'rightarm1']
#  subtasks:
#    - {operator: '!move_torso', args: ['torsoupposture']}
#    - method: 'assume_manipulation_pose'
#      args: ['manipulationareasouthtable1', 'rightarm1']
#      subtasks:
#   ...
string decomposition
