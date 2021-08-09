# the operator name (e.g., 'pick_up_object')
string operator

# the list of arguments (e.g., ['coffee_cup1', 'left_arm'])
string[] args

# preconditions as list of strings.
# e.g., ['(Holding left_arm1 cup1)', '(not (On cup1 pae1))']
string[] preconditions

# Add Effects
string[] adds

# Delete Effects
string[] deletes
