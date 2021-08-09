# This message represents an OWL object property or datatype property.

# the role type (an OWL Datatype Property or Object Property, e.g., "hasArm")
string role_type

# The type of the role filler; this can have one of the following values:
# - the name of a valid OWL class --> evaluate object_filler to get value
# - "xsd:boolean" --> evaluate bool_filler
# - "xsd:float" --> evaluate float_filler
# - "xsd:int" --> evaluate ingeger_filler
# - "xsd:string" --> evaluate string_filler
# - "xsd:dateTime" --> evaluate time_filler
# - "xsd:duration" --> evaluate duration_filler
#
# All other "xsd:..." types should be considered an error.
string filler_type

# The role filler; exactly one is non-empty. Must match filler_type.
string object_filler      # name of an existing OWL individual, e.g., "left_arm"
bool bool_filler
float32 float_filler
int32 integer_filler
string string_filler      # actual string, e.g., "Hello World!"
time time_filler
duration duration_filler
