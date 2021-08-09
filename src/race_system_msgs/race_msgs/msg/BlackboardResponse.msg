string error_message
int32 result_code
# result equals one of the following result codes
# General success
int32 SUCCESS = 0
# General Failure (reason unknown, multiple reasons, 
# exceptions etc., contact blackboard staff 
# (Pascal, 7rost@informatik.uni-hamburg.de))
int32 FAILURE = 1
# Fluent (instance) not found (retrieve_fluents)
int32 NOFLUENTFOUND = 2
# No connection to the sesame server.
int32 NOCONNECTION = 3
# SparQL-Query malformed.
int32 MALFORMEDQUERY = 4
# Type of the Fluent and/or one of its properties does not
# exist in the ontology.
int32 INVALIDTYPE = 5
# Problem with the monotony paradigm of times 
# (only specify times)
int32 TIMESMONOTONYERROR = 6
# Problem with restrictions of propertys and data etc.
# See the ontology for restrictions of classes
int32 RESTRICTIONSERROR = 7