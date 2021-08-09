# the name of the fluent
string name

# the type of the fluent (an OWL class name)
string type

# properties of the fluent
Property[] properties

# start time and finish_time time of an fluent defined as a flexible temporal
# interval, i.e., both start time and finish_time time are flexible timepoints
# - start_time.lower <= finish_time.lower
# - start_time.upper <= finish_time.upper
# - start_time and finish_time may overlap (e.g., start_time = [30, 40] and finish_time = [34, 44])
# - if a time point is known with certainty, lower = upper
# - if a time point is unknown, lower = upper = Time(0.0) = 1970-01-01 00:00:00
FlexibleTimepoint start_time
FlexibleTimepoint finish_time
