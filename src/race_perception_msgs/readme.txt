For adding a new msgs to system (to be compatible with perception_db):
add following items to the race_perception_msgs/include/race_perception_msgs/perception_msgs.h :
	1) add #include of new msg to the RACE perception msg header section 
		example: #include <race_perception_msgs/TOVI.h> 
	
	2) add a key for the new msg to the namespace section
		example: const std::string TOVI = "TOVI_";

It should be noted that the key shold not be redundant and needs to end with underscore '_' .
		
