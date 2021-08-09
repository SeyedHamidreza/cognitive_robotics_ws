cmake_minimum_required(VERSION 2.8.3)
project(race_msgs)

set(PKG_DEPS
	std_msgs
	actionlib_msgs
	sensor_msgs
	geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
		  roscpp
		  message_generation
		  ${PKG_DEPS}
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg 
	FILES
	AffordanceException.msg
	FluentNameArray.msg
	#PlanningActionFeedback.msg
	#PlanningActionResult.msg
	SceneLayoutLabel.msg
	Atom.msg
	FluentNameTranslation.msg
	SpatialCalculus.msg
	BlackboardResponse.msg
	#ExecuteTaskGoal.msg
	#ExecuteTaskFeedback.msg
	#ExecuteTaskResult.msg
	LabeledBoundingBoxMarker.msg
	SpatialEntity.msg
	BoundingBox.msg
	ObjectDetectionArray.msg
	SpatialRelation.msg
	ConfidenceList.msg
	ExecutionPredicateInfo.msg
	ObjectDetection.msg
	SpatialRelationSet.msg
	DispatchedActionInfo.msg
	ExperienceHeader.msg
	ObjectHypothesis.msg
	TaskAtom.msg
	DurationList.msg
	ExperienceTail.msg
	ObjectRecognition.msg
	#PlanningAction.msg
	#PlanningActionGoal.msg
	#PlanningGoal.msg
	#PlanningResult.msg
	#PlanningFeedback.msg
	Triple.msg
	FlexibleTimepoint.msg
	Operator.msg
	Property.msg
	WorldState.msg
	FluentArray.msg
	ParamList.msg
	PropertyTypeDescription.msg
	#ExecuteTaskAction.msg
	#ExecuteTaskActionResult.msg
	#ExecuteTaskActionFeedback.msg
	#ExecuteTaskActionGoal.msg
	Fluent.msg
	Plan.msg
	RobotState.msg
)


## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	AddFluent.srv
	GenerateSHOP2Plan.srv
	GetOntClasses.srv
	ObserveObjectsOn.srv
	AddFluents.srv
	GetActiveObjectsSpatialRelations.srv
	GetOntIndividuals.srv
	ResetRepository.srv
	AddTriple.srv
	GetAffordanceExceptions.srv
	GetPropertyList.srv
	RetrieveFluent.srv
	DeleteTriple.srv
	GetAnchoredObjects.srv
	GetRepositorySize.srv
	SendFluentName.srv
	EnableReplanning.srv
	GetCurrentEpisode.srv
	GetSpatialRelations.srv
	SendFluentNames.srv
	#ExecuteTask.srv
	GetEpisodes.srv
	GetSubClasses.srv
	SetEpisode.srv
	FinishExperienceRecording.srv
	GetFluentsByQuery.srv
	GetTimeLoggingInfo.srv
	StartExperienceRecording.srv
	GenerateChimpPlan.srv
	GetFluentsInTimeRange.srv
	IsConsistent.srv
	UpdateFluents.srv
	GenerateImagineState.srv
	GetNextCommand.srv
	IsHumanWorking.srv
	UpdateTriple.srv
	GenerateProblemFile.srv
	GetNumberOfFluents.srv
	IsSceneLayoutLabel.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES 
	${PKG_DEPS}
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES ur_msgs
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
	#DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############


#install(
#DIRECTORY  include/${PROJECT_NAME}/
#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)



