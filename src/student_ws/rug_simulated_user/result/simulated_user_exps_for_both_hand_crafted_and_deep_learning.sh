
##roscore
roscore 2>/dev/null > /dev/null 2>&1 &
echo "roscore is running"
sleep 2

## For this example, we run the multi_view_RGBD_object_representation with mobileNetV2 base_network 
base_network="mobileNetV2"
rosrun rug_deep_feature_extraction multi_view_RGBD_object_representation.py $base_network > /dev/null 2>&1 &
echo "multi_view_RGBD_object_representation service is running with $base_network base_network"
sleep 10

mkdir COR;
mkdir COR/hand_crafted_exps;
mkdir COR/deep_learning_exps;

let exp_num=1

for i in `seq 1 10`; do 

	### NOTE: Make sure that you have set the proper value for each param in the launch file
	echo roslaunch rug_simulated_user simulated_user_hand_crafted_descriptor.launch random_sequence_generator:=true name_of_approach:="hand_crafted_exps"
	
	roslaunch rug_simulated_user simulated_user_hand_crafted_descriptor.launch random_sequence_generator:=true name_of_approach:="hand_crafted_exps"

	mv "experiment_1" "exp_$exp_num" 
	mv "exp_$exp_num" COR/hand_crafted_exps 
	sleep 2


	### NOTE 1: Make sure that you have set the proper value for each param in the launch file
	### NOTE 2: for deep transfer learning exps, we set random sequence generator param to false to use the same sequence as handcrafted experiment used

	echo roslaunch rug_simulated_user simulated_user_RGBD_deep_learning_descriptor.launch random_sequence_generator:=false base_network:=$base_network name_of_approach:="deep_transfer_learning"

	roslaunch rug_simulated_user simulated_user_RGBD_deep_learning_descriptor.launch random_sequence_generator:=false base_network:=$base_network name_of_approach:="deep_transfer_learning"

	mv "experiment_1" "exp_$exp_num" 
	mv "exp_$exp_num" COR/deep_learning_exps 
	sleep 2

	let exp_num=exp_num+1 

done 

rm sum_all_results_of*
