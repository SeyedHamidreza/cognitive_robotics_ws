
## roscore
roscore 2>/dev/null > /dev/null 2>&1 &
echo "roscore is running"
sleep 5

rm -r $HOME/pdbs
mkdir $HOME/pdbs

## list of available networks : "vgg16_fc1", "vgg16_fc2", "vgg19_fc1", "vgg19_fc2", "xception", "resnet50", "mobileNet",  "mobileNetV2", 
# 				"denseNet121", "denseNet169", "densenet201", "nasnetLarge", "/nasnetMobile", "inception",  "inceptionResnet" 
declare -a base_networks=( "mobileNetV2")

## list of available pooling functions : "MAX" "AVG" "APP"
declare -a pooling=( "MAX" "AVG" "APP")

## list of available distance functions : "cosine" "gower" "chiSquared" "KLDivergance" "symmetricKL" "motyka" "euclidean"  "intersection"  
#                                         "dice" "bhattacharyya" "sorensen" "canberra" "pearson" "neyman"
declare -a distances=( "cosine" "euclidean" "chiSquared" "motyka" "symmetricKL" "motyka")

for n in "${base_networks[@]}"; do	

	## previous service node will be shutdown since a new node will be registered with same name
	rosrun rug_deep_feature_extraction multi_view_RGBD_object_representation.py $n > /dev/null 2>&1 &
	echo "multi_view_RGBD_object_representation service is running"
	sleep 10

	## make a folder to organize all the results there
	mkdir $n"_results"
	
	## make a folder for saving the perceptual memory
	mkdir $HOME/pdbs/$n

	let exp=1
	let k=1
	let b=1

	for b in `seq 1 4`; do 
		let bins=b*50;	
		for p in "${pooling[@]}"; do	
			mkdir $HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/	
			let k=1
						
			echo roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins pooling_function:=$p distance_function:="cosine"  K_for_KNN:=$k pdb_loaded:=false running_a_bunch_of_experiments:=falsename_of_approach:=$n"_TEST"

			roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins pooling_function:=$p distance_function:="cosine"  K_for_KNN:=$k pdb_loaded:=false running_a_bunch_of_experiments:=falsename_of_approach:=$n"_TEST"

			mv "experiment_1" "exp_$exp"
			mv "exp_$exp" $n"_results"					
			let exp=exp+1           
			
			### save the preceptual memory
			mv /tmp/pdb/ $HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/pdb 	
		
			sleep 1

			for k in 3 5 7 9; do 			 			

				echo roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins distance_function:="cosine" pooling_function:=$p K_for_KNN:=$k running_a_bunch_of_experiments:=true pdb_loaded:=true pdb_source:=$HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/pdb name_of_approach:=$n"_TEST"

				roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins distance_function:="cosine" pooling_function:=$p K_for_KNN:=$k running_a_bunch_of_experiments:=true pdb_loaded:=true pdb_source:=$HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/pdb name_of_approach:=$n"_TEST"

				mv "experiment_1" "exp_$exp"
				mv "exp_$exp" $n"_results"					
				let exp=exp+1           

				sleep 1

			done 


			for d in "${distances[@]}"; do
				echo "$d"
				for k in 1 3 5 7 9; do 			 			

					echo roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins distance_function:=$d pooling_function:=$p K_for_KNN:=$k running_a_bunch_of_experiments:=true pdb_loaded:=true pdb_source:=$HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/pdb name_of_approach:=$n"_TEST"
					
					 roslaunch rug_kfold_cross_validation kfold_cross_validation_RGBD_deep_learning_descriptor.launch base_network:=$n orthographic_image_resolution:=$bins distance_function:=$d pooling_function:=$p K_for_KNN:=$k running_a_bunch_of_experiments:=true pdb_loaded:=true pdb_source:=$HOME/pdbs/$n/pdb"_"$bins"_bins_"$p"_pooling"/pdb name_of_approach:=$n"_TEST"

					mv "experiment_1" "exp_$exp"
					mv "exp_$exp" $n"_results"					
					let exp=exp+1           

					sleep 1

				done 

			done
		done
				
	done
done

