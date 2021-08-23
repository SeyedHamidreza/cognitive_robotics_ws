
## roscore
roscore 2>/dev/null > /dev/null 2>&1 &
echo "roscore is running"
sleep 5

rm -r $HOME/pdbs
mkdir $HOME/pdbs

## list of available distance functions : "cosine" "gower" "chiSquared" "KLDivergance" "symmetricKL" "motyka" "euclidean"  "intersection"  "dice" "bhattacharyya" "sorensen" "canberra" "pearson" "neyman"
declare -a distances=( "euclidean" "chiSquared" "symmetricKL" "cosine" "motyka")

############################### GOOD ####################################
mkdir "hand_crafted_results"

# make a directory to save the perceptual memory
mkdir $HOME/pdbs/GOOD

let exp=1
let n=1

for n in 1 6; do 
	let bins=n*25;
	mkdir $HOME/pdbs/GOOD/pdb_$bins"_bins"/
	for d in "${distances[@]}"; do
		let k=1		
		echo "$d"
		echo roslaunch rug_kfold_cross_validation kfold_cross_validation_hand_crafted_descriptor.launch distance_function:=$i object_discriptor:=GOOD number_of_bins:=$bins  distance_function:=$d running_a_bunch_of_experiments:=false K_for_KNN:=$k name_of_approach:=TEST_COR

		roslaunch rug_kfold_cross_validation kfold_cross_validation_hand_crafted_descriptor.launch distance_function:=$i object_discriptor:=GOOD number_of_bins:=$bins  distance_function:=$d running_a_bunch_of_experiments:=false K_for_KNN:=$k name_of_approach:=TEST_COR

		mv "experiment_1" "exp_$exp"
		mv "exp_$exp" "hand_crafted_results"					
		let exp=exp+1           
		
		### save the preceptual memory
		mv /tmp/pdb/ $HOME/pdbs/GOOD/pdb_$bins"_bins"/pdb 	
	
		for k in 3 5 7 9; do 	
			echo roslaunch rug_kfold_cross_validation kfold_cross_validation_hand_crafted_descriptor.launch distance_function:=$i object_discriptor:=GOOD number_of_bins:=$bins  distance_function:=$d running_a_bunch_of_experiments:=true K_for_KNN:=$k pdb_loaded:=true pdb_source:=$HOME/pdbs/GOOD/pdb_$bins"_bins"/pdb name_of_approach:=TEST_COR

			roslaunch rug_kfold_cross_validation kfold_cross_validation_hand_crafted_descriptor.launch distance_function:=$i object_discriptor:=GOOD number_of_bins:=$bins  distance_function:=$d running_a_bunch_of_experiments:=true K_for_KNN:=$k pdb_loaded:=true pdb_source:=$HOME/pdbs/GOOD/pdb_$bins"_bins"/pdb name_of_approach:=TEST_COR

			mv "experiment_1" "exp_$exp"
		   	mv "exp_$exp" "hand_crafted_results"					
		   	let exp=exp+1           
		  
		
		done
	done
done

############################### ESF  ####################################

############################### VFH  ####################################

############################### GRSD ####################################

