#! /bin/bash

CHAINS='larm rarm lleg rleg'
FOLDER_PREFIX="all"

#CHAINS='larm'
#FOLDER_PREFIX="larm"

#CHAINS='lleg'
#FOLDER_PREFIX="lleg"

MEASUREMENTS_PATH_PREFIX="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/"
CAMERA_BAG="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/camera/2014-05-08-15-00-27.bag"
LARM_BAG=${MEASUREMENTS_PATH_PREFIX}"larm/larm750.bag"
RARM_BAG=${MEASUREMENTS_PATH_PREFIX}"rarm/rarm750.bag"
LLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"lleg/lleg750.bag"
RLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"rleg/rleg750.bag"

# set validation ids
rosparam load posen/validation_ids.yaml

# start validation node
rosrun kinematic_calibration validationNode &
valnodepid=$!
sleep 2

 # play bags
eval rosbag play $CAMERA_BAG -i -d 1.5 &&
for CHAIN in $CHAINS; do
      BAGFILE=${MEASUREMENTS_PATH_PREFIX}$CHAIN/${CHAIN}750".bag"
      eval rosbag play $BAGFILE -i -d 0.5 
done

for i in `seq 0 20`
#for i in 0
do
      for rk in "true" "false"
	    do
	    # reset params
	    rosparam load `rospack find kinematic_calibration`/config/params_general.yaml &&
	    rosparam delete /optimization_ids &&
	    rosparam set /optimization_ids ""
	    
	    # set poses
	    echo "optimization_ids: [" > poses.yaml &&
	    for CHAIN in $CHAINS; do
		  cat posen/$CHAIN/${CHAIN}_100 >> poses.yaml
	    done
	    echo "]" >> poses.yaml &&
	    rosparam load poses.yaml &&

	    # set parameters
	    FOLDERNAME=${FOLDER_PREFIX}_randomize_${i}_measurements_rk_${rk}
	    rosparam set /max_iterations 100 &&
	    rosparam set /folder_name $FOLDERNAME &&
	    rosparam set /validation_data_strategy split &&
	    rosparam set /randomize_measurements_per_chain $i &&
	    rosparam set /use_robust_kernel $rk &&

	    rosservice call /kinematic_calibration/start_optimization

	    while [[ ! -f "${FOLDERNAME}/optimization_intermediate_results.csv" ]]
	    do
		  sleep 1
	    done
      done
done

killall validationNode
