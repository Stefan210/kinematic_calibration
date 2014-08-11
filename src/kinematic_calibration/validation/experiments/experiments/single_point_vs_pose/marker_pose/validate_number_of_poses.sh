#! /bin/bash

CHAINS='rarm'
FOLDER_PREFIX="rarm"

MEASUREMENTS_PATH_PREFIX="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/"
CAMERA_BAG="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/camera/2014-05-08-15-00-27.bag"
LARM_BAG=${MEASUREMENTS_PATH_PREFIX}"larm/larm750.bag"
RARM_BAG=${MEASUREMENTS_PATH_PREFIX}"rarm/rarm750.bag"
LLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"lleg/lleg750.bag"
RLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"rleg/rleg750.bag"

# set validation ids
rosparam load ../poses/validation_ids.yaml

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

#for i in `seq 10 100`
for i in `seq 10 10 250`
do
      # reset params
      rosparam load `rospack find kinematic_calibration`/config/params_general.yaml &&
      rosparam delete /optimization_ids &&
      rosparam set /optimization_ids ""
      
      # set some params that stay constant
      rosparam set /max_iterations 100 &&
      rosparam set /validation_data_strategy split &&
      rosparam set /randomize_measurements_per_chain 0 &&
      rosparam set /use_robust_kernel false &&
      rosparam set /add_noise_to_measurements 0 &&
      rosparam set marker_optimization_type full_pose &&
      
      # set poses
      echo "optimization_ids: [" > poses.yaml &&
      for CHAIN in $CHAINS; do
	    cat ../poses/$CHAIN/${CHAIN}_${i} >> poses.yaml
      done
      echo "]" >> poses.yaml &&
      rosparam load poses.yaml &&

      # set parameters
      rosparam set /folder_name ${FOLDER_PREFIX}_${i} &&

      rosservice call /kinematic_calibration/start_optimization

      while [[ ! -f "${FOLDER_PREFIX}_${i}/optimization_intermediate_results.csv" ]]
      do
	    sleep 1
      done
done

killall validationNode
