#! /bin/bash

NAMESPACE="random"
POSES="random"

MEASUREMENTS_PATH_PREFIX="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/"
CAMERA_BAG="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/camera/2014-05-08-15-00-27.bag"
LARM_BAG=${MEASUREMENTS_PATH_PREFIX}"larm/larm750.bag"
RARM_BAG=${MEASUREMENTS_PATH_PREFIX}"rarm/rarm750.bag"
LLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"lleg/lleg750.bag"
RLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"rleg/rleg750.bag"

# start validation node
export ROS_NAMESPACE=/$NAMESPACE
rosrun kinematic_calibration validationNode &
valnodepid=$!
sleep 2

# play bags
eval rosbag play $CAMERA_BAG -i -d 1.5 &&
for CHAIN in $CHAINS; do
      BAGFILE=${MEASUREMENTS_PATH_PREFIX}$CHAIN/${CHAIN}750".bag"
      eval rosbag play $BAGFILE -i -d 0.5 
done

for valset in 4
#for valset in `seq 1 5`
do
      MAINFOLDER="all_except_${valset}"
      if [[ ! -d "${MAINFOLDER}" ]]
      then
	    mkdir ${MAINFOLDER}
      fi
      cd ./${MAINFOLDER} &&      
      
      # set validation ids
      echo "/${NAMESPACE}/validation_ids: [" > poses.yaml &&
      for CHAIN in $CHAINS; do
		  cat ../../../poses/${POSES}/${CHAIN}/${CHAIN}_partition_${valset} >> poses.yaml
      done
      echo "]" >> poses.yaml &&
      rosparam load poses.yaml 

      #for i in `seq 5 50`
      for i in 12 29 38  
      do
	    # reset params
	    rosparam load `rospack find kinematic_calibration`/config/params_general.yaml &&
	    rosparam delete /${NAMESPACE}/optimization_ids &&
	    rosparam set /${NAMESPACE}/optimization_ids ""
	    
	    # set some params that stay constant
	    rosparam set /${NAMESPACE}/randomize_measurements_per_chain 0 &&
	    rosparam set /${NAMESPACE}/use_robust_kernel false &&
	    rosparam set /${NAMESPACE}/add_noise_to_measurements 0 &&
	    rosparam set /${NAMESPACE}/marker_optimization_type single_point &&
	    rosparam set /${NAMESPACE}/calibrate_joint_6D false &&
	    rosparam set /${NAMESPACE}/calibrate_joint_offsets true &&
	    rosparam set /${NAMESPACE}/calibrate_camera_transform true &&
	    rosparam set /${NAMESPACE}/calibrate_camera_intrinsics true &&
	    rosparam set /${NAMESPACE}/calibrate_marker_transform true &&
	    
	    # set poses
	    echo "/${NAMESPACE}/optimization_ids: [" > poses.yaml &&
	    for CHAIN in $CHAINS; do
		cat ../../../poses/${POSES}/${CHAIN}/all_except_${valset}/${CHAIN}_${i} >> poses.yaml
	    done
	    echo "]" >> poses.yaml &&
	    rosparam load poses.yaml &&
	    
	    # set parameters
	    rosparam set /${NAMESPACE}/max_iterations 50 &&
	    rosparam set /${NAMESPACE}/folder_name ${FOLDER_PREFIX}_${i} &&
	    rosparam set /${NAMESPACE}/validation_data_strategy split &&

	    rosservice call /${NAMESPACE}/kinematic_calibration/start_optimization &&  
	    
	    mv ../${FOLDER_PREFIX}_${i} ./
      done
      cd ../
done

kill $valnodepid
