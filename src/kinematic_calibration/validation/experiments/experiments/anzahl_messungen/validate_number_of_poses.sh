#! /bin/bash

MEASUREMENTS_PATH_PREFIX="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/"
CAMERA_BAG="/media/Daten/Studium/Uni\ Freiburg\ Master\ Informatik/WS201314/Masterthesis/bagfiles_measurements/2014-05-16\ Validierung\ Messungen/concatenated/camera/2014-05-08-15-00-27.bag"
LARM_BAG=${MEASUREMENTS_PATH_PREFIX}"larm/larm750.bag"
RARM_BAG=${MEASUREMENTS_PATH_PREFIX}"rarm/rarm750.bag"
LLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"lleg/lleg750.bag"
RLEG_BAG=${MEASUREMENTS_PATH_PREFIX}"rleg/rleg750.bag"



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

OPT1=(B A A)
OPT2=(C C B)
VAL=(A B C)

for set in 2
#for set in 0 1 2
do
      MAINFOLDER=${OPT1[${set}]}${OPT2[${set}]}_${VAL[${set}]}
      if [[ ! -d "${MAINFOLDER}" ]]
      then
	    mkdir ${MAINFOLDER}
      fi
      cd ./${MAINFOLDER} &&      
      
      # set validation ids
      echo "validation_ids: [" > poses.yaml &&
      for CHAIN in $CHAINS; do
	    cat ../../poses/ids_${CHAIN}${VAL[${set}]}.txt >> poses.yaml
      done
      echo "]" >> poses.yaml &&
      rosparam load poses.yaml 

      #for i in `seq 10 100`
      for i in 29 30 69 88 94 95 96 97 98 99 100
      do
	    # reset params
	    rosparam load `rospack find kinematic_calibration`/config/params_general.yaml &&
	    rosparam delete /optimization_ids &&
	    rosparam set /optimization_ids ""
	    
	    # set some params that stay constant
	    rosparam set /randomize_measurements_per_chain 0 &&
	    rosparam set /use_robust_kernel false &&
	    rosparam set /add_noise_to_measurements 0 &&
	    rosparam set marker_optimization_type single_point &&
	    rosparam set calibrate_joint_6D false &&
	    rosparam set calibrate_joint_offsets true &&
	    
	    # set poses
	    echo "optimization_ids: [" > poses.yaml &&
	    for CHAIN in $CHAINS; do
		  cat ../../poses/optimal/${OPT1[${set}]}${OPT2[${set}]}/${CHAIN}/${CHAIN}_${i} >> poses.yaml
	    done
	    echo "]" >> poses.yaml &&
	    rosparam load poses.yaml &&
	    
	    # set parameters
	    rosparam set /max_iterations 50 &&
	    rosparam set /folder_name ${FOLDER_PREFIX}_${i} &&
	    rosparam set /validation_data_strategy split &&

	    rosservice call /kinematic_calibration/start_optimization &&  
	    
	    mv ../${FOLDER_PREFIX}_${i} ./
      done
      cd ../
done

killall validationNode
