#! /bin/bash

POSES="random"
CHAINS="larm"

for valset in `seq 1 5`
do
      for i in `seq 5 50`
      do    
	    # set poses
	    echo "initial_pose_ids/${valset}/${i}: [" > poses.yaml &&
	    for CHAIN in $CHAINS; do
		#cat ../../../poses/${POSES}/${CHAIN}/all_except_${valset}/${CHAIN}_${i} >> poses.yaml
		cat all_except_${valset}/${CHAIN}_${i} >> poses.yaml
	    done
	    echo "]" >> poses.yaml &&
	    rosparam load poses.yaml
      done
done
