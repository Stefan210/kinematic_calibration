-------------------------------------------------------------------------------
| I) Generierung der Posenmengen
-------------------------------------------------------------------------------

1) In "launch/pose_generation.launch": "poseSelectionNode" zu "validationPoseSelectionNode" ändern

2) Parameter:
  a) bereits in launch/pose_generation.launch enthalten (inkl. Doku.):
    pose_source: "measurement" | 
    num_of_poses: 50
  b) zusätzlich:
    - Selektionsalgorithmus:
      selection_strategy: optimal | random | random_exchange | improve_given
	--> optimal: selektiert Posen gemäß Optimierungsalgorithmus
	--> random: selektiert zufällige Posen
	--> random_exchange: selektiert zufällige Posen und optimiert die Mengen mittels "Exachange"
	--> improve_given: nimmt initiale Posen, gemäß der IDs, die mit dem Parameter 
	      "initial_pose_ids/<partition_number>/<list_of_ids>"
	    angegeben werden
    - Anzahl der Splits/Partitionen:
      num_of_splits: 5
  
3) Optimierung starten:
  a) roslaunch kinematic_calibration pose_generation.launch chain_name:=<chain_name>
  b) bag-files abspielen (camera_info, /kinematic_calibraion/measurement_data)
  c) rosservice call /kinematic_calibration/stop_collecting

  4) Die generierten Posen liegen in ROSHOME/<chain_name>/...