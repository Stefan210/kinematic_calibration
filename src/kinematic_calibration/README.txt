===============================================================================
GENERAL USAGE
===============================================================================

1. Step: Adjust the configuration files.
-------------------------------------------------------------------------------

a) Open the config folder:
      roscd kinematic_calibration/config

b) Adjust the contents of the file "params_general" to your needs:

- parameters for selecting which parameters should be calibrated:
      calibrate_joint_offsets: [boolean]
      calibrate_camera_transform: [boolean]
      calibrate_camera_intrinsics: [boolean]
      calibrate_marker_transform: [boolean]

- file name suffixes which will contain the calibrated values after the optimization:
      joint_offsets_filename_suffix: [string]
      camera_transform_filename_suffix: [string]
      urdf_filename_suffix: robot_model_[string]
      marker_transforms_filename_suffix: [string]
      camera_intrnsics_filename: [string]

c)
For each kinematic chain, create or modify the following three files:
- params_CHAINNAME_general.yaml (contains general information about the chain):
      chain_name: [string] (identifier of the chain; eg. larm)
      chain_root: [string] (first frame after the camera; eg. HeadPitch_link)
      chain_tip: [string] (last frame of the chain; eg. l_wrist)
      marker_type: [string] (decides the marker detection strategy; eg. checkerboard, circle)
      marker_frame: [string] (eg. LMarker_frame, l_tip)
      CHAINNAME_marker_radius: [float] (optional; used if marker_type = circle; eg. 0.11)
      
      Only the joint offsets of the non-fixed joints contained in the chain 
      from chain_root to chain_tip will be calibrated.
      
      The transformation from chain_tip to marker_frame will be estimated and 
      initially loaded from the model. If marker_frame does not exist (which is ok),
      the identity transformation will be used as initial guess.
      
- params_CHAINNAME_capturing.yaml (contains information needed for data capturing):
      start_pose_num: [integer] (number of first pose)
      end_pose_num: [integer] (number of last psoe)
      headYaw_min: [float] (for searching the marker)
      headYaw_max: [float] (for searching the marker)
      headYaw_step: [float] (for searching the marker)
      headPitch_min: [float] (for searching the marker)
      haedPitch_max: [float] (for searching the marker)
      headPitch_step: [float] (for searching the marker)
      marker_color_r: [integer] (optional; currently only used for marker_type = circle and no radius given)
      marker_color_g: [integer] (optional; currently only used for marker_type = circle and no radius given)
      marker_color_b: [integer] (optional; currently only used for marker_type = circle and no radius given)

- poses_CHAINNAME.yaml (contains the poses used for data capturing for that chain):
      E.g.:

      larm080:
            joint_names: [LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand]
            positions: [0.1058039665222168, -0.14730596542358398, -1.2533202171325684, -0.7362780570983887,
                  1.2087500095367432, 0.3700000047683716]
            time_from_start: 1.0
      ...
      
      Note that the pose names have to follow the pattern CHAINNAME concatenated 
      with the number %3i, e.g. larm001, larm002, ..., larm080
   
   
2. Step: Start necessary nodes.
-------------------------------------------------------------------------------
a) Start roscore.

b) On the robot:
- nao_driver: (publishing /joint_states)
      roslaunch nao_driver nao_driver.launch

c) On the local machine:
- nao_camera: (publishing /nao_camera/camera_info and /nao_camera/image_raw)
      NAO_IP=ra.local roslaunch nao_driver nao_camera.launch (adjust NAO_IP!)
- robot model: (publishing /tf and loads the robot model)
      roslaunch nao_description nao_v4_publisher.launch OR
      roslaunch nao_description2 nao_publisher.launch robot:=ra OR
      roslaunch nao_description2 nao_publisher.launch robot:=ra_xylopy OR
      (whatever loads the uncalibrated robot model to the parameter server)
      
- optimiztaion: (waits for /kinematic_calibration/measurement_data and publishes /kinematic_calibration/calibration_result)
      roslaunch kinematic_calibration optimization.launch
      
- update: (waits for /kinematic_calibration/calibration_result)
      rosrun kinematic_calibration updateNode
            
      
3. Step: Capture Data.
-------------------------------------------------------------------------------
Repeat the following steps for each chain:

a) Make sure that none of the markers of other chains are visible to the camera.

b) Optionally record the measurements into a bag file:
      mkdir /tmp/CHAINNAME
      cd /tmp/CHAINNANE
      rosbag record /kinematic_calibration/measurement_data      

c) Start the data capturing node:
      roslaunch kinematic_calibration data_capturing.launch chain_name:=CHAINNAME

d) If b) was followed, then you should also save the parameters into a file:
      rosparam dump /tmp/CHAINNAME/params.txt

The data capturing process can be interrupted by calling:
      rosservice call /kinematic_calibration/data_capture/pause "reason: 'REASON'"
To proceed with data caputring, call:
      rosservice call /kinematic_calibration/data_capture/reseume "reason: 'REASON'"
      
      
      
4. Step: Optimize.
-------------------------------------------------------------------------------

a) Exclude bad measurements:
In your /tmp folder, you find the images which where used for the measurements.
Copy all images which you don't want to use for the optimization into a seperate folder, eg. /tmp/bad
Execute
      rosrun kinematic_calibration IgnoreMeasurementsConfigGeneration.py /tmp/bad/ && rosparam load ignore_measurements.yaml

b) Start the optimization process:
      roslaunch kinematic_calibration optimization.launch
      


5. Step: Update the model and camera intrinsics.
-------------------------------------------------------------------------------

a) Move the ROBOTNAME_calibration_*.xacro files into nao_description/urdf/ or nao_description2/urdf/

b) Move the nao_bottom_640x480.yaml file into nao_driver/config/
