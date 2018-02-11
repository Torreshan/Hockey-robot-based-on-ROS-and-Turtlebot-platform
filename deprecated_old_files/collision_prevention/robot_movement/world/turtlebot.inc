define kinect ranger
(
	sensor
	(
		range [ 0.8 3.5 ]
		fov 58.0
		samples 640
	)
	# generic model properties
	color "black"
	size [ 0.035 0.18 0.025 ]
	block (
		points 4
		point[0] [ -0.0175 -0.09 ]
		point[1] [ -0.0175  0.09 ]
		point[2] [  0.0175  0.09 ]
		point[3] [  0.0175 -0.09 ]
		z [ 0 0.025 ]
	)
)

define xition camera
(
	camera
	(
		resolution [ 1280 1024 ]
		range [ 0.2 6 ]
		fov [ 58 45 ]
	)
	size [ 0.035 0.18 0.025 ]
)

define rp_lidar ranger
(
	sensor
	(
		range [ 0 6 ]
		fov 360
		samples 360
	)
	
	# generic model properties
	color "grey"
	size [ 0.1 0.07 0.06 ]
	block (
		points 6
		point[0] [  0.04   0.04  ]
		point[1] [ -0.045  0.02  ]
		point[2] [ -0.065  0.005 ]
		point[3] [ -0.065 -0.005 ]
		point[4] [ -0.045 -0.02  ]
		point[5] [  0.04  -0.04  ]
		z [ 0 0.03 ]
	)
	block (
		points 16
		point[0]  [  0.032  0.013 ]
		point[1]  [  0.025  0.025 ]
		point[2]  [  0.013  0.032 ]
		point[3]  [  0.000  0.035 ]
		point[4]  [ -0.013  0.032 ]
		point[5]  [ -0.025  0.025 ]
		point[6]  [ -0.032  0.013 ]
		point[7]  [ -0.035  0.000 ]
		point[8]  [ -0.032 -0.013 ]
		point[9]  [ -0.025 -0.025 ]
		point[10] [ -0.013 -0.032 ]
		point[11] [ -0.000 -0.035 ]
		point[12] [  0.013 -0.032 ]
		point[13] [  0.025 -0.025 ]
		point[14] [  0.032 -0.013 ]
		point[15] [  0.035 -0.000 ]
		z [ 0.03 0.06 ]
	)
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

	# Actual size
	size [ 0.35 0.35 0.26 ]

	# Center of rotation
	origin [ 0.0 0.0 0.0 0.0 ]
	
	# Estimated mass in kg
	mass 3.9
	
	# Body shape
	# this block approximates the circular shape of a Roomba
	block( 
		points 16
		point[0] [ 0.225 0.000 ]
		point[1] [ 0.208 0.086 ]
		point[2] [ 0.159 0.159 ]
		point[3] [ 0.086 0.208 ]
		point[4] [ 0.000 0.225 ]
		point[5] [ -0.086 0.208 ]
		point[6] [ -0.159 0.159 ]
		point[7] [ -0.208 0.086 ]
		point[8] [ -0.225 0.000 ]
		point[9] [ -0.208 -0.086 ]
		point[10] [ -0.159 -0.159 ]
		point[11] [ -0.086 -0.208 ]
		point[12] [ -0.000 -0.225 ]
		point[13] [ 0.086 -0.208 ]
		point[14] [ 0.159 -0.159 ]
		point[15] [ 0.208 -0.086 ]
		z [0 0.1]
	)
	
	block (
		points 8
		point[0] [  0.16  0.11  ]
		point[1] [  0.07  0.165 ]
		point[2] [ -0.01  0.165 ]
		point[3] [ -0.11  0.1   ]
		point[4] [ -0.11 -0.1   ]
		point[5] [ -0.01 -0.165 ]
		point[6] [  0.07 -0.165 ]
		point[7] [  0.16 -0.11  ]
		z [0.136 0.142]
	)
	
	block (
		points 8
		point[0] [  0.13  0.11  ]
		point[1] [  0.04  0.165 ]
		point[2] [ -0.04  0.165 ]
		point[3] [ -0.14  0.1   ]
		point[4] [ -0.14 -0.1   ]
		point[5] [ -0.04 -0.165 ]
		point[6] [  0.04 -0.165 ]
		point[7] [  0.13 -0.11  ]
		z [0.193 0.20]
	)
	
	# this bumper array VERY crudely approximates the Roomba's bumpers
#	bumper( size [.01 .01 .01 0]
#		bcount 2  
#		blength .33
#		bpose[0] [0.12  0.12 .02  45]
#		bpose[1] [0.12 -0.12 .02 -45] 
#	)
	
	color "red"				# Default color.
	drive "diff"		 	# Differential steering model.
	gui_nose 1				# Draw a nose on the robot so we can see which way it points

	kinect ( pose [ 0.1 0.0 -0.04 0.0 ])
	xition ( pose [ 0.1 0.0 -0.04 0.0 ])
	rp_lidar ( pose [ 0 0 0 0 ] )
)
