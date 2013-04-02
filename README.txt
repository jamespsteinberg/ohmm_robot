L1: Implementing basic drive commands.
	
L2:
	Global Navigation: Takes in a series of points in a world view and guides the robot to them.
	Local Navigation: Guides robot to an inputted coordinate point in world view using a Bug2 algorithm.
	OhmmMapClient: A live GUI of the robot in world frame from a top-view perspective.

L3:
	Grasping: Grabs and Retreives an object using the arm with an inputted coordinate point.
	InverseKinematics: Implements basic arm movement commands in a 3D world frame using the keyboard.

L4:
	DebugViewer: Robot hosts a live webserver streaming out its video feed in normal RGB colors, HSV values, and a binary Grayscale with blob tracking
	ObjectDetection: Searches and retreives an object after recognizing and clicking it through the DebugViewer while in its starting pose.
		
Programmed by:
	Jason Shrand
	James Steinberg
	Baba

Ohmm Robot:
	1 Orangutan SVP board powering the AVR
	1 PandaBoard powering the Arm
	2 Sharp Analog IR sensors
	2 Bump Sensors
	2 Drive motors powering normal wheels
	2 Wheels
	1 Mecanum wheel in the back
	1 Robotics arm with 4 degrees of freedom
	1 Camera

See picture included in folder.