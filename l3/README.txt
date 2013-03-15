--------------------------------------------------------------------------------
INSTRUCTIONS
--------------------------------------------------------------------------------
INVERSE KINEMATICS:

COMPILE / RUN:  Run the following command from terminal in this directory

./RUN.sh -I [port]
   port:  Optional parameter to specify OHMM serial port to connect with
          If omitted, the default port /dev/ttyACM1 is used.

Examples:  

./RUN.sh -I
 Uses default /dev/ttyACM1 port to run Inverse Kinematics program

./RUN.sh -I /dev/tty-usbserial1
 Uses port named tty-usbserial1 instead of default

--------------------------------------------------------------------------------
GRASPING:

COMPILE / RUN:  Run the following command from terminal in this directory

./RUN.sh -G [x y] [port]
   x, y:  Optional parameters that specify a world-frame location (x,y),
          in millimeters.  If omitted, the OHMM will revert to the
          keyboard-controlled Inverse Kinematics shown above.
   
   port:  Optional parameter to specify OHMM serial port to connect with
          If omitted, the default port /dev/ttyACM1 is used.

Examples:

./RUN.sh -G
 Will behave exactly the same as ./RUN.sh -I

./RUN.sh -G /dev/tty-usbserial1
 Will behave exactly the same as ./RUN.sh -I /dev/tty-usbserial1

./RUN.sh 30 40
 OHMM will grasp at location (30mm, 40mm) in world frame, using default port

./RUN.sh 30 40 /dev/tty-usbserial1
 OHMM will grasp at location (30mm, 40mm) in world frame, using port
 /dev/tty-usbserial1
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
KNOWN BUGS:

-Inverse Kinematics behaves strangely if robot is rotated by over PI/2 radians in world frame.

-Gripper won't shut all the way sometimes.  It seems to become uncalibrated as we use it

