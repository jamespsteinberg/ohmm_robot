|==============================================================================
|  INSTRUCTIONS                                                                |
--------------------------------------------------------------------------------
 COMPILE / RUN:   Run the following command from terminal in this directory.

./RUN.sh
 

SOURCE CODE:
  Our final code is defined in two files:
    ObjectDetection/src/ObjectDetect.java
    ObjectDetection/src/VisualServo.java
--------------------------------------------------------------------------------
KNOWN BUGS:
   1.  In the three-stacked compound image, clicking a point in the bottom two
     images will result in an array out-of-bounds error.

   2.  The robot can lose track of pink ball beneath its own arm
   LIST ANY OTHER BUGS YOU KNOW HERE

   3.  There tends to be a 3-4 second lag between the frame being processed
       and the actual position of the robot
