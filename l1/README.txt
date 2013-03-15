README
   


--------------------------------------------------------------------------------
Compile Instructions:


make && make program
minicom
< Any command >

--------------------------------------------------------------------------------
Known Bugs:

When turning, the robot would consistently stop short a few degrees of the 
specified target.  We are unsure what is causing this, whether it be friction,
inertia, etc.  To compensate, we created a constant floating point value,
equal to 0.938 that we multiply the odometer value by when computing
change in orientation.  We came up with this value through trial and error
and found that its use improved our robot's accuracy.
We would like to find a better solution to this issue in the future. We will
try out different implentations for robot movement in the near future and 
figure out a better long-term solution. 
--------------------------------------------------------------------------------
Tests:

dtsq (Drive test square):
Takes in one floating number as an argument. Noodles will drive in a 
square starting with a left turn (if number is positive). The square will have
sides equal to the input.

dtb (Drive test boomerang):
Takes in one floating number as an argument. Noodles will drive forward the 
given distance, stop, and rotate 180 degrees counterclockwise. Then it will 
return to the starting position, rotate to the original position counterclockwise,
and stop.