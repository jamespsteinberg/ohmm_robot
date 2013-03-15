#!/bin/bash

function printUsage(){
   echo "---------------------------------------------------------------------------------------------------------------------------"
   echo "|                                                    USAGE                                                                |"
   echo "---------------------------------------------------------------------------------------------------------------------------"
   echo "| $0:                            | OHMM will run on /dev/ttyACM1 serial port, and respond to keyboard input         |"
   echo "| $0 [port]                      | OHMM will run on specified serial port, and respond to keyboard input            |" 
   echo "| $0 [x] [y]                     | OHMM will run on /dev/ttyACM1 serial port, and grasp at given (x,y) coordinates  |"
   echo "| $0 [x] [y] [port]              | OHMM will run on specified serial port, and grasp at given (x,y) coordinates     |" 
   echo "|                                |                                                                                        |"
   echo "| Examples:                      |                                                                                        |"
   echo "| $0: /dev/tty.usbserial1        | Respond to user input, using port /dev/tty.usbserial                             |"
   echo "| $0: 40 30                      | Grasp at point (30mm, 40mm) in world frame, using default port (/dev/ttyACM1)    |"
   echo "| $0: 40 30 /dev/tty.usbserial1  | Grasp at point (30mm, 40mm) in world frame, using port /dev/tty.usbserial        |"
   echo "---------------------------------------------------------------------------------------------------------------------------"
}


if [ $# -gt 3 ]
then
    printUsage
else
    CLASSCOUNT=$(ls bin | wc -l)

   #Temporarily copy InverseKinematics file into source
    cp ../InverseKinematics/src/InverseKinematics.java src/InverseKinematics.java
    if [ $CLASSCOUNT -gt 0 ]
    then
       make clean
    fi
    make

    if [ $# == 0 ] #Use defaults
    then
       java -cp jars/OHMM-newest.jar:bin Grasping "/dev/ttyACM1"
    elif [ $# == 1 ] #Provided port
    then  
       java -cp jars/OHMM-newest.jar:bin Grasping $1
    elif [ $# == 2 ] #Provided x, y
    then
       java -cp jars/OHMM-newest.jar:bin Grasping $1 $2 "/dev/ttyACM1"
    elif [ $# == 3 ] #Provided x, y, port
    then
       java -cp jars/OHMM-newest.jar:bin Grasping $1 $2 $3
    fi

    #Remove the InverseKinematics source file
    rm src/InverseKinematics.java
fi