#!/bin/bash

function printUsage(){
echo "---------------------------------------------------------------------------------------------------------------------------"
echo "|                                                    USAGE                                                                |"
echo "---------------------------------------------------------------------------------------------------------------------------"
echo "| $0 -G (--Grasping) [args]              | Runs Grasping program with the given arguments                           |"
echo "|                                              | (See README.txt for argument details)                                    |"
echo "|                                              |                                                                          |"
echo "| $0 -I (--InverseKinematics) [args]     | Runs Inverse Kinematics program with the given arguments                 |"
echo "|                                              | (See README.txt for argument details)                                    |"
echo "---------------------------------------------------------------------------------------------------------------------------"

}

if [ $# == 0 ]
then
   printUsage
else
  if [[ $1 == -G || $1 == -g || $1 == --Grasping ]]
  then
     shift
     cd Grasping
     ./RUN.sh $@
     cd ..
  elif [[ $1 == -I || $1 == -i || $1 == --InverseKinematics ]]
  then
     shift
     cd InverseKinematics
     ./RUN.sh $@
     cd ..
  else
     printUsage
  fi
fi