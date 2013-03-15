#!/bin/bash

CLASSCOUNT=$(ls bin | wc -l)
   
if [ $CLASSCOUNT -gt 0 ]
then
   make clean
fi
make
if [ $# -eq 1 ] #User provided port
then
   java -cp jars/OHMM-newest.jar:bin InverseKinematics $1
else #Use default 
   java -cp jars/OHMM-newest.jar:bin InverseKinematics "/dev/ttyACM1"
fi
