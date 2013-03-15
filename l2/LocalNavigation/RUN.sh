#!/bin/bash

CLASSCOUNT=$(ls bin | wc -l)
if [ -z "$1" ]
then
   echo "Usage: $0 <goal-x-coor (meters)>"
   echo "Example: $0 3.5"
else
   if [ $CLASSCOUNT -gt 0 ]
   then
      make clean
   fi
   make
   sudo iptables -I INPUT 1 -p tcp -m tcp --dport 4321 -j ACCEPT
   java -cp jars/OHMM-newest.jar:bin OhmmNavigation $1
fi
