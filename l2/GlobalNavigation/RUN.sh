#!/bin/bash

CLASSCOUNT=$(ls bin | wc -l)
if [ -z "$1" ]
then
   echo "Usage: $0 <mapFileName>"
else
   if [ $CLASSCOUNT -gt 0 ]
   then
      make clean
   fi
   make
   cd bin/l2
   java -cp ../../jars/OHMM-newest.jar:../../bin l2/GNav $1
fi
