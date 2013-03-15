#!/bin/bash


if [ -z "$1" ]
then
   echo "Usage: $0 ipAddress" 
else
   CLASSCOUNT=$(ls bin | wc -l)
   if [ $CLASSCOUNT -gt 0 ]
   then
      make clean
   fi
   make
   java -cp bin OhmmMapClient $1 4321
fi
