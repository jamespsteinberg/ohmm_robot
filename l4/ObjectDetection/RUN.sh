#!/bin/bash

    CLASSCOUNT=$(ls bin | wc -l)

    if [ $CLASSCOUNT -gt 0 ]
    then
       make clean
    fi
    make

    #Temporarily copy Grasp.java into src directory
    #cp ../../../ohmm-sw-site/hlp/ohmm/Grasp.java src/Grasp.java
 
       java -cp ../jars/OHMM-newest.jar:../jars/opencv/bin/opencv-244.jar:bin ObjectDetect
    #rm src/Grasp.java #Remove temporary file after compiling
