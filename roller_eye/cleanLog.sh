#!/bin/bash

function ergodic(){
  for file in `ls $1`
  do
    if [ -d $1"/"$file ]
    then
      echo " "
    else
      local path=$1"/"$file 
      local name=$file      
      local size=`du --max-depth=1 $path|awk '{print $1}'` 
      if [ $size -gt 1024 ]; then
        #clean it
        echo "" > $path
      fi
    fi
  done
}
IFS=$'\n' 
INIT_PATH="/var/log";
ergodic $INIT_PATH
