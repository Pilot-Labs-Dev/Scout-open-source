#!/bin/bash
function ergodic(){
  for file in `ls $1`
  do
    if [ -d $1"/"$file ]
    then
      echo ""
    else
      local path=$1"/"$file 
      sz=$(stat -c %s /$path | tr -d '\n')     
      if [ $sz -gt 102400 ]; then  
	      truncate -s 0 $path
      fi
    fi
  done
}
IFS=$'\n' 
INIT_PATH="/var/log";
ergodic $INIT_PATH

TMP_LOG_PATH="/tmp/latest/";
ergodic $TMP_LOG_PATH
