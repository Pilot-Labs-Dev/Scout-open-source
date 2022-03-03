#!/bin/sh
if [ -f /etc/localtime ]
then
zone=$(ls -l /etc/|grep localtime |awk -F "zoneinfo/" '{print $2}')
else
zone=NONE
fi
echo "$zone"
