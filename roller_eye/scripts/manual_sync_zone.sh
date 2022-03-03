#!/bin/sh

if [ $# -eq 2 ]
then
    [ -f /etc/localtime ]&&{
	rm -f /etc/localtime
	}
	[ -f /root/.profile ]&&{
	#rm /root/.profile
	sed -i '/TZ=/d' /root/.profile
	}
	if [ -f /usr/share/zoneinfo/$1 ]
	then
	    ln -s /usr/share/zoneinfo/$1 /etc/localtime
	else
		echo export TZ=$2 >> /root/.profile
    
		aa=`cat /root/.profile|grep "GMT+"`
		bb=`cat /root/.profile|grep "GMT-"`
		if [ "-$aa" != "-" ]
		then
		sed -i 's/+/-/' /root/.profile
		fi
		if [ "-$bb" != "-" ]
		then
		sed -i 's/-/+/' /root/.profile
		fi
	fi
else
    [ -f /etc/localtime ]&&{
	rm -f /etc/localtime
	}
	[ -f /root/.profile ]&&{
	#rm /root/.profile
	sed -i '/TZ=/d' /root/.profile
	}
	if [ -f /usr/share/zoneinfo/$1 ]
	then
	    ln -s /usr/share/zoneinfo/$1 /etc/localtime
	else
		echo export TZ=$1 >> /root/.profile
    
		aa=`cat /root/.profile|grep "GMT+"`
		bb=`cat /root/.profile|grep "GMT-"`
		if [ "-$aa" != "-" ]
		then
		sed -i 's/+/-/' /root/.profile
		fi
		if [ "-$bb" != "-" ]
		then
		sed -i 's/-/+/' /root/.profile
		fi
	fi 
fi















