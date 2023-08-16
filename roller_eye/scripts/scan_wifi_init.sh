#!/bin/bash
INTERFACE_NAME=wlan0

#check wifi sta mode is ok or not
ifconfig $INTERFACE_NAME | grep '\s*inet'
if [ "$?" == "0" ];then
		ps aux|grep wpa_supplicant|grep -v grep
		if [ "$?" == "0" ];then
			echo "wifi is connected!!!"
			exit 0
		fi
fi

systemctl stop wpa_supplicant.service
systemctl stop hostapd.service
systemctl stop udhcpd.service

sleep 1
killall wpa_supplicant
killall hostapd
killall udhcpd
dhclient_pid=$(ps aux|grep dhclient|grep $INTERFACE_NAME|awk '{print $2}')
if [ "$dhclient_pid" != "" ];then
	echo stop dhclient $dhclient_pid
	kill $dhclient_pid
fi
ip addr flush dev wlan0
sleep 1

wpa_supplicant -i $INTERFACE_NAME -c /etc/wpa_supplicant/wpa_supplicant.conf &