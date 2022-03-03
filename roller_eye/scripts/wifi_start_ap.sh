#!/bin/bash
INTERFACE_NAME=wlan0
MASTER_IP=10.42.0.1
MAX_WAIT_TIME=30
hostapd_config_file=/etc/hostapd/hostapd.conf
hostapd_config_file_bak=/etc/hostapd/hostapd.conf.bak

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
sleep 2

function exit_connect()
{
	echo "start wifi fail"
	exit 1
}

if [ ! -f $hostapd_config_file ];then
	/usr/local/bin/wifi_config_ap.sh ""	""
	if [ "$?" != "0" ];then
		echo "config wifi fail"
		exit_connect
	fi
fi

MASTER_SSID=$(cat $hostapd_config_file |grep '^ssid'|sed 's/ssid=//g')

systemctl start hostapd.service
for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	ssid=$(iwconfig wlan0|grep ESSID|sed 's/\s*$//g')
	ssid=${ssid#*ESSID:}
	echo "read ssid:\"$ssid\",record ssid:\"$MASTER_SSID\""
	if [ "$ssid" == "\"$MASTER_SSID\"" ];then
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait setup ap mode ..."
		sleep 1
	else
		echo "setup ap mode fail"
		exit_connect
	fi
done

ifconfig wlan0 $MASTER_IP netmask 255.255.255.0 up

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	ip=$(ifconfig wlan0|grep inet|grep netmask|awk '{print $2}')
	echo ip is:$ip
	if [ "$ip" == "$MASTER_IP" ];then
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait setup ip ..."
		sleep 1
	else
		echo "set ip fail"
		exit_connect
	fi
done

systemctl start udhcpd.service

echo "start wifi ok"


