#!/bin/bash
INTERFACE_NAME=wlan0
MAX_WAIT_TIME=30

#check wifi sta mode is ok or not
ifconfig $INTERFACE_NAME | grep '\s*inet'
if [ "$?" == "0" ];then
		ps aux|grep wpa_supplicant|grep -v grep
		if [ "$?" == "0" ];then
			echo "wifi is connected!!!"
			#exit 0
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
sleep 2

wpa_supplicant -i $INTERFACE_NAME -c /etc/wpa_supplicant/wpa_supplicant.conf &

ssid="$1"
psk="$2"
bssid=""
empty_psk='""'

echo $ssid
echo $psk
echo $bssid

if [ ! $bssid ]; then
  echo "bssid IS NULL"
else
  echo "bssid NOT NULL"
fi

function exit_connect()
{
	killall wpa_supplicant
	echo "start wifi fail"
	exit 1
}

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	net_id=$(wpa_cli -i $INTERFACE_NAME add_network)
	if [ "$?" == "0" ];then
		echo "add network ok..."
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait add network ..."
		sleep 1
	else
		echo "add network fail..."
		exit_connect
	fi
done

if [ ! $bssid ]; then
  echo "bssid IS NULL"
else
	for((times=1;times<=$MAX_WAIT_TIME;times++))
	do
		res=$(wpa_cli -i $INTERFACE_NAME bssid $net_id  "$bssid")

		if [ "$res" == "OK" ];then
			echo "set bssid ok ..."
			break
		fi
		if [ "$times" != "$MAX_WAIT_TIME" ];then
			echo "wait set bssid ..."
			sleep 1
		else
			echo "set bssid fail .."
			exit_connect
		fi
	done
fi

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	res=$(wpa_cli -i $INTERFACE_NAME set_network $net_id ssid "$ssid")
	if [ "$res" == "OK" ];then
		echo "set ssid ok..."
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait set ssid ..."
		sleep 1
	else
		echo "set ssid fail..."
		exit_connect
	fi
done

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	if [ "$psk" != "$empty_psk" ];then
		res=$(wpa_cli -i $INTERFACE_NAME set_network $net_id psk "$psk")
	else
		res=$(wpa_cli -i $INTERFACE_NAME set_network $net_id key_mgmt NONE)
	fi
	if [ "$res" == "OK" ];then
		echo "set psk ok ..."
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait set psk ..."
		sleep 1
	else
		echo "set psk fail .."
		exit_connect
	fi
done

for((times=0;times<=10;times++))
do
	res=$(wpa_cli -i $INTERFACE_NAME set_network $net_id scan_ssid 1)

	if [ "$res" == "OK" ];then
		echo "set scan_ssid ok ..."
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait set scan_ssid ..."
		sleep 1
	else
		echo "set scan_ssid fail .."
		exit_connect
	fi
done

wpa_cli -i $INTERFACE_NAME select_network $net_id

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	res=$(wpa_cli -i $INTERFACE_NAME enable_network $net_id)
	if [ "$res" == "OK" ];then
		echo "enable network ok ..."
		break
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait enable network ..."
		sleep 1
	else
		echo "enable network fail"
		exit_connect
	fi
done

echo "get ip address ..."
dhclient $INTERFACE_NAME &

for((times=1;times<=$MAX_WAIT_TIME;times++))
do
	ifconfig $INTERFACE_NAME | grep '\s*inet'
	if [ "$?" == "0" ];then
		echo "get ip address ok"
		break;
	fi
	if [ "$times" != "$MAX_WAIT_TIME" ];then
		echo "wait get ip address ..."
		sleep 1
	else
		echo "get ip address fail"
		exit_connect
	fi
done

echo "start wifi ok"