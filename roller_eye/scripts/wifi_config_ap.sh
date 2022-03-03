#!/bin/bash

hostapd_config_file=/etc/hostapd/hostapd.conf
hostapd_config_file_bak=/etc/hostapd/hostapd.conf.bak
sn_config_file=/var/roller_eye/config/sn

function check_result()
{
    if [ "$?" != "0" ];then
        echo "$1"
        if [ -f $hostapd_config_file_bak ];then
            echo "restore ap config..."
            mv  -f $hostapd_config_file_bak $hostapd_config_file
        fi
        exit 1
    fi
}

rm -f $hostapd_config_file_bak
if [ -f $hostapd_config_file ];then
    mv -f $hostapd_config_file $hostapd_config_file_bak
    if [ "$?" != "0" ];then
        echo "backup ap config file fail"
        exit 1
    fi
fi

sn=$(cat $sn_config_file)
check_result "get sn fail"
sn_md5=$(md5sum $sn_config_file|awk '{print $1}')
check_result "cal sn md5sum fail"

if [ "$1" == "" ];then
    ssid="ssid=robot-scout-${sn_md5:0-6}"
else
    ssid="ssid=$1"
fi

if [ "$2" == "" ];then
    passwd="wpa_passphrase=r0123456"
else
    passwd="wpa_passphrase=$2"
    touch /var/roller_eye/config/ap
fi

function wifi_config_append()
{
    echo "$1" >> $hostapd_config_file
    check_result "write ap config fail"
}

wifi_config_append "interface=wlan0"
wifi_config_append  "driver=nl80211"
wifi_config_append  "ieee80211n=1"
wifi_config_append  "$ssid"
wifi_config_append  "channel=36"
wifi_config_append  "hw_mode=a"
wifi_config_append  "ignore_broadcast_ssid=0"
wifi_config_append  "auth_algs=1"
wifi_config_append  "wpa=2"
wifi_config_append  "$passwd"
wifi_config_append  "wpa_key_mgmt=WPA-PSK"
wifi_config_append  "wpa_pairwise=TKIP"
wifi_config_append  "rsn_pairwise=CCMP"
wifi_config_append  "max_num_sta=1"