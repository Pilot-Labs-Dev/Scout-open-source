#!/bin/sh

SCOUT_DEFAULT_CONFIG_DIR="/var/roller_eye/config/"
SCOUT_DYNAMIC_DATA_DIR="/userdata/roller_eye/"

touch "/userdata/scout_resetting"

sync
sleep 3

echo "reset:stopping roller_eye service"
systemctl stop roller_eye

rm $SCOUT_DEFAULT_CONFIG_DIR"ap"
rm $SCOUT_DEFAULT_CONFIG_DIR"wifi"
/usr/local/bin/wifi_config_ap.sh "" ""
/usr/local/bin/manual_sync_zone.sh "GMT+0000"

rm $SCOUT_DEFAULT_CONFIG_DIR"userID"
rm $SCOUT_DEFAULT_CONFIG_DIR"lastPatrolName"

#echo "a" > $SCOUT_DEFAULT_CONFIG_DIR"userID"
cp $SCOUT_DEFAULT_CONFIG_DIR"monitor_default.json" $SCOUT_DEFAULT_CONFIG_DIR"monitor"
cp $SCOUT_DEFAULT_CONFIG_DIR"video_default.yaml" $SCOUT_DEFAULT_CONFIG_DIR"video.yaml"
cp $SCOUT_DEFAULT_CONFIG_DIR"soundEffect_default.json" $SCOUT_DEFAULT_CONFIG_DIR"soundEffect"
# cp $SCOUT_DEFAULT_CONFIG_DIR"motion_default" $SCOUT_DEFAULT_CONFIG_DIR"motion"

sync

rm -rf $SCOUT_DYNAMIC_DATA_DIR"cache/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"media_files/"
sync
rm -rf $SCOUT_DYNAMIC_DATA_DIR"sensor/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"navigate/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"navigate_jpg/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"ready/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"scratch/"
rm -rf $SCOUT_DYNAMIC_DATA_DIR"timer_task/"
sync

sleep 0.1

echo "reset:start roller_eye"

rm "/userdata/scout_resetting"
sync

reboot