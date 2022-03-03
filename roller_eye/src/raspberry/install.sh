#! /bin/bash

echo -e "\033[32m Install motor server start ... \033[0m"

INSTALL_DIR=/opt/motor
OS_NAME=$(cat /etc/os-release |grep ^NAME=|awk -F '=' '{print $2}')
echo "OS is "$OS_NAME

SERVICE_PATH=/lib/systemd/system

if [ ! -d $INSTALL_DIR ];then
    mkdir $INSTALL_DIR
fi

systemctl stop motor.service
cp motor_serve.py $INSTALL_DIR/

cp motor.service $SERVICE_PATH
systemctl enable motor.service
systemctl start motor.service

echo -e "\033[32m Install motor server done! \033[0m"
