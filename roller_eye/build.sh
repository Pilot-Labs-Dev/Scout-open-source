#!/bin/bash
arch=`uname -m`
if [ ! -f DEBIAN/control.$arch ];then
    echo "archtecture not support "
    exit 1
fi
DEBIAN_CTL_NAME=control.$arch
FW_APP_VER=$(git rev-parse --short HEAD)
set -e
cd ../..

ROS_VER=
ROS_VER_NUM=`ls /opt/ros|wc -w`

if [ "$ROS_VER_NUM" == "0" ];then
    echo -e "\033[31mNone ROS SDK found \033[0m" 
    exit 1
fi

DEB_DIR=roller_eye
BUILD_TYPE=Debug

set -- $(getopt frv: "$@")

while [ -n $1 ]
do
    case "$1" in
    -r) BUILD_TYPE=Release;;
    -f)  rm -rf build devel;;
    -v) ROS_VER=$2
            shift;;
    --) shift
        break;;
    *) echo "param error!!!"
        exit 1;;
    esac
    shift
done

if [ "$ROS_VER_NUM" != "1" -a "$ROS_VER" == "" ];then
    ROS_VER=`ls /opt/ros`
    echo -e "\033[33mFound multiple ROS SDK:$ROS_VER ,use -v option to select\033[0m"
    exit  1
else
    ROS_VER=`ls /opt/ros`   
fi

echo -e "\033[32mROS Version:$ROS_VER \033[0m" 
ROS_DIR=/opt/ros/$ROS_VER
OUT=roller_eye-$arch-$BUILD_TYPE-$FW_APP_VER.deb

rm -rf install
rm -rf $DEB_DIR

echo -e "\033[32mBuild Type $BUILD_TYPE \033[0m"
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE --make-args -j6
cd build
echo -e "\033[32mMake install ....\033[0m"
make install -j4
cd ..
mkdir $DEB_DIR
cp -R src/roller_eye/DEBIAN $DEB_DIR
mv $DEB_DIR/DEBIAN/$DEBIAN_CTL_NAME $DEB_DIR/DEBIAN/control
rm -rf $DEB_DIR/DEBIAN/control.*
mv install/rootfs/* $DEB_DIR/
rm -rf install/rootfs
mkdir -p $DEB_DIR$ROS_DIR
mv install/include $DEB_DIR$ROS_DIR
mv install/lib $DEB_DIR$ROS_DIR
mv install/share $DEB_DIR$ROS_DIR
chmod 755 src/roller_eye/vio/vio
cp  src/roller_eye/vio/vio $DEB_DIR$ROS_DIR/lib/roller_eye
echo -e "\033[32mBuilding deb...\033[0m"
dpkg-deb -b $DEB_DIR $OUT
rm -rf install
rm -rf $DEB_DIR
rm -f src/roller_eye/roller_eye-$arch-$BUILD_TYPE-*deb
mv $OUT src/roller_eye/
echo -e "\033[32mBuild all done,output=$OUT\033[0m"




