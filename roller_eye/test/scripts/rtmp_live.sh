#/bin/bash
#create a rtmp stream,view it via ffplay 
sn_path=/var/roller_eye/config/sn
if [ -f  $sn_path ];then
	sn=`cat $sn_path`
else
	sn="unkown"
	echo -e "\033[33mSN not found,set to $sn \033[0m"
fi
lebal="test"

stamp=`date "+%Y%m%d%H%M%S"`
streamName=$sn-$stamp-$lebal

duration=1000000

sock_proxy_list_path=/opt/sockproxy/proxy_list.json
rtmp_uri=`cat /opt/sockproxy/proxy_list.json |grep -A 3 60003|grep '"dst"'|awk -F "\"" '{print $4}'`
echo $rtmp_uri | grep '[0-9a-zA-Z]\+\.[0-9a-zA-Z]\+:[0-9]\+'
if [ "$?" != "0" ];then
	echo invlid uri:$rtmp_uri 
	exit 1
fi
full_uri=rtmps://publisherName:123456@$rtmp_uri/live/$streamName
echo -e "\033[32mcreate live stream $full_uri ,duration is $duration secs\033[0m"
rosservice call /RTMPNode/rtmp_start "live" $streamName 0 1 $duration 0

ffplay -fflags nobuffer $full_uri

echo -e "\033[32mremove live stream $full_uri\033[0m"
rosservice call /RTMPNode/rtmp_stop "live" $streamName
