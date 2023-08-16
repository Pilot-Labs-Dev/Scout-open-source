#!/bin/bash

# set -x

if [ "$#" != "2" ];then
    echo "Param error: $*"
    exit 1
fi

# unit: KB
# ROOT_MIN_FREE=1000000000
ROOT_MIN_FREE=1000000

media_path="$1"
ros_ns="$2"


function del_oldest_item()
{
    oldest_created=`ls {*.mp4,*-1.jpg} | sort -r | tail -1`
    echo "oldest_created: $oldest_created"
    if [ -z "$oldest_created" ];then
        echo "oldest_created not found."
        return 1
    fi

    #"2019-11-25-14-48-59_LR2D6C3In32lVXwR_RETe8yTf68h0cjMb-14988.mp4"
    if [[ $oldest_created =~ .mp4$ ]];then
        echo "oldest: $oldest_created"
        tmp=${oldest_created#*_}
        jid=${tmp#*_}
        jid=${jid%-*}
        jid=${jid%.*}
        tmp=${tmp%_*}
        echo "mp4 case: $tmp"
    #"2019-11-23-15-29-11_k9iX1YeaV6R4fA1v-1.jpg"
    elif [[ $oldest_created =~ -1.jpg$ ]];then
        tmp=${oldest_created#*_}
        tmp=${tmp%-*}
        echo "jpg case: $tmp"
    fi

    if [[ $ros_ns =~ /$ ]];then
        srv_name="/RecorderAgentNode/record_delete_file"
    else
        srv_name=$ros_ns"/RecorderAgentNode/record_delete_file"
    fi

    res=$(rosservice call $srv_name $tmp)
    
    if [ "$res" != "status: 0" ]; then
       echo "not del include $jid files"
       rm *$jid*
    fi
}

function get_root_available_size()
{
    echo $( df | grep /$ |  awk '{print $4}')
}

function get_userdata_available_size()
{
    echo $( df | grep /userdata$ |  awk '{print $4}')
}

if [ ! -e "$media_path" ];then
    echo "media_path not exist: $media_path"
    exit 1
fi

available_size=$(get_userdata_available_size)
echo "ROOT_MIN_FREE: $ROOT_MIN_FREE, available_size: $available_size"

cd $media_path

while [ $available_size -lt $ROOT_MIN_FREE ]
do
    del_oldest_item
    if [ $? -ne 0 ];then
        echo "while break."
        break
    fi
    available_size=$(get_userdata_available_size)
    echo "available_size: $available_size"
    # sleep 1
done

cd -
echo "check done."
