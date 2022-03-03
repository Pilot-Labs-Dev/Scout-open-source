function killLastBkProc()
{
    last_proc=$!
    ps $last_proc|grep $1
    if [ "$?" == "0" ];then
        kill $last_proc
    else
        echo -e "\033[31m process $1 not exist\033[0m"
    fi
}

ROLLER_EYE_CONFIG_PATH=/var/roller_eye/config
function gen_p2p_config_file()
{
    CONFIG_PATH="$(dirname "$0")"/config.txt
    if [ -f $CONFIG_PATH ];then
        return
    fi
    auth_path=$ROLLER_EYE_CONFIG_PATH/p2p_auth
    pass_path=$ROLLER_EYE_CONFIG_PATH/p2p_passwd

    if [ ! -f $auth_path ];then
        echo -e "\033[31m p2p auth config not exist\033[0m"
        exit 1
    fi
    if [ ! -f $pass_path ];then
        echo -e "\033[31m p2p passwd config not exist\033[0m"
        exit 1
    fi

    auth_key=$(cat $auth_path)
    passwd=$(cat $pass_path)

    echo "iotc_auth_key:$auth_key" >> $CONFIG_PATH
    echo "av_account:admin" >> $CONFIG_PATH
    echo "av_password:$passwd" >> $CONFIG_PATH
    echo "av_identity:admin" >> $CONFIG_PATH
    echo "av_token:888888" >> $CONFIG_PATH
}
function get_p2p_uid()
{
    p2p_path=$ROLLER_EYE_CONFIG_PATH/p2p_uid
    if [ ! -f $p2p_path ];then
        echo -e "\033[31m p2p uid config not exist\033[0m"
        exit 1
    fi
    cat $p2p_path
}