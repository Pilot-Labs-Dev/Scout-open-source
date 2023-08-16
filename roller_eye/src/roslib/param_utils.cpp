#include"roller_eye/param_utils.h"
#include"roller_eye/ros_tools.h"
#include"roller_eye/plt_config.h"
#include"roller_eye/motion_set_zone.h"
#include"roller_eye/plog_plus.h"
#include "roller_eye/nav_get_status.h"

namespace roller_eye{
    static const char* PARAM_UTIL_TAG="ParamUTils";
    void load_monitor_param(MonitorParam &param)
    {
        json data;
        PltConfig::getInstance()->getMonitorParam(data);
        if(parse_monitor_param(data,param)<0){
            param.person=false;
            param.motion=false;
            param.cat=false;
            param.dog=false;
            param.zone.enable=false;
            param.zone.contours.clear();
        }
    }
    int parse_monitor_param(json& data,MonitorParam &param)
    {
        try{
            param.person=data["person"];
            param.dog=data["dog"];
            param.cat=data["cat"];
            param.motion=data["motion"];

            param.zone.enable=data["zone"]["active"];
            param.zone.motion=data["zone"]["motion"];
            param.zone.contours.clear();
            for(auto& area:data["zone"]["areas"]){
                roller_eye::contour con;
                con.inside=area["inside"];
                roller_eye::point point;
                for(auto& contour:area["contour"]){
                    point.x=contour["x"];
                    point.y=contour["y"];
                    con.points.push_back(point);
                }
                param.zone.contours.push_back(std::move(con));
            }
            return 0;
        }catch(...){
            return -1;
        }
    }
    void dump_monitor_param(const MonitorParam& param,json& data)
    {
        data["person"]=param.person;
        data["motion"]=param.motion;
        data["dog"]=param.dog;
        data["cat"]=param.cat;
        data["zone"]["active"]=param.zone.enable;
        data["zone"]["motion"]=param.zone.motion;
        data["zone"]["areas"]=json::array();
        auto& areas=data["zone"]["areas"];
        for(auto& contour:param.zone.contours){
            json con;
            con["inside"]=(bool)contour.inside;
            con["contour"]=json::array();
            auto &points=con["contour"];
            for(auto& p:contour.points){
                json point;
                point["x"]=p.x;
                point["y"]=p.y;
                points.push_back(point);
            }
            areas.push_back(std::move(con));
        }
    }
    MonitorHandle::MonitorHandle()
    {
        mSetZone=mHandle.serviceClient<roller_eye::motion_set_zone>("CoreNode/motion_set_zone");
        mGetNavStatusClient = mHandle.serviceClient<nav_get_status>("NavPathNode/nav_get_status");
    }
    void MonitorHandle::setupMonitor(const MonitorParam& param,bool close)
    {
        bool bPatrol = false;
        nav_get_status req;
        if(mGetNavStatusClient.call(req)){
            bPatrol = req.response.status;
        }
        PLOG_DEBUG(PARAM_UTIL_TAG, "setupMonitor bPatrol:%d %d %d", bPatrol, param.person, close);
        if(param.person && (!close || bPatrol)){
            PLOG_DEBUG(PARAM_UTIL_TAG, "%s:%d",__FILE__, __LINE__);
            if(!mPersion){
                PLOG_DEBUG(PARAM_UTIL_TAG, "%s:%d",__FILE__, __LINE__);
                mPersion=start_monitor_person(mHandle);
            }
        }else{
            if (mPersion){
                PLOG_DEBUG(PARAM_UTIL_TAG, "%s:%d",__FILE__, __LINE__);
                stop_monitor_person(mPersion);
            }
        }
        if(param.dog &&  (!close || bPatrol)){
            if(!mDog){
                mDog=start_monitor_dog(mHandle);
            }
        }else{
            if (mDog){
                stop_monitor_dog(mDog);
            }
        }
        if(param.cat &&  (!close || bPatrol)){
            if(!mCat){
                mCat=start_monitor_cat(mHandle);
            }
        }else{
            if (mCat){
                stop_monitor_cat(mCat);
            }
        }
        bool motion=param.motion||(param.zone.enable && param.zone.motion);
        if(motion && !close){
            if(!mMotion){
                mMotion=start_monitor_motion(mHandle);
            }
        }else{
            if (mMotion){
                stop_monitor_motion(mMotion);
            }
        }
        //PLOG_INFO(PARAM_UTIL_TAG,"monitor status:person=%d,dog=%d,cat=%d,motion=%d,close=%d",param.person,param.dog,param.cat,motion,close);
    }
    int  MonitorHandle::setupZone(const MonitorParam& param)
    {
        roller_eye::motion_set_zone par;
        par.request.contours=param.zone.contours;
        if(!mSetZone.call(par)){
            return -1;
        }
        return 0;
    }

    int parse_vio_param(json& data,VioParam &param)
    {
        try{
            param.enable=data["enable"];
            return 0;
        }catch(...){
            return -1;
        }
    }

    void load_vio_param(VioParam &param)
    {
        json data;
        PltConfig::getInstance()->getMotionParam(data);
        if(parse_vio_param(data,param)<0){
            param.enable=false;
        }
    }


}