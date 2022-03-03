#ifndef __ROLLER_EYE_PARAM_UTILS__
#define __ROLLER_EYE_PARAM_UTILS__
#include<vector>
#include"nlohmann/json.hpp"
#include"roller_eye/contour.h"
#include"roller_eye/ros_tools.h"

using namespace std;
using json=nlohmann::json;

namespace roller_eye{

    struct MonitorZone{
        bool enable;
        bool motion;
        vector<roller_eye::contour> contours; 
    };
    struct MonitorParam{
        bool person;
        bool dog;
        bool cat;
        bool motion;
        MonitorZone zone;
    };
    struct VioParam{
        bool enable;
    };

    void load_monitor_param(MonitorParam &param);

    int parse_monitor_param(json& data,MonitorParam &param);

    void dump_monitor_param(const MonitorParam& param,json& data);

    class MonitorHandle{
    public:
        MonitorHandle();
        void setupMonitor(const MonitorParam& param,bool close);
        int setupZone(const MonitorParam& param);
    private:
        ros::Subscriber mPersion;
        ros::Subscriber mDog;
        ros::Subscriber mCat;
        ros::Subscriber mMotion;
        ros::NodeHandle mHandle;
        ros::ServiceClient mSetZone;
        ros::ServiceClient mGetNavStatusClient;

        ros::Publisher mEnableMotionPub;
    }; 

    void load_vio_param(VioParam &param);
}
#endif