#ifndef __ROLLER_EYE_TRACK_TRACE_H__
#define __ROLLER_EYE_TRACK_TRACE_H__
#include<cstdio>
#include<vector>
#include<string>
#include<memory>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

using namespace std;
namespace roller_eye{
struct TrackPoint{
    TrackPoint()=default;
    TrackPoint(int64_t timestamp,Eigen::Vector3d &pos,Eigen::Quaterniond &ori,float vx,float vy,float w);
    int loadFromFile(FILE* file);
    int saveToFile(FILE* file);
    int64_t timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    float vx;
    float vy;
    float w;
};


class TrackList{
public:
    TrackList();
     int loadFromFile(const string& path);
     int saveToFile(const string& path);
     void push(TrackPoint& point);
     void pop();
     void clear();
     TrackPoint& back();
     TrackPoint& front();
     int size();
     void clip(Eigen::Vector3d&pos,int cnt);
     void erase(float distance);
     vector<TrackPoint> &TrackPointList();
     int64_t getMaxDistStamp() {return mMaxTimestamp;}
private:
    vector<TrackPoint> mPoints;
    int64_t mMaxTimestamp;
};

class TrackTrace{
public:
    TrackTrace();

    void setTrackList(shared_ptr<TrackList> tracks);

    void updatePose(Eigen::Vector3d&pos,Eigen::Quaterniond& q);

    bool traceOnce(float& vx,float &vy,float &w,float& yaw);

    int size();
    void clear();
    bool done();
    void erase(int cnt);
    void clipTrace(int cnt);
    float getTracePercent();
    double  distanceFrontPoint(Eigen::Quaterniond& q,Eigen::Vector3d& pos);
    shared_ptr<TrackList> &getTrackList();
private:
    void calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance);
    int mTrackOrigSize;
    Eigen::Vector3d mPositon;
    Eigen::Quaterniond mOrientation;
    shared_ptr<TrackList> mTracks;
    float mTrackLookDistance;
};
}
#endif
