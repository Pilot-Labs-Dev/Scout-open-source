#include<cmath>
#include<cfloat>
#include <iostream>
#include"track_trace.h"
#include "roller_eye/plt_config.h"
#include "roller_eye/param_utils.h"

#define TRACK_POINT_SIZE                11
#define TRACK_PIONT_FORMAT      "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%f,%f,%f\n"

#define TRACK_TRACE_TAG          "TrackTrace"

#define  TRACK_VELOCITY                     0.2 // track speed
//#define TRACK_LOOK_DISTANCE                       0.3
#define TRACK_LOOK_DISTANCE                       0.2
#define TRACK_MIN_DISTANCE                           0.1
#define TRACK_BIG_ANGLE                                  (M_PI*2/3)
namespace roller_eye{
    TrackPoint::TrackPoint(int64_t timestamp,Eigen::Vector3d &pos,Eigen::Quaterniond &ori,float vx,float vy,float w):
    position(pos),
    orientation(ori)
    {
        this->timestamp=timestamp;
        this->vx=vx;
        this->vy=vy;
        this->w=w;
    }
    int TrackPoint::loadFromFile(FILE* file)
    {
        if(file==NULL){
            return -1;
        }
        int64_t timestamp;
        double px,py,pz;
        double qw,qx,qy,qz;
        float vx,vy,w;
        int ret=fscanf(file,TRACK_PIONT_FORMAT,&timestamp,&px,&py,&pz,&qw,&qx,&qy,&qz,&vx,&vy,&w);
        if(ret==TRACK_POINT_SIZE){
            position.x()=px;
            position.y()=py;
            position.z()=pz;
            orientation.w()=qw;
            orientation.x()=qx;
            orientation.y()=qy;
            orientation.z()=qz;
            this->timestamp=timestamp;
            this->vx=vx;
            this->vy=vy;
            this->w=w;
            return 0;
        }
        return -1;
    }
    int TrackPoint::saveToFile(FILE* file)
    {
        if(file==NULL){
            return -1;
        }
        //position.x() = 0; //for test
        fprintf(file,TRACK_PIONT_FORMAT,timestamp,position.x(),position.y(),position.z(),
        orientation.w(),orientation.x(),orientation.y(),orientation.z(), vx,vy,w);
        return 0;
    }
    TrackList::TrackList()
    {
        mMaxTimestamp = 0;
    }
     int TrackList::loadFromFile(const string& path)
     {
         FILE *file=fopen(path.c_str(),"r");
         if(file==NULL){
             return -1;
         }
         TrackPoint point;
         while(!feof(file)){
            if(point.loadFromFile(file)!=0){
                mPoints.clear();
                break;
            }
            mPoints.push_back(point);
         }
         fclose(file);
         double maxDist = -1;
        Eigen::Quaterniond q = Eigen::Quaterniond(1.0,0.0,0.0,0.0);
         for (auto it=mPoints.begin(); it!=mPoints.end(); it++){
            Eigen::Vector3d d=q.inverse()*(mPoints.begin()->position-it->position);
            double dis=d.norm();
            if (dis > maxDist){
                mMaxTimestamp = it->timestamp;
                maxDist = dis;
            }
         }
         return mPoints.empty()?-1:0;
     }
     int TrackList::saveToFile(const string& path)
     {
         FILE *file=fopen(path.c_str(),"w");
         if(file==NULL){
             return -1;
         }
         for(auto p=mPoints.rbegin();p!=mPoints.rend();++p){
            if(p->saveToFile(file)!=0){
                fclose(file);
                remove(path.c_str());
                return -1;
            }
         }
         fclose(file);
         return 0;
     }
     void TrackList::push(TrackPoint& point)
     {
         mPoints.push_back(point);
     }
     void TrackList::pop()
     {
         mPoints.pop_back();
     }
     void TrackList::clear()
     {
         mPoints.clear();
     }
     TrackPoint& TrackList::back()
     {
         return mPoints.back();
     }

     TrackPoint& TrackList::front()
     {
         return mPoints.front();
     }

     int TrackList::size()
     {
         return (int)mPoints.size();
     }
     void TrackList::clip(Eigen::Vector3d&pos,int cnt)
     {
         int len=(int)mPoints.size();
         int minIdx=len-1;
         double minDist=DBL_MAX;
         double d;
         if (len<cnt){
             mPoints.resize(0);
             return;
         }
#if 1
         for(int i=minIdx,j=cnt;i>=0&&j>0;i--,j--){
            d=(pos-mPoints[i].position).norm();
             if(d<minDist){
                 minIdx=i;
                 if (minDist > DBL_MAX/2){
                    minDist=d;
                 }
                 std::cout<<"minDist: "<<minDist<<std::endl;
             }
         }
         std::cout<<"TrackList::clip minIdx "<<minIdx<<", minDist: "<<minDist<<std::endl;
#else
        if (len>cnt){
            minIdx = len-cnt;
        }
#endif
        //  if(len-minDist>cnt/4&&len>=cnt){
        //      mPoints.resize(len-cnt);
        //  }
         if(len>cnt){
             mPoints.resize(len-cnt);
         }
     }

     void TrackList::erase(float distance)
     {
         int cnt = 0;
         Eigen::Quaterniond q = Eigen::Quaterniond(1.0,0.0,0.0,0.0);
         for (auto it=mPoints.begin(); it!=mPoints.end(); it++){
            Eigen::Vector3d d=q.inverse()*(mPoints.begin()->position-it->position);
            double dis=d.norm();
             //if (abs(mPoints.begin()->position.y() - it->position.y()) > distance){
             if ( dis > distance){
                 std::cout<<"TrackList::erase cnt "<<cnt<<" "<<dis<<std::endl;
                 break;
             }
             cnt++;
         }

         if (cnt>0){
             mPoints.erase(mPoints.begin(), mPoints.begin()+cnt);
         }
    }

    vector<TrackPoint> &TrackList::TrackPointList(){
		return mPoints;
    }

    TrackTrace::TrackTrace()
    {
        mTrackOrigSize = 0;
        DeviceDefaultConfig cfg;
        mTrackLookDistance = cfg.getTrackLookDistance();
    }
    void TrackTrace::calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance)
    {
        Eigen::Vector3d d=q.inverse()*(goal-pos);
        distance=d.norm();
        if(distance<TRACK_MIN_DISTANCE/2){
            angle=0;
            return;
        }
        angle=std::asin(d.y()/distance);
        if(d.x()<0){
            angle=(angle>0?(M_PI-angle):(-M_PI-angle));
        }
        angle-=M_PI_2;
        if(angle<-M_PI){
            angle+=2*M_PI;
        }
    }
    void TrackTrace::setTrackList(shared_ptr<TrackList> tracks)
    {
        mTracks=tracks;
        mTrackOrigSize = mTracks->size();
    }

    shared_ptr<TrackList> & TrackTrace::getTrackList()
    {
		return mTracks;
    }

    float TrackTrace::getTracePercent()
    {
        float per = 1.0;
        if (mTracks && mTrackOrigSize>0){
            per = static_cast<float>(mTracks->size())/mTrackOrigSize;
        }

        return per;
    }

    int TrackTrace::size()
    {
        return mTracks->size();
    }

    void TrackTrace::updatePose(Eigen::Vector3d&pos,Eigen::Quaterniond& q)
    {
        mPositon=pos;
        mOrientation=q;
    }
    void TrackTrace::clipTrace(int cnt)
    {
        mTracks->clip(mPositon,cnt);
    }

    void TrackTrace::clear()
    {
        mTracks->clear();
    }
    void TrackTrace::erase(int cnt)
    {

    }

    bool TrackTrace::traceOnce(float& vx,float &vy,float &w,float& yaw)
    {
        vx=vy=w=0.0;
        yaw = 0.0;
        if(mTracks->size()==0){
            return false;
        }
        double distance;
        double angle;
        bool bigAngle;
        int64_t maxTimeStamp = mTracks->getMaxDistStamp();
        while(mTracks->size()!=0){
            calDistanceAndAngle(mOrientation,mPositon,mTracks->back().position,angle,distance);
            bigAngle=abs(angle)>TRACK_BIG_ANGLE;
            // PLOG_DEBUG(TRACK_TRACE_TAG, "traceOnce angle:%f distance:%f, bigAngle: %d\n",angle,distance, bigAngle);
            if(distance>mTrackLookDistance||mTracks->size()==1||bigAngle){
                break;
            }else{
                if (maxTimeStamp == mTracks->back().timestamp){
                    mTracks->pop();
                    break;
                }
                mTracks->pop();
            }
        }
        if(distance<TRACK_MIN_DISTANCE&&mTracks->size()==1){
            mTracks->pop();
            return false;
        }
        if(bigAngle){//this means a U turn
            yaw=(float)angle;
            std::cout<<"TrackTrace::traceOnce discard "<<distance<<" cm"<<std::endl;
            return true;
        }

        vx = 0.0;
        vy = TRACK_VELOCITY;
        w = (distance > 0.0) ? 2*std::sin(angle)*vy/distance : 0;
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
		{
			if (abs(vx)<0.01 && abs(vy)<0.01)
				w += 2.0;		///1.5;                 ///roll for belt
		}
         if ((mTracks->size()>30) && abs(angle) < 0.0526){   //for test ,don't roll when angle < 3 degrees
             w=0;
         }
		// PLOG_DEBUG(TRACK_TRACE_TAG, " vx: %f,vy: %f,w: %f,yaw: %f)",vx,vy,w,yaw );
        return false;
    }
    bool TrackTrace::done()
    {
        if(!mTracks){
            return true;
        }
        return mTracks->size()==0;
    }

    double  TrackTrace::distanceFrontPoint(Eigen::Quaterniond& q,Eigen::Vector3d& pos)
    {
        Eigen::Vector3d d=q.inverse()*(mTracks->front().position-pos);
        return d.norm();
    }
}
