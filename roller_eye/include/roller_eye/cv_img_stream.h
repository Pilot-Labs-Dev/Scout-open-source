#ifndef __ROLLER_EYE_CV_IMG_H
#define __ROLLER_EYE_CV_IMG_H

#include<vector>
#include <mutex>
#include<opencv2/opencv.hpp>
#include<roller_eye/video_stream.h>
#include"camera_handle.hpp"

using namespace std;
namespace roller_eye{
class CVGreyImageStream{
public:
    CVGreyImageStream(int width,int height);
    ~CVGreyImageStream();
    int getCVImg(cv::Mat &img,bool autoRetry=true);
    void stopStream();
    void onParamChanged(CaputreParam *param, bool resume);
private:
    int mWidth;
    int mHeight;
    vector<uint8_t> mGreyData;
    VSHandle mStream;
    recursive_mutex mReMutex;
    bool mNeedResume;
};
void getROICVImg(cv::Mat& src,cv::Mat& dst,cv::Rect2f &roi);
}
#endif