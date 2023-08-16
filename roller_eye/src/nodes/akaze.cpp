#include<list>
#include<mutex>
#include<memory>
#include<functional>

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
using namespace std;
#ifndef 	SHM_FAILED
#define 	SHM_FAILED -1
#endif
#define FLAG IPC_CREAT|SHM_R|SHM_W

typedef struct
{
    int id;
    int rid;
    float angle;
}recordData;
const char* shmt_path = "/tmp";

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int writeAngle(float angle)
{
    key_t key;
    key = ftok(shmt_path, 1);
    if(key == -1){
        cout << "ftok failed" << endl;
    }
    int shmid;
    shmid = shmget(key,sizeof(recordData),FLAG);
    if(shmid == SHM_FAILED){
        cout<<"share memory create fail!"<<endl;
        return -1;
    }
    static int id=10;
    recordData *pRD;
    pRD = (recordData*)shmat(shmid,NULL,0);
    pRD->rid = pRD->id;
    pRD->angle = angle;

    if(shmdt(pRD) !=0){
        cout<<"share memory detach fail!"<<endl;
        return -1;
    }
    return 0;
}

/************************************************* 
Function:       match_akaze
Description:  Feature point matching. The detector, matcher and parameters 
                           selected here are related to the hardware resources of Scout.
                           If the amount of calculation is large, the resources may be insufficient
Input:         strBasePic- Previously saved pictures file name
                      strCurPic- Currently acquired picture file name
Output:      call writeAngle write to shared memory
Return:       true- success, false- fail
*************************************************/
int match_akaze(cv::Mat& subimg,cv::Mat& mainimg, std::vector<cv::KeyPoint>& keypoints_sub,
                                         std::vector<cv::KeyPoint>& keypoints_main, std::vector<cv::DMatch>& matches)
{
    Mat kpimg;
    //Ptr<AKAZE> detector = AKAZE::create();
    //Ptr<KAZE> detector = KAZE::create(false,false, 0.0001,3,4,KAZE::DIFF_CHARBONNIER);
    //Ptr<KAZE> detector = KAZE::create(false,false, 0.0001,3,4);
    //Ptr<KAZE> detector = KAZE::create();

   
    Ptr<KAZE> detector = KAZE::create(false,false, 0.0001,3,4,KAZE::DIFF_CHARBONNIER);
   //Ptr<AKAZE> detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3,0.0001f,3,4,KAZE::DIFF_CHARBONNIER);

    Mat descript_sub,descript_main;

    cout<<"KAZE::create"<<endl;
    /** Detects keypoints and computes the descriptors */
    detector->detectAndCompute(subimg,Mat(),keypoints_sub,descript_sub);
    detector->detectAndCompute(mainimg,Mat(),keypoints_main,descript_main);
    cout<<"detectAndCompute"<<endl;

    //BFMatcher matcher;
    BFMatcher matcher(NORM_L2SQR);
    matcher.match(descript_sub,descript_main,matches);
    cout<<"matcher"<<endl;

    return descript_sub.rows;
}

/************************************************* 
Function:       getDiffAngle
Description:  According to the feature points of the two pictures, the angle offset is calculated.
Input:         strBasePic- Previously saved pictures file name
                      strCurPic- Currently acquired picture file name
Output:      call writeAngle write to shared memory
Return:       true- success, false- fail
*************************************************/
bool getDiffAngle(string strBasePic, string strCurPic)
{
  float angle = 0.0;

  
  cout<<"getDiffAngle"<<endl;
  cv::Mat src    = cv::imread(strBasePic);
  cv::Mat grey = cv::imread(strCurPic);

  if (NULL==src.data ||  NULL==grey.data){
      return false;
  }

  
  vector<cv::KeyPoint> keypoints_sub;
  vector<cv::KeyPoint> keypoints_main;
  vector<cv::DMatch> matches;

  cout<<"match_akaze"<<endl;

  //Feature point matching
  int nRows = match_akaze(grey,src, keypoints_sub, keypoints_main, matches);

  float maxdist = 0;
  float mindist = 1000;

  for(int i=0;i<nRows;i++){
       float distance = matches[i].distance;
       if(distance>maxdist){
           maxdist = distance;
       }
       if(distance<mindist){
           mindist = distance;
       }
  }

    /*
    The field of view angle of the camera is 120 degrees. Assuming that the maximum offset angle is 15 degrees,
     pass max_y_shift and max_x_shift shift valve value filters the feature points that meet the requirements,
       and calculates the offset angle according to the difference of x-axis.
    */
   int img_height = src.rows;
   int img_width = src.cols;
   float camera_angle=120;
   float max_shift=15;
   float max_y_shift=max_shift/camera_angle*img_height;
   float max_x_shift=max_shift/camera_angle*img_width;

   float x=0;
   float x1=0;
   int count = 0;
   vector<cv::DMatch> goodmatches;
   for(int i=0;i<nRows;i++){
       //float distance = matches[i].distance;
       float xDiff = keypoints_sub[matches[i].queryIdx].pt.x-keypoints_main[matches[i].trainIdx].pt.x;
       float yDiff = keypoints_sub[matches[i].queryIdx].pt.y-keypoints_main[matches[i].trainIdx].pt.y;
       if (abs(xDiff)<max_x_shift && abs(yDiff)<max_y_shift){
           x += keypoints_sub[matches[i].queryIdx].pt.x;
           x1 += keypoints_main[matches[i].trainIdx].pt.x;
           count++;
           goodmatches.push_back(matches[i]);
       }
  }

  cout<<"keypoints_sub size="<<keypoints_sub.size() << " keypoints_main size=" <<keypoints_main.size()<<endl;

  if (count>2){
      //angle = 120*(x1-x)/(count*1280);
      angle = 120*(x1-x)/(count*img_width);
   }
  angle = (angle*M_PI)/180;

  cout<<"angle="<<angle<<endl;
  writeAngle(angle);
  return true;
}

int main(int argc, char **argv)
{
    if (argc != 3){
        return -1;
    }

    //Get angle offset from shared memory
    getDiffAngle(argv[1],argv[2]);
    //clGetDiffAngle(argv[1],argv[2]);
    return 0;
}