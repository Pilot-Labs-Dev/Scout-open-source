#ifndef __CAMERA_HANDLE__HPP__
#define __CAMERA_HANDLE__HPP__

#include <ros/ros.h>
#include <atomic>
#include <thread>
#include <mutex>
#include"roller_eye/plterrno.h"
#include"roller_eye/single_class.h"
#include"roller_eye/video_stream.h"

namespace roller_eye {

typedef std::function<void(CaputreParam*,bool)> ParamListener;
typedef std::function<void(int)> VideoStreamEventCB;

class CameraHandle: public SingleClass<CameraHandle>
{
public:
    // static CameraHandle *getInstance();
void newCameraHandle(int w, int h, int frm_num, int frm_rate_den,uint32_t fmt, int h264Quality, int cameraLight);

VSHandle createVideoStream(int buffCnt=0);
CaputreParam getCaptureParam(void);
void registParamListener(ParamListener recv);
void registVideoStreamEventCB(const VideoStreamEventCB& cb);
void sendVideoStreamEvent(int evt);
~CameraHandle();

private:
friend class SingleClass<CameraHandle>;
CameraHandle(const CameraHandle &);
CameraHandle& operator=(const CameraHandle &);

CameraHandle();
CMHandle camera_handle_create(CaputreParam *param);
bool isNewParam(int w, int h, int frm_rate_num, int frm_rate_den, uint32_t fmt, int h264Quality, int cameraLight);

CaputreParam m_param;
// VSHandle vsHandle;
CMHandle cmHandle;
std::vector<ParamListener> m_parListeners;
std::mutex m_mutex;
VideoStreamEventCB mEvt;
};

void set_image_grey_mode(bool grey);

} // namespace roller_eye

#endif // __CAMERA_HANDLE__HPP__

