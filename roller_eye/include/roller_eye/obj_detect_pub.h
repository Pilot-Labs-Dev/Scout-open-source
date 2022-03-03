#ifndef __OBJ_DETECT_PUB_H__
#define __OBJ_DETECT_PUB_H__

#include <ros/ros.h>
#include "roller_eye/detect.h"
#include "roller_eye/camera_handle.hpp"
#include "roller_eye/system_define.h"
#include "roller_eye/publisher.h"

#include "tensorflow/lite/model.h"
#include "tensorflow/lite/string.h"

#include "absl/memory/memory.h"
#include "tensorflow/lite/delegates/nnapi/nnapi_delegate.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"
#include "tensorflow/lite/profiling/profiler.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/tools/evaluation/utils.h"


using namespace std;
using namespace tflite;
using TfLiteDelegatePtr = tflite::Interpreter::TfLiteDelegatePtr;
using TfLiteDelegatePtrMap = std::map<std::string, TfLiteDelegatePtr>;

namespace roller_eye {

typedef std::function<void(const detect&)> ObjListener;
#define LOG(x) std::cerr

const string TF_FILE_HOME = string(ROLLER_EYE_FILE_HOME_PATH);
const string TF_OBJ_DETECT_MODEL_NAME = TF_FILE_HOME + "tflite/detect.tflite";
const string TF_OBJ_DETECT_LABEL_NAME = TF_FILE_HOME + "tflite/labelmap.txt";

const string TF_PILE_DETECT_MODEL_NAME = TF_FILE_HOME + "tflite/detect_quant_home.tflite";
const string TF_PILE_DETECT_LABEL_NAME = TF_FILE_HOME + "tflite/labelquantmap.txt";

struct Settings {
  bool verbose = false;
  bool accel = false;
  bool old_accel = false;
  bool input_floating = false;
  bool profiling = false;
  bool allow_fp16 = false;
  bool gl_backend = false;
  int loop_count = 1;
  float input_mean = 127.5f;
  float input_std = 127.5f;
  string model_name = TF_OBJ_DETECT_MODEL_NAME;
  tflite::FlatBufferModel* model = nullptr;
  string labels_file_name = TF_OBJ_DETECT_LABEL_NAME;
  int number_of_threads = 1;
  int number_of_results = 5;
  int max_profiling_buffer_entries = 1024;
  int number_of_warmup_runs = 2;
  // const string test_bmp_dir = "./tf_test_bmp";
  const string test_bmp_dir;
};


class ObjDetectPub
{
public:
  ObjDetectPub(ros::NodeHandle& nh, CameraHandle *ch,string name="obj", bool bChargingPile=false);
  ~ObjDetectPub();

  void subConnectObj(int cnt);
  void subDisconnectObj(int cnt);
  void startPublish();
  void stopPublish();
  void setObjListener(ObjListener listener);
  
private:
  FrameBuff *getFrame();
  void onParamChanged(CaputreParam *param, bool resume);
  void publishLoop();

  void tfliteInit(Settings *s);
  double get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }
  TfLiteDelegatePtrMap GetDelegates(Settings* s);
  TfLiteDelegatePtr CreateGPUDelegate(Settings* s);
  TfLiteStatus ReadLabelsFile(const string& file_name, std::vector<string>* result, size_t* found_label_count);
  void PrintProfilingInfo(const profiling::ProfileEvent* e, uint32_t op_index, TfLiteRegistration registration);

  int subRefNum;
  std::atomic<bool> m_pubRunning;
  std::thread m_pubThread;

  CameraHandle *mCameraHandle;
  VSHandle vsHandle;
  ObjListener mObjListener;
  Publisher<roller_eye::detect> mPub;
  ros::Publisher mSysEvtPub;

  Settings mSetting;
  std::vector<string> labels;
  size_t label_count;
  std::unique_ptr<tflite::FlatBufferModel> model;
  std::unique_ptr<tflite::Interpreter> interpreter;
  tflite::ops::builtin::BuiltinOpResolver resolver;

  bool mBChargingPile;
  static bool mStartDetectCharingPile;    
  bool mNeedResume;
};


} // namespace roller_eye

#endif  //__OBJ_DETECT_PUB_H__
