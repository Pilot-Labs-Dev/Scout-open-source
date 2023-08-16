#include <fstream>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include "std_msgs/Int32.h"
#include "tf_detect/bitmap_helpers_impl.h"
#include "tf_detect/get_top_n.h"
#include "roller_eye/plt_tools.h"
#include "roller_eye/obj_detect_pub.h"
#include "roller_eye/graphic_utils.h"
#include "roller_eye/sound_effects_mgr.h"
#include"roller_eye/util_class.h"
#include "roller_eye/param_utils.h"

#include <sys/syscall.h>


using namespace std;

namespace roller_eye {

static int rm_dir(std::string dir_full_path)
{
    DIR* dirp = opendir(dir_full_path.c_str());
    if(!dirp)
    {
        perror("rm_dir:opendir");
        return -1;
    }
    struct dirent *dir;
    struct stat st;
    while((dir = readdir(dirp)) != NULL)
    {
        if (strcmp(dir->d_name,".") == 0 || strcmp(dir->d_name,"..") == 0)
        {
            continue;
        }
        std::string sub_path = dir_full_path + '/' + dir->d_name;
        if (lstat(sub_path.c_str(),&st) == -1)
        {
            ROS_ERROR("rm_dir:lstat failed in %s\n",sub_path.c_str());
            continue;
        }
        if (S_ISDIR(st.st_mode))
        {
            if(rm_dir(sub_path) == -1)
            {
                closedir(dirp);
                return -1;
            }
            rmdir(sub_path.c_str());
        }
        else if(S_ISREG(st.st_mode))
        {
            unlink(sub_path.c_str());
        }
        else
        {
            ROS_DEBUG("rm_dir:st_mode %d", st.st_mode);
            continue;
        }
    }
    if(rmdir(dir_full_path.c_str()) == -1)//delete dir itself.
    {
        closedir(dirp);
        return -1;
    }
    closedir(dirp);
    return 0;
}

static void checkTestBmpDir(const char *dirname) {
  if (access(dirname, 0) == 0) {
    rm_dir(dirname);
  }

  int flag = mkdir(dirname, 0777);
}


bool ObjDetectPub::mStartDetectCharingPile = false;

ObjDetectPub::ObjDetectPub(ros::NodeHandle& nh, CameraHandle *ch, string name, bool bChargingPile) :
subRefNum(0), m_pubRunning(false), mCameraHandle(ch), vsHandle(nullptr), mObjListener(nullptr),
mPub(nh, name, 10, bind(&ObjDetectPub::subConnectObj,this,placeholders::_1), bind(&ObjDetectPub::subDisconnectObj,this,placeholders::_1)),
mBChargingPile(bChargingPile),mNeedResume(false)
{
  ch->registParamListener(std::bind(&ObjDetectPub::onParamChanged, this, placeholders::_1, placeholders::_2));
  if (bChargingPile){
    mSetting.model_name=TF_PILE_DETECT_MODEL_NAME;
    mSetting.labels_file_name=TF_PILE_DETECT_LABEL_NAME;
  }
  tfliteInit(&mSetting);

  mSysEvtPub = nh.advertise<std_msgs::Int32>("/system_event",1);
}

ObjDetectPub::~ObjDetectPub()
{
  subRefNum = 0;
  m_pubRunning = false;

  if (vsHandle != nullptr) {
    video_stream_destory(vsHandle);
    vsHandle = nullptr;
  }
  mCameraHandle = nullptr;
}

void ObjDetectPub::setObjListener(ObjListener listener)
{
  mObjListener = listener;
}

void ObjDetectPub::onParamChanged(CaputreParam *param, bool resume)
{
  if ((!resume && !m_pubRunning) ||
        (resume && !mNeedResume)) {
          mNeedResume = false;
    return;
  }
  if (!resume){
    mNeedResume = true;
    stopPublish();
    video_stream_destory(vsHandle);
  }else {
    mNeedResume = false;
    vsHandle = mCameraHandle->createVideoStream(1);
    startPublish();
  }
}

FrameBuff *ObjDetectPub::getFrame() {
  return (subRefNum > 0) ? video_stream_get_frame(vsHandle) : nullptr;
}

void ObjDetectPub::subConnectObj(int cnt)
{
    ROS_DEBUG("subConnectObj subscriber cnt: %d", cnt);
    subRefNum = cnt;
    if (subRefNum > 0) {
        if (m_pubRunning == false) {
            vsHandle = mCameraHandle->createVideoStream(1);
            startPublish();
        } else {
            ROS_DEBUG("video stream already running.");
        }
    }
}

void ObjDetectPub::subDisconnectObj(int cnt)
{
    ROS_DEBUG("subDisconnectObj subscriber cnt: %d", cnt);
    subRefNum = cnt;
    if (subRefNum == 0) {
        if (m_pubRunning == true) {
            stopPublish();
            video_stream_destory(vsHandle);
            vsHandle = nullptr;
        } else {
            ROS_DEBUG("video stream already stop.");
        }
    }
}

void ObjDetectPub::startPublish()
{
  m_pubRunning = true;
  if (mBChargingPile){
    mStartDetectCharingPile = true;
    ROS_ERROR("mStartDetectCharingPile start");
  }
  m_pubThread = std::thread(&ObjDetectPub::publishLoop, this);
}

void ObjDetectPub::stopPublish()
{
  if(!m_pubRunning) {
    return;
  }

  if (mBChargingPile){
    mStartDetectCharingPile = false;
    ROS_ERROR("mStartDetectCharingPile stop");
  }

  m_pubRunning = false;
  m_pubThread.join();
}


void ObjDetectPub::publishLoop()
{
  pthread_t tid;
  tid =  syscall(SYS_gettid);
  printf("ObjDetectPub pid is %u \n", (unsigned int)tid);
  FrameBuff *frameBuff;
  detect objMsg;

  Settings *s = &mSetting;
  int video_width = mCameraHandle->getCaptureParam().sInfo.fInfo.width;
  int video_height = mCameraHandle->getCaptureParam().sInfo.fInfo.height;
  int fmt = mCameraHandle->getCaptureParam().sInfo.fInfo.fmt;

  uint32_t frame_seq = 0;
  int frame_data_size = video_width * video_height * 3;
  uint8_t* frame_data = new uint8_t[frame_data_size];
  std::vector<TfLiteTensor*> outputs;

  struct timeval start_time_total, stop_time_total;

	while (m_pubRunning) {
    if (mStartDetectCharingPile && !mBChargingPile){
      usleep(20*1000);
      continue;
    }

    gettimeofday(&start_time_total, nullptr);
    frameBuff = getFrame();
    if (frameBuff == nullptr) {
        continue;
    }

    objMsg.score = -1;
    int input = interpreter->inputs()[0]; //175

    // Save outputs
    outputs.clear();
    for(unsigned int i=0; i < interpreter->outputs().size(); i++) {
        outputs.push_back(interpreter->tensor(interpreter->outputs()[i]));
    }

    if (interpreter->AllocateTensors() != kTfLiteOk) {
      LOG(FATAL) << "Failed to allocate tensors!";
    }

    // get input dimension from the input tensor metadata
    // assuming one input only
    TfLiteIntArray* dims = interpreter->tensor(input)->dims;
    int wanted_height = dims->data[1];
    int wanted_width = dims->data[2];
    int wanted_channels = dims->data[3];

    if (s->verbose) {
      LOG(INFO) << "input tensor type:" << interpreter->tensor(input)->type << ". "
        << wanted_height << "x"<< wanted_width << "x" << wanted_channels << " \n";
    }

    if (fmt == V4L2_PIX_FMT_YUV420) {
      #ifndef APP_ARCH_X86
      GraphicUtils::getInstance()->rgaDrmConvert((uint8_t*)frameBuff->addr, video_width, video_height, RK_FORMAT_YCbCr_420_P,
        interpreter->typed_tensor<uint8_t>(input), wanted_width, wanted_height, RK_FORMAT_RGB_888);
      #else
        LOG(FATAL) << "RGA is only support of rockchip platform.";
      #endif
    } else if (fmt == V4L2_PIX_FMT_YUYV) {
      GraphicUtils::yuyv2rgb24((uint8_t*)frameBuff->addr, frame_data, video_width, video_height, video_width*2);
      int video_channels = 3;
      switch (interpreter->tensor(input)->type) {
        case kTfLiteFloat32: //1
          s->input_floating = true;
          resize<float>(interpreter->typed_tensor<float>(input), frame_data,
                        video_height, video_width, video_channels, wanted_height,
                        wanted_width, wanted_channels, s);
          break;

        case kTfLiteUInt8: //3
          resize<uint8_t>(interpreter->typed_tensor<uint8_t>(input), frame_data,
                          video_height, video_width, video_channels,
                          wanted_height, wanted_width, wanted_channels, s);
          break;
        default:
          LOG(FATAL) << "cannot handle input type "
                    << interpreter->tensor(input)->type << " yet";
          exit(-1);
      }
    }
    auto profiler =
      absl::make_unique<profiling::Profiler>(s->max_profiling_buffer_entries);
    interpreter->SetProfiler(profiler.get());

    if (s->profiling) profiler->StartProfiling();
    if (s->loop_count > 1)
      for (int i = 0; i < s->number_of_warmup_runs; i++) {
        if (interpreter->Invoke() != kTfLiteOk) {
          LOG(FATAL) << "Failed to invoke tflite!\n";
        }
      }

    // struct timeval start_time, stop_time;
    // gettimeofday(&start_time, nullptr);
    for (int i = 0; i < s->loop_count; i++) {
      if (interpreter->Invoke() != kTfLiteOk) {
        LOG(FATAL) << "Failed to invoke tflite!\n";
      }
    }

    if (s->profiling) {
      profiler->StopProfiling();
      auto profile_events = profiler->GetProfileEvents();
      for (int i = 0; i < (int)profile_events.size(); i++) {
        auto op_index = profile_events[i]->event_metadata;
        const auto node_and_registration =
            interpreter->node_and_registration(op_index);
        const TfLiteRegistration registration = node_and_registration->second;
        PrintProfilingInfo(profile_events[i], op_index, registration);
      }
    }

    if (outputs.size() >= 4) {
        const int    num_detections    = *TensorData<float>(outputs[3], 0);
        const float* detection_classes =  TensorData<float>(outputs[1], 0);
        const float* detection_scores  =  TensorData<float>(outputs[2], 0);
        const float* detection_boxes   =  TensorData<float>(outputs[0], 0);
        float maxScore=0.5;//min score is 0.5
        if (mBChargingPile){
          maxScore = 0.7;
        }
        int maxObjIdx=-1;
        for (int i = 0; i < num_detections; i++) {
            // Ignore first one
            if (detection_classes[i] + 1 == 0) continue;

            // Get max score
            if (detection_scores[i] > maxScore) maxObjIdx=i;
        }
        if(maxObjIdx>=0){
           int i=maxObjIdx;
           // Get class
            const int cls = detection_classes[i] + 1;

            // Get score
            float score = detection_scores[i];

            // Get class label
            const string label = labels[cls];

            // Get coordinates
            const float top    = detection_boxes[4 * i]     * video_height;
            const float left   = detection_boxes[4 * i + 1] * video_width;
            const float bottom = detection_boxes[4 * i + 2] * video_height;
            const float right  = detection_boxes[4 * i + 3] * video_width;

            objMsg.seq = ++frame_seq;
            objMsg.score = score;
            objMsg.index = cls;
            objMsg.name = label;

            if(objMsg.name.size()>0 && *objMsg.name.rbegin()=='\r'){//if label file is edit in windows,'\n' may be '\r\n'
              objMsg.name.erase(objMsg.name.end()-1);
            }
            objMsg.width = video_width;
            objMsg.height = video_height;
            objMsg.top = top;
            objMsg.left = left;
            objMsg.bottom = bottom;
            objMsg.right = right;
            objMsg.stamp = start_time_total.tv_sec * 1000000 + start_time_total.tv_usec;
        }
    }

    gettimeofday(&stop_time_total, nullptr);

    bool bValidDetect = true;
    MonitorParam param;
    load_monitor_param(param);
    if (objMsg.name.compare("person") == 0
            && !param.person){
        bValidDetect = false;
    }else if (objMsg.name.compare("cat") == 0
            && !param.cat){
        bValidDetect = false;
    }else if (objMsg.name.compare("dog") == 0
            && !param.dog){
        bValidDetect = false;
    }

    if (bValidDetect && objMsg.score > 0) {

      if (objMsg.name.compare("home")!=0){
          std_msgs::Int32 event;
          event.data = SYSEVT_OBJDETECTED;
          mSysEvtPub.publish(event);
      }

      mPub.publish(objMsg);
    }


    if (bValidDetect &&  mObjListener != nullptr) {
      mObjListener(objMsg);
    }
    video_stream_return_frame(vsHandle, frameBuff);
    usleep(20*1000);
  }

  if (mObjListener != nullptr) {
    objMsg.score = -1;
    mObjListener(objMsg);
  }

  delete[] frame_data;
}


void ObjDetectPub::tfliteInit(Settings *s)
{
  if (access(s->model_name.c_str(), 0) != 0) {
    ROS_FATAL("Not found model file: %s", s->model_name.c_str());
    return;
  }
  if (access(s->labels_file_name.c_str(), 0) != 0) {
    ROS_FATAL("Not found label file: %s", s->labels_file_name.c_str());
    return;
  }

  model = tflite::FlatBufferModel::BuildFromFile(s->model_name.c_str());
  if (!model) {
    LOG(FATAL) << "\nFailed to mmap model " << s->model_name << "\n";
    exit(-1);
  }
  s->model = model.get();
  model->error_reporter();

  tflite::InterpreterBuilder(*model, resolver)(&interpreter);
  if (!interpreter) {
    LOG(FATAL) << "Failed to construct interpreter\n";
    exit(-1);
  }

  interpreter->UseNNAPI(s->old_accel);
  interpreter->SetAllowFp16PrecisionForFp32(s->allow_fp16);

  if (s->verbose) {
    LOG(INFO) << "tensors size: " << interpreter->tensors_size() << "\n";
    LOG(INFO) << "nodes size: " << interpreter->nodes_size() << "\n";
    LOG(INFO) << "input(0) name: " << interpreter->GetInputName(0) << "\n";
    LOG(INFO) << "number of inputs: " << interpreter->inputs().size() << "\n";
    LOG(INFO) << "number of outputs: " << interpreter->outputs().size() << "\n";

    int t_size = interpreter->tensors_size();
    for (int i = 0; i < t_size; i++) {
      if (interpreter->tensor(i)->name)
        LOG(INFO) << i << ": " << interpreter->tensor(i)->name << ", "
                  << interpreter->tensor(i)->bytes << ", "
                  << interpreter->tensor(i)->type << ", "
                  << interpreter->tensor(i)->params.scale << ", "
                  << interpreter->tensor(i)->params.zero_point << "\n";
    }
  }

  if (s->number_of_threads != -1) {
    interpreter->SetNumThreads(s->number_of_threads);
  }

  auto delegates_ = GetDelegates(s);
  for (const auto& delegate : delegates_) {
    if (interpreter->ModifyGraphWithDelegate(delegate.second.get()) != kTfLiteOk) {
      LOG(FATAL) << "Failed to apply " << delegate.first << " delegate.";
    } else {
      LOG(INFO) << "Applied " << delegate.first << " delegate.";
    }
  }

  if (s->verbose) {
    LOG(INFO) << "\nPrintInterpreterState:\n";
    PrintInterpreterState(interpreter.get());
  }

  if (ReadLabelsFile(s->labels_file_name, &labels, &label_count) != kTfLiteOk) {
    LOG(FATAL) << "\nFailed to read labels file: " << s->labels_file_name << "\n";
    exit(-1);
  }

}


TfLiteDelegatePtr ObjDetectPub::CreateGPUDelegate(Settings* s)
{
#if defined(__ANDROID__)
  TfLiteGpuDelegateOptions options;
  options.metadata = TfLiteGpuDelegateGetModelMetadata(s->model->GetModel());
  if (s->allow_fp16) {
    options.compile_options.precision_loss_allowed = 1;
  } else {
    options.compile_options.precision_loss_allowed = 0;
  }
  options.compile_options.preferred_gl_object_type =
      TFLITE_GL_OBJECT_TYPE_FASTEST;
  options.compile_options.dynamic_batch_enabled = 0;

  return evaluation::CreateGPUDelegate(s->model, &options);
#else
  return evaluation::CreateGPUDelegate(s->model);
#endif
}

TfLiteDelegatePtrMap ObjDetectPub::GetDelegates(Settings* s)
{
  TfLiteDelegatePtrMap delegates;
  if (s->gl_backend) {
    auto delegate = CreateGPUDelegate(s);
    if (!delegate) {
      LOG(INFO) << "GPU acceleration is unsupported on this platform.";
    } else {
      delegates.emplace("GPU", std::move(delegate));
    }
  }

  if (s->accel) {
    auto delegate = evaluation::CreateNNAPIDelegate();
    if (!delegate) {
      LOG(INFO) << "NNAPI acceleration is unsupported on this platform.";
    } else {
      delegates.emplace("NNAPI", evaluation::CreateNNAPIDelegate());
    }
  }
  return delegates;
}

// Takes a file name, and loads a list of labels from it, one per line, and
// returns a vector of the strings. It pads with empty strings so the length
// of the result is a multiple of 16, because our model expects that.
TfLiteStatus ObjDetectPub::ReadLabelsFile(const string& file_name,
                            std::vector<string>* result,
                            size_t* found_label_count) {
  std::ifstream file(file_name);
  if (!file) {
    LOG(FATAL) << "Labels file " << file_name << " not found\n";
    return kTfLiteError;
  }
  result->clear();
  string line;
  while (std::getline(file, line)) {
    result->push_back(line);
  }
  *found_label_count = result->size();
  const int padding = 16;
  while (result->size() % padding) {
    result->emplace_back();
  }
  return kTfLiteOk;
}

void ObjDetectPub::PrintProfilingInfo(const profiling::ProfileEvent* e, uint32_t op_index,
                        TfLiteRegistration registration) {
  // output something like
  // time (ms) , Node xxx, OpCode xxx, symblic name
  //      5.352, Node   5, OpCode   4, DEPTHWISE_CONV_2D

  LOG(INFO) << std::fixed << std::setw(10) << std::setprecision(3)
            << (e->end_timestamp_us - e->begin_timestamp_us) / 1000.0
            << ", Node " << std::setw(3) << std::setprecision(3) << op_index
            << ", OpCode " << std::setw(3) << std::setprecision(3)
            << registration.builtin_code << ", "
            << EnumNameBuiltinOperator(
                   static_cast<BuiltinOperator>(registration.builtin_code))
            << "\n";
}


} // namespace roller_eye
