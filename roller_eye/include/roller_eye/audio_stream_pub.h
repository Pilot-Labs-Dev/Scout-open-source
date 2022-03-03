#ifndef __AUDIO_STREAM_PUB_H__
#define __AUDIO_STREAM_PUB_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <pulse/simple.h>
//#include <pulse/error.h>
#include <ros/ros.h>

#include "roller_eye/encoder_aac.h"
#include "roller_eye/frame.h"
#include "roller_eye/publisher.h"

namespace roller_eye {

#if 0
#define A_STREAM_VERBOSE(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define A_STREAM_VERBOSE(fmt, ...)  (void)0
#endif

#define SAMPLE_RATE	22050 //16000
#define AUDIO_CHANNELS	1
#define SAMPLES_PRE_FRAME	1024
#define BYTES_PRE_FRAME		(SAMPLES_PRE_FRAME * 2 * AUDIO_CHANNELS)
#define MAX_SIZE			(50)


class AudioStreamPub
{
public:
    AudioStreamPub(ros::NodeHandle& nh);
	~AudioStreamPub();

	void resetReader();
	bool read(char *buf);
	void write(char *buf);

    void subConnectAAC(int cnt);
    void subDisconnectAAC(int cnt);
	void startPublish();
	void stopPublish();
	int getNumSubscribers();


private:
	void startCapture();
	void stopCapture();
	void flush();
	void captureLoop();
	void waitData();

	void publishLoop();
    void publishAAC(Frame *tmpFrame,  const Frame* ptrEncodedFrame);

	pa_simple *pa_record;
	std::array<char *, MAX_SIZE> m_data;
	std::mutex m_paMutex;

	unsigned int m_in;
	unsigned int m_out;

    std::mutex m_dataMutex;
    std::condition_variable m_dataTrigger;

    int subRefNum;
	std::atomic<bool> m_capRunning;
	std::atomic<bool> m_pubRunning;
	std::thread m_capThread;
	std::thread m_pubThread;

	std::shared_ptr<Encoder> mEncoder;
    Publisher<roller_eye::frame> mPub;
};

} // namespace roller_eye

#endif // __AUDIO_STREAM_PUB_H__

