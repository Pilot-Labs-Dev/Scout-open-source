#include <unistd.h>
#include <cstring>

#include "roller_eye/audio_stream_pub.h"
#include "roller_eye/arecord.h"


namespace roller_eye {
#define TEST_SAVE_FILE 0

AudioStreamPub::AudioStreamPub(ros::NodeHandle& nh) :
subRefNum(0), m_capRunning(false), m_pubRunning(false),
mPub(nh, "aac", 100, bind(&AudioStreamPub::subConnectAAC,this,placeholders::_1), bind(&AudioStreamPub::subDisconnectAAC,this,placeholders::_1))
{
    for(auto &data: m_data) {
		data = new char[BYTES_PRE_FRAME];
	}
	m_in = 0;
	m_out = 0;

    mEncoder = make_shared<EncoderAAC>(SAMPLE_RATE, 16, AUDIO_CHANNELS);
}

AudioStreamPub::~AudioStreamPub()
{
	for(auto &data: m_data) {
		delete[] data;
    }
	mEncoder = nullptr;
}

void AudioStreamPub::resetReader()
{
	m_out = m_in;
}

bool AudioStreamPub::read(char *buf)
{
	waitData();
	if(!m_pubRunning){
		return false;
	}
	std::memcpy(buf, m_data[m_out % MAX_SIZE], BYTES_PRE_FRAME);
	m_out++;
	return true;
}

void AudioStreamPub::write(char *buf)
{
	std::memcpy(m_data[m_in % MAX_SIZE], buf, BYTES_PRE_FRAME);
	std::unique_lock<std::mutex> lock(m_dataMutex);
	m_in++;
	m_dataTrigger.notify_all();
}

void AudioStreamPub::waitData()
{
    std::unique_lock<std::mutex> lock(m_dataMutex);
    m_dataTrigger.wait(lock, [this]() {return (m_in != m_out || !m_pubRunning);});
}


void AudioStreamPub::flush()
{
	int error = 0;

	pa_simple_flush(pa_record, &error);
	m_out = m_in;
}

void AudioStreamPub::captureLoop()
{
	int error = 0;

	int ret = arecord_init(SND_PCM_FORMAT_S16_LE, AUDIO_CHANNELS, SAMPLE_RATE);
	if (ret !=0){
		ROS_DEBUG("arecord_init failed!\n");
		return;
	}
    u_char buf[BYTES_PRE_FRAME];
	ROS_DEBUG("aac capture running.\n");
	while(m_capRunning) {
		m_paMutex.lock();

		ret = arecord_read((u_char *)(m_data[m_in % MAX_SIZE]), BYTES_PRE_FRAME);

		if(ret < 0) {
			ROS_DEBUG("arecord_read() failed\n");
		} else {
			std::unique_lock<std::mutex> lock(m_dataMutex);
			m_in++;
			m_dataTrigger.notify_all();
		}
		m_paMutex.unlock();
	}
	arecord_uninit();
}

void AudioStreamPub::startCapture()
{
	m_capRunning = true;
	m_capThread = std::thread(&AudioStreamPub::captureLoop, this);
}

void AudioStreamPub::stopCapture()
{
	if(!m_capRunning) {
        return;
	}

	m_capRunning = false;
	m_capThread.join();
}

void AudioStreamPub::startPublish()
{
	m_pubRunning = true;
	m_pubThread = std::thread(&AudioStreamPub::publishLoop, this);
}

void AudioStreamPub::stopPublish()
{
	if(!m_pubRunning) {
        return;
	}
	m_dataMutex.lock();
	m_pubRunning = false;
	m_dataMutex.unlock();
	m_dataTrigger.notify_all();
	m_pubThread.join();
}

void AudioStreamPub::subConnectAAC(int cnt)
{
  	ROS_DEBUG("subConnectAAC subscriber cnt: %d", cnt);
  	subRefNum = cnt;
	if (subRefNum > 0) {
		if (m_pubRunning == false) {
			ROS_DEBUG("aac startCapture and startPublish.\n");
			this->startCapture();
			this->startPublish();
		} else {
			ROS_DEBUG("publishing stream already created.\n");
		}
	}
}
void AudioStreamPub::subDisconnectAAC(int cnt)
{
  	ROS_DEBUG("subDisconnectAAC subscriber cnt: %d", cnt);
  	subRefNum = cnt;
	if (subRefNum == 0) {
		if (m_pubRunning == true) {
			this->stopPublish();
			this->stopCapture();
		} else {
			ROS_DEBUG("publishing stream already stop.\n");
		}
	}
}

int AudioStreamPub::getNumSubscribers()
{
  return subRefNum;
}

void AudioStreamPub::publishAAC(Frame *tmpFrame, const Frame* ptrEncodedFrame)
{
	static uint32_t seq = 0;
	roller_eye::frame msgFrame;
	auto ptrAudioFrame = dynamic_cast<const AudioFrame*>(ptrEncodedFrame);
	if (ptrAudioFrame != nullptr) {
		msgFrame.seq = ++seq;
		msgFrame.session = 2000; //todo
		msgFrame.stamp = tmpFrame->getTimestamp();
		msgFrame.type = msgFrame.AUDIO_STREAM_AAC;
		//in audio  frame,par1 is sample rate,par2 is bitwidth,par3 is channels
		msgFrame.par1 = ptrAudioFrame->sampleRate;
		msgFrame.par2 = ptrAudioFrame->bitwidth;
		msgFrame.par3 = ptrAudioFrame->channels;
		msgFrame.data.clear();
		msgFrame.data.insert(msgFrame.data.end(), &ptrAudioFrame->getData()[0], &ptrAudioFrame->getData()[ptrAudioFrame->getSize()]);

		A_STREAM_VERBOSE("publishAAC: %u, stamp: %ld, datasize: %lu\n", msgFrame.seq, msgFrame.stamp, msgFrame.data.size());

		mPub.publish(msgFrame);
	}
}

void AudioStreamPub::publishLoop()
{
	uint8_t audiobuf[BYTES_PRE_FRAME];
  	Frame *tmpFrame = new Frame();
	struct timespec tpTime;
	uint64_t timeStamp = 0;

#if TEST_SAVE_FILE
	std::ofstream outfile("testAudio.pcm", std::ofstream::binary);
	if (!outfile.is_open()) {
		std::cout << "Create file failed:" << "testAudio.pcm" << std::endl;
	}
	std::ofstream aacfile("testAudio.aac", std::ofstream::binary);
	if (!aacfile.is_open()) {
		std::cout << "Create file failed:" << "testAudio.aac" << std::endl;
	}
#endif

	ROS_DEBUG("aac publish running.\n");
	this->resetReader();
	while (m_pubRunning) {
		if(!this->read((char *)audiobuf)){
			continue;
		}

		clock_gettime(CLOCK_MONOTONIC, &tpTime);
		timeStamp = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;
		tmpFrame->setData(audiobuf,BYTES_PRE_FRAME,timeStamp);

		mEncoder->put_frame(tmpFrame);
		publishAAC(tmpFrame, mEncoder->encode_frame());

	#if TEST_SAVE_FILE
		outfile.write((char *)audiobuf, sizeof(audiobuf));
		if (ptrEncodedFrame != nullptr)
			aacfile.write((char *)ptrEncodedFrame->data, ptrEncodedFrame->size);
	#endif
	}

	delete tmpFrame;
#if TEST_SAVE_FILE
	outfile.close();
	aacfile.close();
#endif
}


} // namespace roller_eye



