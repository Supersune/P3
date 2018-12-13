//#ifndef DATACOLLECTOR_H5
//#define DATACOLLECTOR_H

#pragma once
#define _USE_MATH_DEFINES
#include <array>
#include <iostream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include "emgProcessing.h"

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>



class DataCollector : public myo::DeviceListener
{
public:
	DataCollector();
	~DataCollector();

	bool onArm;
	myo::Arm whichArm;
	bool isUnlocked;
	std::array<int, 8> emgSamples;
	myo::Pose currentPose;
	std::string poseString;

	emgProcessing processing;
	
	


	//public slots:

	void onUnpair(myo::Myo* myo, uint64_t timestamp);
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState);
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp);
	void onUnlock(myo::Myo* myo, uint64_t timestamp);
	void onLock(myo::Myo* myo, uint64_t timestamp);
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);
	int print();

private:

	//private slots:

};

