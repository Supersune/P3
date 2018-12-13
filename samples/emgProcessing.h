#pragma once
#include <array>
#include <queue> //Used for deque.
#include <iostream>
#include <myo/myo.hpp>
#include <SerialPort.h>
//#include <fstream>//Writes to text file.

class emgProcessing
{
public:
	emgProcessing();
	~emgProcessing();
	float emgProcessing::processing(std::array<int, 8> emgSamples, std::string currentPose);
private:
	std::array<std::deque<int>, 8> emgSamplesQue; //Used to store the values for the averaging.
	std::array<float, 8> emgSamplesAvg; //used to store the average value of the emg samples.
	float emgSamplesAvgOut; //used to store the average value of the emg output.
	float emgSamplesAvgOut1; //used to store the average value of the emg output.
	int samples = 0; //Used to know when there is enough samples to calculate the average.
	int avgSamples = 50; //Sets the amount of samples used for averaging.
	//std::ofstream myfile; //
	const char port[10] = "\\\\.\\COM18"; // port name
	SerialPort arduino = SerialPort((port));
	char intStr[30];
	//int int1;
	//sprintf(intStr, "%d", int1);
	char buffer[50];
	std::string modeChangeGestureFor = "fist";
	std::string modeChangeGestureBack = "fingersSpread";
	char modeChangeChar = '3';
	char output[MAX_DATA_LENGTH];
	std::string data = "0";
	float emgThreshold = 0.08;
	float emgSensitivity = 40;
	char modeChangeCharBack = '4';
	char modeChangeCharFor = '3';
};

