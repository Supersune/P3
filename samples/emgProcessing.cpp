#include "emgProcessing.h"
#include <fstream>//Writes to text file.
std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;
std::ofstream myfile4;
int test2 = 0;


emgProcessing::emgProcessing()
{
	if (arduino.isConnected()) {
		std::cout << "connection made" << std::endl << std::endl;
	}
	else {
		std::cout << "error in port name" << std::endl << std::endl;
	}
	
}


emgProcessing::~emgProcessing()
{
}

float emgProcessing::processing(std::array<int, 8> emgSamples, std::string currentPose)
{
	
	if (currentPose == modeChangeGestureFor)
	{
		data = modeChangeCharFor;
		char c_string [MAX_DATA_LENGTH];

		std::copy(data.begin(), data.end(), c_string);

		arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);

		std::cout << std::flush;


	}
	else if (currentPose == modeChangeGestureBack)
	{
		data = modeChangeCharBack;
		char c_string [MAX_DATA_LENGTH];

		std::copy(data.begin(), data.end(), c_string);

		arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);

		std::cout << std::flush;
	}
	else
	{

		for (int i = 0; i < 8; i++)//puts the absulute value of the emg data in to a queue that stores the newest data first.
		{
			emgSamplesQue[i].push_back(abs((int)emgSamples[i]));
		}
		


		if (samples == avgSamples + 1) //Executes if we have enough emg samples to calculate the average.
		{
			for (int i = 0; i < 8; i++) //removes the oldest element.
			{
				emgSamplesQue[i].pop_front();
			}
			for (int j = 0; j < avgSamples; j++) //Add all of the emg samples together.
			{
				for (int i = 0; i < 8; i++)
				{
					emgSamplesAvg[i] += emgSamplesQue[i].at(j);
				}
			}
			for (int i = 0; i < 8; i++) //Divides the added emg samples and divides it with the amout of added emg samples to get the average.
			{
				emgSamplesAvg[i] = emgSamplesAvg[i] / avgSamples;
			}
			/*if (test2 == 0)
			{
				myfile2.open("Trajectory2.csv");
			}
			myfile2 << test2 << "," << (int)emgSamplesAvg[0] << "," << (int)emgSamplesAvg[1] << "," << (int)emgSamplesAvg[2] << "," << (int)emgSamplesAvg[3] << "," << (int)emgSamplesAvg[4] << "," << (int)emgSamplesAvg[5] << "," << (int)emgSamplesAvg[6] << "," << (int)emgSamplesAvg[7] << std::endl;*/
		}
		else //Increase the samples counter and set the emgSamplesAvg = 0 until we can calculate the average.
		{
			samples++;
			for (int i = 0; i < 8; i++)
			{
				emgSamplesAvg[i] = 0;
			}
		}
		
		emgSamplesAvgOut = ((emgSamplesAvg[2] + emgSamplesAvg[3] + emgSamplesAvg[4]) / 3 - (emgSamplesAvg[0] + emgSamplesAvg[6] + emgSamplesAvg[7]) / 3) / emgSensitivity;
		
		if (test2 == 0)
		{
			myfile1.open("Trajectory1.csv");
		}
		myfile1 << test2 << "," << (abs((int)emgSamples[0])) << "," << (abs((int)emgSamples[1])) << "," << (abs((int)emgSamples[2])) << "," << (abs((int)emgSamples[3])) << "," << (abs((int)emgSamples[4])) << "," << (abs((int)emgSamples[5])) << "," << (abs((int)emgSamples[6])) << "," << (abs((int)emgSamples[7])) << std::endl;
		if (test2 == 0)
			{
				myfile2.open("Trajectory2.csv");
			}
			myfile2 << test2 << "," << (float)emgSamplesAvg[0] << "," << (float)emgSamplesAvg[1] << "," << (float)emgSamplesAvg[2] << "," << (float)emgSamplesAvg[3] << "," << (float)emgSamplesAvg[4] << "," << (float)emgSamplesAvg[5] << "," << (float)emgSamplesAvg[6] << "," << (float)emgSamplesAvg[7] << std::endl;
		if (test2 == 0)
		{
			myfile3.open("Trajectory3.csv");
		}
		myfile3 << test2 << "," << ((emgSamplesAvg[2] + emgSamplesAvg[3] + emgSamplesAvg[4]) / 3) << "," << ((emgSamplesAvg[0] + emgSamplesAvg[6] + emgSamplesAvg[7]) / 3)  << std::endl;


		if (test2 == 0)
		{
			myfile4.open("Trajectory4.csv");
		}
		myfile4 << test2 << "," << (((emgSamplesAvg[2] + emgSamplesAvg[3] + emgSamplesAvg[4]) / 3 - (emgSamplesAvg[0] + emgSamplesAvg[6] + emgSamplesAvg[7]) / 3) / emgSensitivity) << std::endl;
		test2++;
		
		if (emgSamplesAvgOut < emgThreshold & emgSamplesAvgOut > -emgThreshold)
		{
			emgSamplesAvgOut = 0;

		}
		else
		{
			if (emgSamplesAvgOut > emgThreshold)
			{
				emgSamplesAvgOut = emgSamplesAvgOut - emgThreshold;
			}
			if (emgSamplesAvgOut < -emgThreshold)
			{
				emgSamplesAvgOut = emgSamplesAvgOut + emgThreshold;
			}
		}

		if (emgSamplesAvgOut > 1)
		{
			emgSamplesAvgOut = 1;
		}

		else if (emgSamplesAvgOut < -1)
		{
			emgSamplesAvgOut = -1;
		}

		sprintf(buffer, "%f", emgSamplesAvgOut);
		data = buffer;
		//std::cout << buffer << std::endl;
		std::cout << emgSamplesQue[5].size() << std::endl;
		char c_string [MAX_DATA_LENGTH];

		std::copy(data.begin(), data.end(), c_string);

		arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);
		
	}
	return 0.0f;
}
