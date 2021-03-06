#include "pch.h" //Included as standard by visual studio.
#include <iostream> //Included as standard by visual studio.
#include <array> //Here to make the array work.
#include <chrono> //Here to make sleep work.
#include <thread> //Here to make sleep work.
#include <queue> //Used for deque.


int main()
{
	std::array<int8_t, 8> emgSamples; //Stores the value of the emulated output fom the myo armband.
	std::array<std::deque<int>, 8> emgSamplesQue; //Used to store the values for the averaging.
	std::array<int, 8> emgSamplesAvg; //used to store the average value of the emg samples.
	int samples=0; //Used to know when there is enough samples to calculate the average.
	int avgSamples = 10; //Sets the amount of samples used for averaging.
	
	//std::queue<int> emgSamplesAvg;
	
	while (true)//Makes this code run until the program is terminated.
	{
		for (int i = 0; i < 8; i++) //generates random values and stores them in emgSamples. This is here to emulate the emg output from the myo armband.
		{
			emgSamples[i] = rand() % 201 - 100;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Here to slow the amount of data gathered.

		for (int i = 0; i < 8; i++)//puts the absulute value of the emg data in to a queue that stores the newest data first.
		{
			emgSamplesQue[i].push_back(abs((int)emgSamples[i]));
		}
		
		if (samples==avgSamples+1) //Executes if we have enough emg samples to calculate the average.
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
		}
		else //Increase the samples counter and set the emgSamplesAvg = 0 until we can calculate the average.
		{
			samples++;
			for(int i=0; i < 8; i++)
			{
				emgSamplesAvg[i] = 0;
			}
		}

	for (int i = 0; i < 8; i++) //Prints the average of the emg samples.
	{
		std::cout << emgSamplesAvg[i] << " ";
	}
	std::cout << "\n\r";
	}
}
