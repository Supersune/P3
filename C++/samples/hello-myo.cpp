// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <array>
#include <iostream>
#include <fstream>//Writes to text file.
#include <sstream>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <queue> //Used for deque.
#include <chrono> //Here to make sleep work.
#include <thread> //Here to make sleep work.

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include <SerialPort.h>

std::array<std::deque<int>, 8> emgSamplesQue; //Used to store the values for the averaging.
std::array<float, 8> emgSamplesAvg; //used to store the average value of the emg samples.
float emgSamplesAvgOut; //used to store the average value of the emg output.
float emgSamplesAvgOut1; //used to store the average value of the emg output.
int samples = 0; //Used to know when there is enough samples to calculate the average.
int avgSamples = 50; //Sets the amount of samples used for averaging.
std::ofstream myfile;
char port[] = "\\\\.\\COM18"; // port name
SerialPort arduino(port);
char intStr[30];
//int int1;
//sprintf(intStr, "%d", int1);
char buffer[50];
std::string modeChangeGestureFor = "fist";
std::string modeChangeGestureBack = "fingersSpread";
char modeChangeChar = '3';
char output[MAX_DATA_LENGTH];
std::string data = "0";
float emgThreshold = 0.3;
float emgSensitivity = 25;//40

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : emgSamples(), onArm(false), isUnlocked(false), currentPose()
    {
    }

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
		emgSamples.fill(0);
        onArm = false;
        isUnlocked = false;
    }

   
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

		if (currentPose.toString() == modeChangeGestureFor)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			data = "3";
			char c_string[MAX_DATA_LENGTH];

			std::copy(data.begin(), data.end(), c_string);

			arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);

			std::this_thread::sleep_for(std::chrono::milliseconds(30));

			std::cout << std::flush;
		}

		if (currentPose.toString() == modeChangeGestureBack)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			data = "4";
			char c_string[MAX_DATA_LENGTH];

			std::copy(data.begin(), data.end(), c_string);

			arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);
			std::this_thread::sleep_for(std::chrono::milliseconds(30));


			std::cout << std::flush;
		}

        if (true/*pose != myo::Pose::unknown && pose != myo::Pose::rest*/) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);

            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        } 

		else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }
    }

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{
		for (int i = 0; i < 8; i++) {
			emgSamples[i] = emg[i];
		}


	}
	

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

		for (size_t i = 0; i < emgSamples.size(); i++) {
			std::ostringstream oss;
			oss << static_cast<int>(emgSamples[i]);
			std::string emgString = oss.str();

			std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
		}

        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            
			
			std::string poseString = currentPose.toString();


            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                      << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
			
			if (poseString == "waveIn") {
				std::cout << "Going down!";
			
			}
			else if (poseString == "waveOut")
			{
				std::cout << "Going up!  ";
			}
			else
			{
				std::cout << "           ";
			}

		
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }
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
		emgSamplesAvgOut1 = (emgSamplesAvg[2] + emgSamplesAvg[3] + emgSamplesAvg[4]) / 3 - (emgSamplesAvg[0] + emgSamplesAvg[6] + emgSamplesAvg[7]) / 3;

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

		if (emgSamplesAvgOut < -1)
		{
			emgSamplesAvgOut = -1;
		}


		//for (int i = 0; i < 8; i++) //Prints the average of the emg samples.
		//{
		//	myfile << emgSamplesAvg[i] << " ";
		//}
		//myfile << emgSamplesAvgOut << ";" << emgSamplesAvgOut1 << std::endl;
		myfile << emgSamplesAvgOut << std::endl;
		sprintf(buffer, "%f", emgSamplesAvgOut);
		data = buffer;
		//arduino.writeSerialPort("1", MAX_DATA_LENGTH);
		//arduino.readSerialPort(output, MAX_DATA_LENGTH);
		//std::cout << output << std::endl;
		//myfile << "\n\r";
		char c_string[MAX_DATA_LENGTH];

		std::copy(data.begin(), data.end(), c_string);

		arduino.writeSerialPort(c_string, MAX_DATA_LENGTH);



        std::cout << std::flush;
	
    }
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;

    // These values are set by onEmgData and onPose() above.
	std::array<int8_t, 8> emgSamples;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
	
	if (arduino.isConnected()) {
		std::cout << "Connection made" << std::endl << std::endl;
	}
	else {
		std::cout << "Error in port name" << std::endl << std::endl;
	}
	myfile.open("Trajectory.src");
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("com.example.hello-myo");

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    myo::Myo* myo = hub.waitForMyo(10000);

    // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

	// Next we enable EMG streaming on the found Myo.
	myo->setStreamEmg(myo::Myo::streamEmgEnabled);

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/180);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
        collector.print();
    }

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
