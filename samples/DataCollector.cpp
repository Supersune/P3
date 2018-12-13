#include "DataCollector.h"
#include <chrono>
#include <thread>
#include <fstream>//Writes to text file.
std::ofstream myfile;
int test = 0;
int posCap = 0;
DataCollector::DataCollector() : emgSamples(), onArm(false), isUnlocked(false), currentPose()
{
	
}

DataCollector::~DataCollector()
{
}
// These values are set by onArmSync() and onArmUnsync() above.
bool onArm;
myo::Arm whichArm;

// This is set by onUnlocked() and onLocked() above.
bool isUnlocked;

// These values are set by onEmgData and onPose() above.
std::array<int8_t, 8> emgSamples;
myo::Pose currentPose;

// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
void DataCollector::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
	// We've lost a Myo.
	// Let's clean up some leftover state.
	emgSamples.fill(0);
	onArm = false;
	isUnlocked = false;
}


// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
// making a fist, or not making a fist anymore.
void DataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
	currentPose = pose;

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
void DataCollector::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
	myo::WarmupState warmupState)
{
	onArm = true;
	whichArm = arm;
}

// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
// when Myo is moved around on the arm.
void DataCollector::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
	onArm = false;
}

// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
void DataCollector::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
	isUnlocked = true;
}

// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
void DataCollector::onLock(myo::Myo* myo, uint64_t timestamp)
{
	isUnlocked = false;
}

void DataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	//if (test == 0)
	//{
	//	myfile.open("Trajectory.csv");
	//}
	//test++;
	for (int i = 0; i < 8; i++) {
		emgSamples[i] = emg[i];
	}
		myfile << test << "," << (int)emg[0] << "," << (int)emg[1] << "," << (int)emg[2] << "," << (int)emg[3] << "," << (int)emg[4] << "," << (int)emg[5] << "," << (int)emg[6] << "," << (int)emg[7] << std::endl;
		myfile << "\n" << std::endl;
		processing.processing(emgSamples, "");
	
}


// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
// For this example, the functions overridden above are sufficient.

// We define this function to print the current values that were updated by the on...() functions above.
int DataCollector::print()
{
	//std::string poseString = currentPose.toString();
	// Clear the current line
	//std::cout << '\r';

	for (size_t i = 0; i < emgSamples.size(); i++) {
		//std::ostringstream oss;
		//oss << static_cast<int>(emgSamples[i]);
		//std::string emgString = oss.str();

		//std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
	}
	/*std::string boollocked ="";
	if (isUnlocked == 0) {
		boollocked = "locked";
	}
	else if (isUnlocked == 1) {
		boollocked = "locked";
	}
	std::cout << "What is Bool unlocked?: " << boollocked << "<<<<" << std::endl;*/
	if (onArm) {

		// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

		// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
		// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
		// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().


	//	std::string posestring = "not unknown";
	//	//std::cout << posestring << std::endl;
	//	posestring = currentpose.tostring();
	//	std::cout << "what is current pose: " << posestring << " ... ";
	//	if (poscap == 0) { std::cout << "position: joint 1."; }

	//	else if (poscap == 1) { std::cout << "position: height."; }
	//	else if (poscap == 2) { std::cout << "position: stretch."; }
	//	else if (poscap == 3) { std::cout << "position: gripper."; }

	//	/*
	//	std::cout << '[' << (isunlocked ? "unlocked" : "locked  ") << ']'
	//		<< '[' << (whicharm == myo::armleft ? "l" : "r") << ']'
	//		<< '[' << posestring << std::string(14 - posestring.size(), ' ') << ']';

	//		*/
	//	if (posestring == "wavein") {

	//		return(2);

	//	}
	//	else if (posestring == "waveout")
	//	{
	//		return(1);

	//	}
	//	else if (posestring == "fist") {
	//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//		poscap++;
	//		if (poscap = 4) { poscap = 0; }
	//		return(3);
	//	}
	//	//else if (posestring == "doubletap") {
	//		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//		//return(3);
	//	//}
	//	else
	//	{
	//		return(0);

	//	}
		

	}
	else {
		// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.

		//std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
	}

	std::cout << std::flush;
	return(0);
}

