#include "HeadPose.h"
#include "HeadPoseExtern.h"

#include<iostream>
#include <thread>


using namespace std; 

extern "C"{

static thread headPose;

	void RunPupil(){
		headPose = RunPupilAsync();
	}

    void RunEyePupil(){ 
		 instance().open();
	}

	thread RunPupilAsync(){
		thread theThreadPupil(RunEyePupil);
		return theThreadPupil;
	}

	float SendX(){
		return instance().getX();
	}
	
	float SendY(){
		return instance().getY();
	}
} 