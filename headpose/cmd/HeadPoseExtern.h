#pragma once

#include<iostream>
#include <thread>

using namespace std;
 
static HeadPose &instance(){
		static HeadPose headPose;
		return headPose;
};

extern "C"{
    
    thread RunPupilAsync();

	void RunPupil();
	void RunEyePupil();
	float SendX();
	float SendY();
}  
