
#pragma once

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class HeadPose{

	private:
		int xx;
		int _lx=0,_ly=0,_rx=0,_ry=0,rX,rY,lX,lY;
        int _nosex=0,_nosey=0,nosex,nosey;
        int _right_eyex=0,_right_eyey=0,right_eyex,right_eyey;
        int _left_eyex=0,_left_eyey=0,left_eyex,left_eyey;
        int _right_sidex=0,_right_sidey=0,right_sidex,right_sidey;
        int _left_sidex=0,_left_sidey=0,left_sidex,left_sidey;
        int _mouth_rightx=0,_mouth_righty=0,mouth_rightx,mouth_righty;
        int _mouth_leftx=0, _mouth_lefty=0,mouth_leftx, mouth_lefty;  
        int _mouth_downx=0,_mouth_downy=0,mouth_downx,mouth_downy;
        int _mouth_upx=0,_mouth_upy=0,mouth_upx,mouth_upy;
        int _sellionx=0,_selliony=0,sellionx,selliony;
        int _eyebrow_rightx=0, _eyebrow_righty=0,eyebrow_rightx, eyebrow_righty;
        int _eyebrow_leftx=0, _eyebrow_lefty=0,eyebrow_leftx, eyebrow_lefty;
        int _mouth_center_upx=0, _mouth_center_upy=0,mouth_center_upx, mouth_center_upy;
        int _mouth_center_downx=0,_mouth_center_downy=0,mouth_center_downx,mouth_center_downy;
        int _mentonx=0, _mentony=0,mentonx, mentony;

		float x; float y;
        float _stomionx=0,_stomiony=0,stomionx,stomiony;
        
		double check1yl,check1yr;

 	public:
		cv::Mat find_face( cv::Mat frame, int method, bool show ) ;
		int open();
		float getX();
		float getY();
		int trying();
}; 
