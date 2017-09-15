#include <iostream>

#include <opencv2/highgui/highgui.hpp>


//*** dlib libraries
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/threads.h>
#include <dlib/misc_api.h>

//*** opencv libraries
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string>
#include <math.h>
#include <cmath>
#include <time.h>


#include "HeadPose.h"
#include "HeadPoseExtern.h"

using namespace dlib;
using namespace std;

const static cv::Point3f P3D_SELLION(0., 0.,0.);
const static cv::Point3f P3D_RIGHT_EYE(-20., -65.5,-5.);
const static cv::Point3f P3D_LEFT_EYE(-20., 65.5,-5.);
const static cv::Point3f P3D_RIGHT_EAR(-100., -77.5,-6.);
const static cv::Point3f P3D_LEFT_EAR(-100., 77.5,-6.);
const static cv::Point3f P3D_NOSE(21.0, 0., -48.0);
const static cv::Point3f P3D_STOMMION(10.0, 0., -75.0);
const static cv::Point3f P3D_MENTON(0., 0.,-133.0);

enum 
{
     NOSE=30,
     RIGHT_EYE=36,
     LEFT_EYE=45,
     RIGHT_SIDE=0,
     LEFT_SIDE=16,
     EYEBROW_RIGHT=21,
     EYEBROW_LEFT=22,
     MOUTH_UP=51,
     MOUTH_DOWN=57,
     MOUTH_RIGHT=48,
     MOUTH_LEFT=54,
     SELLION=27,
     MOUTH_CENTER_TOP=62,
     MOUTH_CENTER_BOTTOM=66,
     MENTON=8
};

string filename = "/home/mydidimo/Desktop/headpose/lib/camera.yml";

int HeadPose::open(){
 
	cv::VideoCapture cap;
    cap.open(0);

     cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    // read camera matrix and distortion coefficients from file
    cv::Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distortion;
    // close the input file

    std::vector<cv::Point3f> head_points;
    cv::Mat rvec = cv::Mat(cv::Size(3,1), CV_64F);
    CvMat r;
    cv::Mat tvec = cv::Mat(cv::Size(3,1), CV_64F);

    head_points.push_back(P3D_SELLION);
    head_points.push_back(P3D_RIGHT_EYE);
    head_points.push_back(P3D_LEFT_EYE);
    head_points.push_back(P3D_RIGHT_EAR);
    head_points.push_back(P3D_LEFT_EAR);
    head_points.push_back(P3D_MENTON);
    head_points.push_back(P3D_NOSE);
    head_points.push_back(P3D_STOMMION);

int n=0,m=0;

    try
    {

        cap.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
        cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );        
        //image_window win;
        cv::Mat left,right;
        // Load face detection and pose estimation models.
        frontal_face_detector detector = get_frontal_face_detector();
        shape_predictor pose_model;
        deserialize("/home/mydidimo/Desktop/headpose/lib/shape_predictor_68_face_landmarks.dat") >> pose_model;

    

        std::vector<cv::Point2f> imageFramePoints;
        std::vector<cv::Point3f> framePoints;

        framePoints.push_back(cv::Point3d(0.0,0.0,0.0));
        framePoints.push_back(cv::Point3d(50,0,0));
        framePoints.push_back(cv::Point3d(0,50,0));
        framePoints.push_back(cv::Point3d(0,0,50));

        // Grab and process frames until the main window is closed by the user.
     
        while(1)
        {
            // Grab a frame
            cv::Mat temp,gray,dst;
            cap >> dst;

            //flipping image (taking mirror image)
            temp = cv::Mat(dst.rows, dst.cols, CV_8UC3);
            cv::flip(dst, temp, 1);
            cvtColor( temp, gray, CV_BGR2GRAY );
            cv_image<bgr_pixel> cimg(temp);

            // Detect faces 
            std::vector<rectangle> faces = detector(cimg);
            // Find the pose of each face.
            std::vector<full_object_detection> shapes;
            for (unsigned long i = 0; i < faces.size(); ++i)
            {
                full_object_detection shape = pose_model(cimg, faces[i]);
                cv::Rect l(shape.part(36).x()-5,shape.part(36).y()-20,(shape.part(39).x()-shape.part(36).x())+10,40);             
                cv::Rect r(shape.part(42).x()-5,shape.part(42).y()-20,(shape.part(45).x()-shape.part(42).x())+10,40);    
        
                nosex=shape.part(NOSE).x();nosey=shape.part(NOSE).y();
                right_eyex=shape.part(RIGHT_EYE).x(); right_eyey=shape.part(RIGHT_EYE).y();
                left_eyex=shape.part(LEFT_EYE).x(); left_eyey=shape.part(LEFT_EYE).y();
                right_sidex=shape.part(RIGHT_SIDE).x(); right_sidey=shape.part(RIGHT_SIDE).y();
                left_sidex=shape.part(LEFT_SIDE).x(); left_sidey=shape.part(LEFT_SIDE).y();
                mouth_rightx=shape.part(MOUTH_RIGHT).x(); mouth_righty=shape.part(MOUTH_RIGHT).y();
                mouth_leftx=shape.part(MOUTH_LEFT).x(); mouth_lefty=shape.part(MOUTH_LEFT).y();  
                mouth_downx=shape.part(MOUTH_DOWN).x(); mouth_downy=shape.part(MOUTH_DOWN).y();
                mouth_upx=shape.part(MOUTH_UP).x(); mouth_upy=shape.part(MOUTH_UP).y();
                sellionx=shape.part(SELLION).x(); selliony=shape.part(SELLION).y();
                eyebrow_rightx=shape.part(EYEBROW_RIGHT).x(); eyebrow_righty=shape.part(EYEBROW_RIGHT).y();
                eyebrow_leftx=shape.part(EYEBROW_LEFT).x(); eyebrow_lefty=shape.part(EYEBROW_LEFT).y();
                mouth_center_upx=shape.part(MOUTH_CENTER_TOP).x(); mouth_center_upy=shape.part(MOUTH_CENTER_TOP).y();
                mouth_center_downx=shape.part(MOUTH_CENTER_BOTTOM).x(); mouth_center_downy=shape.part(MOUTH_CENTER_BOTTOM).y();
                mentonx=shape.part(MENTON).x(); mentony=shape.part(MENTON).y();
                stomionx=(mouth_center_upx+mouth_center_downx)*0.5;
                stomiony=(mouth_center_upx+mouth_center_downx)*0.5;

                check1yl=(shape.part(39).x() + shape.part(36).x())*05;
                check1yr=(shape.part(45).x() + shape.part(42).x())*0.5;


                double dl=(shape.part(39).x()-shape.part(36).x());
                double dr=(shape.part(45).x()-shape.part(42).x());
        
                std::vector<cv::Point2f> detected_points;
                detected_points.push_back(cv::Point(sellionx,selliony));
                detected_points.push_back(cv::Point(right_eyex,right_eyey));
                detected_points.push_back(cv::Point(left_eyex,left_eyey));
                detected_points.push_back(cv::Point(right_sidex,right_sidey));
                detected_points.push_back(cv::Point(left_sidex,left_sidey));
                detected_points.push_back(cv::Point(mentonx,mentony));
                detected_points.push_back(cv::Point(nosex,nosey));
                detected_points.push_back(cv::Point(stomionx,stomiony));
    
                cv::solvePnP(cv::Mat(head_points),cv::Mat(detected_points),intrinsics, distortion, rvec, tvec, false);
                cv::projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );
        
                double theta = cv::norm(rvec);
                double rx=rvec.at<double>(0,0);
                double ry=rvec.at<double>(1,0);
                double rz=rvec.at<double>(2,0);
        
                line(temp, cv::Point((int)imageFramePoints[0].x,(int)imageFramePoints[0].y), cv::Point((int)imageFramePoints[1].x,(int)imageFramePoints[1].y), cv::Scalar(255,0,0),2,8 );
                line(temp, cv::Point((int)imageFramePoints[0].x,(int)imageFramePoints[0].y), cv::Point((int)imageFramePoints[2].x,(int)imageFramePoints[2].y), cv::Scalar(0,255,0),2,8 );
                line(temp, cv::Point((int)imageFramePoints[0].x,(int)imageFramePoints[0].y), cv::Point((int)imageFramePoints[3].x,(int)imageFramePoints[3].y), cv::Scalar(0,0,255),2,8 );

                 x = (float)imageFramePoints[1].x / 370;
                 y = (float)imageFramePoints[2].y / 370;

                 x = x * 90.0f;
                 y = y * 90.0f;

		 //*** data to the x axis
		 x = (x-75)*4;
	
		//*** data to the y axis		
		 y = (y-55)*4;

            }

        cv::imshow("image",temp);
	    cv::waitKey(1);
          
        }
    }
    catch(serialization_error& e)
    {
        cout << endl << e.what() << endl;
    }
    catch(exception& e)
    {
        cout << e.what() << endl;
    }
}

float HeadPose::getX(){
    printf ("x:%.2f\n", x);
    return x; 
}

float HeadPose::getY(){
    printf ("y:%.2f\n", y);
    return y;
}