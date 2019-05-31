// #ifndef V9_GOALPOST_GLOBAL_H
// #define V9_GOALPOST_GLOBAL_H
#pragma once

#define DEBUG
#define PROCESS
#define KALIBRASI

#include<opencv2/opencv.hpp>
#include<ros/ros.h>
#include<ros/package.h>
#include<geometry_msgs/Point.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<image_transport/image_transport.h>

using namespace std;
using namespace cv;

static void onMouse(int event, int x, int y, int, void*);
void treshSob(int, void*);


class GoalPostDetector{
private:
   ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber get_frame_camera;
	
    void GetFrameCamCallback(const sensor_msgs::ImageConstPtr& msg);
	
	cv_bridge::CvImagePtr cvImgPtr;
	unsigned int img_encoding_;
	geometry_msgs::Point goal_pos;
	ros::Publisher goal_pos_pub;

	Mat canvas;
	Mat frame;
	Mat segmentedGreen;
	Mat segmentedGreenWhite;
	Mat imYX, imYY;

	std::string path_yaml;
	int R, G, B;

	bool goal_detected = false;


protected:
	Mat bacaYAML(string namafile);
	void simpanTabel(Mat matrix, string namafile);
	void masukanMatrix(int nilai, Mat gambar, Rect kotak);
	Mat captureInCanvas(Mat _frame, int type_debug);

public:
   enum TABEL{
    TABEL_HIJAU = 1,
    TABEL_ORANGE = 2,
    TABEL_PUTIH = 3,
    TABEL_OTHER = 0
   };
   
	enum STATE{
	    STATE_HIJAU = 1,
	    STATE_PUTIH = 2,
	    STATE_OTHER = 0
	};

   enum ENCODING{
      IMG_MONO = 0,
      IMG_RGB = 1
    };

	enum TYPE_DEBUG{
		TYPE_HIJAU_BINER = 0,
		TYPE_HIJAU_PUTIH_BINER = 2
	};
   
	void process();

	GoalPostDetector();
	~GoalPostDetector();
};


enum QQQ{
	Q1 = 1,
	MEDIAN = 2,
	Q3 = 3
};
void SWWAP(int* a, int* b, LineIterator* vla, LineIterator* vlb, vector<float> dfa, vector<float> dfb);
int partition (vector<float>& V, vector<LineIterator>& VL, vector<vector<float> >& DF, int low, int high);
void quickSort(vector<float>& V, vector<LineIterator>& VL, vector<vector<float> >& DF, int low, int high);
void calcQuartil(vector<float> V, float& quartil, int QQQ);

//#endif
