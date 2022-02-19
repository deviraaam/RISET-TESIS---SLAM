#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <stdbool.h>
#include <fstream>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "tes_vrep/Pose.h"

#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "icp.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define pi 3.14159265
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )


#define imgArr 10
//#define centerX 400
//#define centerY 400
#define centerX 100
#define centerY 200
//#define centerX 450
//#define centerY 450
#define scaleFactor 40
#define awal	1
#define incr	1
#define iter	200

using namespace cv;
using namespace std;

float x_now, y_now, t_now;
float x_last, y_last, t_last;
float enc_xL, enc_yL, enc_tL;
float enc_x, enc_y, enc_t;
float x_pos,y_pos,t_pos,ti_pos,t_proc;

float datLaser[3][2][2000];
float laserData[2000];
float angle_step;
unsigned int scan_size;
bool scanready=false,serialready=false;
float xtemp=0,ytemp=0,ttemp=0;
float ddx, ddy;
float x_coo=0, y_coo=0;

unsigned long int loop=0;
unsigned int cnt=0;
Mat img = Mat(670, 670, CV_8UC3, cv::Scalar(0));
Mat imgSave[imgArr];
std::vector<CvPoint2D32f> ref_points_gl, new_points_gl;
IplImage *image_base;

unsigned long int p=0;
float degx,degy,dx,dy;
float dr_x,dr_y;
int idx=-1;
int idx_l1,idx_l2,idx_l3;
//int idx= 0;
float deg_offset=0;

char rekam_txt[30];
ofstream file_kalmanxyt;
ofstream file_odom,file_laser,file_datain,file_odomkal;
fstream tst_text, tst_text_1, tst_text_2, lidar_source;

ofstream file_teta;
char file_name[100];


float R[4] = {1.f,0.f,0.f,1.f}, T[2] = {0.,0.};
Mat r = Mat(2,2,CV_32F,R);
Mat t = Mat(2,1,CV_32F,T);

fstream errorku;
fstream dataslam_x;
fstream dataslam_y;
fstream jumlah_idx;
fstream odom_drx;
fstream odom_dry;
//>>>Kalman Variable
int stateSize[2] = {4,2};
int measSize[2] = {2,2};
int contrSize = 0;
float x_pred,y_pred,t_pred;
float x_cor,y_cor,t_cor;
unsigned int type = CV_32F;

cv::KalmanFilter kfxy(stateSize[0], measSize[0], contrSize, type); // (4, 2, 0, CV_32F)
cv::KalmanFilter kft(stateSize[1], measSize[1], contrSize, type); // (2, 2, ,0, CV_32F)

cv::Mat statexy(stateSize[0], 1, type);  // [x, y, v_x, v_y] 	// (4, 1, CV_32F)
cv::Mat measxy(measSize[0], 1, type);    // [z_x, z_y]			// (2, 1, CV_32F)
cv::Mat statet(stateSize[1], 1, type);  // [t, v_t]				// (2, 1, CV_32F)
cv::Mat meast(measSize[1], 1, type);    // [z_t]				// (2, 1, CV_32F)
//cv::Mat procNoise(stateSize, 1, type)
// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
//<<<Kalman Variable



double ticks;
std::vector<CvPoint2D32f> map_coordinates;

float total_err_icp = 0;
int total_scan = 0;



struct varRobot{
	float Motor1;
	float Motor2;
	float Motor3;
	float Motor4;
	
	float posx;
	float posy;
	float post;

	float odom_x;
	float odom_y;
	float odom_t;

	float posz;
	float ddeg;

	float imux_dat;
	float imuy_dat;
	float imuz_dat;
	float imuzs_dat;

    float codox_dat;
	float codoy_dat;
	float codoz_dat;
	float cododeg_dat;

	float new_post;
	float post_g;

	float gyro_x, gyro_y, gyro_z;
	float new_gyro_z;
};
struct varRobot robot,friends;



double getDegree(int x,int y){
double deg;
	deg=atan2((double)x,(double)y)*57.2957795;

return deg;
}

char ts;
void  timeSamplingCallback(const ros::TimerEvent&)
{
   ts=1;
}


void poseCallback(const tes_vrep::Pose::ConstPtr& msg)
{
	x_pos = msg->x;		//coordinate x = position x
	y_pos = msg->y;		//coordinate y = position y
	t_pos = msg->t;		//orientation encoder
	ti_pos = msg->ti;	//orientation IMU
	//if(t_pos>300 && loop<500){t_pos-=360;cout<<"t>300\n";}
	//if(ti_pos>180 && loop<500){ti_pos-=360;cout<<"ti>300\n";}
	if(abs(t_pos-ti_pos)>300){
		if(t_pos>300){t_pos-=360;}
		else if(ti_pos>300){ti_pos-=360;}	
	}
}



void posXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	robot.posx = msg->data;
	
 // ROS_INFO("I heard: [%f]", robot.posx);
}
void posYcallback(const std_msgs::Float32::ConstPtr& msg)
{	
	robot.posy =  msg->data;
	
 //ROS_INFO("I heard: [%f]", robot.posy);
}
void posTcallback(const std_msgs::Float32::ConstPtr& msg)
{ 
	robot.post =  msg->data;
	robot.new_post = robot.post * 0.01745329252;
	//t_cor = robot.new_post;
	//t_pos = robot.new_post;
	//printf("post = %f\n", robot.post);

 // ROS_INFO("I heard: [%f]", robot.post);
}


void odoXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	robot.odom_x = msg->data;
	//printf("robot.odom_x = %f\n", robot.odom_x);

}
void odoYcallback(const std_msgs::Float32::ConstPtr& msg)
{	
	robot.odom_y =  msg->data;  
	//printf("robot.odom_y = %f\n", robot.odom_y);
	
}
void odoTcallback(const std_msgs::Float32::ConstPtr& msg)
{ 
	robot.odom_t =  msg->data;
	robot.odom_t = robot.odom_t * pi / 180;
}


void gyroXCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_x = msg->data;
}
void gyroYCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_y = msg->data;
}
void gyroZCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_z = msg->data;
	robot.new_gyro_z = (robot.gyro_z-90) * pi / 180; // (robot.gyro_z - degreerobot) *(pi/180)	
	//degreerobot = degree robot from vrep

	//printf("gyro_z = %f\n", robot.gyro_z);
}


void kirim(){   //============= from stm32 to serial ROS =====================

	enc_x = robot.odom_x;
	enc_y = robot.odom_y;
	enc_t = robot.odom_t;

	if(enc_t<0) enc_t = 360+enc_t;
	if(enc_t==360){enc_t=0;}

	x_now=x_last+(enc_x-enc_xL)*cos(-t_now/57.2957f)+(enc_y-enc_yL)*sin(-t_now/57.2957f);
	y_now=y_last+(enc_y-enc_yL)*cos(-t_now/57.2957f)-(enc_x-enc_xL)*sin(-t_now/57.2957f);
	t_now=t_last+(enc_t-enc_tL);
	
	if(t_now>=360){t_now-=360;}
	else if(t_now<0){t_now+=360;}
	
	enc_xL = enc_x;
	enc_yL = enc_y;
	enc_tL = enc_t;
	x_last = x_now;y_last = y_now;t_last = t_now;	
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    double angle, distance, degree;

    int count = scan->scan_time / scan->time_increment;
    scan_size = scan->ranges.size();
    printf("scan_size = %d",scan_size);

	for(size_t i = 0; i < scan->ranges.size(); i++){
        angle = RAD2DEG(scan->angle_min + i*scan->angle_increment);
        degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        laserData[i] = scan->ranges[i];
		
		//printf("%0.3f    ", laserData[i]);
	}
    
    scanready = true;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "slam");

	ros::NodeHandle n;
	float timer=0.001;//10 ms
    ros::Timer timer1 = n.createTimer(ros::Duration(timer), timeSamplingCallback);

	//ros::Subscriber pose_sub = n.subscribe<guiderobotROS::Pose>("/poseMaster", 1000, poseCallback);
	//ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scanslam", 1000, scanslamCallback);
	
	

    cetak_icp(); //gak dipake?
	cetak_kdtree();
	ros::Subscriber posx = n.subscribe<std_msgs::Float32>("/posx",200,posXcallback);
	ros::Subscriber posy = n.subscribe<std_msgs::Float32>("/posy",200,posYcallback);
	ros::Subscriber post = n.subscribe<std_msgs::Float32>("/post",200,posTcallback);

	ros::Subscriber odomx = n.subscribe<std_msgs::Float32>("/odom_x",200,odoXcallback);
	ros::Subscriber odomy = n.subscribe<std_msgs::Float32>("/odom_y",200,odoYcallback);
	ros::Subscriber odomt = n.subscribe<std_msgs::Float32>("/odom_t",200,odoTcallback);
    
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/lasserDat", 2000, laserCallback);
	
	//ros::Subscriber gyro_x_sub = n.subscribe<std_msgs::Float32>("/gyro_x_dat", 100, gyroXCallback);
	//ros::Subscriber gyro_y_sub = n.subscribe<std_msgs::Float32>("/gyro_y_dat", 100, gyroYCallback);
	ros::Subscriber gyro_z_sub = n.subscribe<std_msgs::Float32>("/gyro_z_dat", 100, gyroZCallback);

    ros::Duration rate_0(20);
    rate_0.sleep();
	

    ros::Rate rate_1(100);
    
    scanready = false;
	for(int i=0;i<imgArr;i++){
	    imgSave[i]=Mat(670, 670, CV_8UC3, Scalar(0));
	}
	float rangee = 10;
	float lower = -1*(rangee/2);
	float fMin_x = 0.01;
	float fMax_x = 0.9;
	float fMin_y = 0.01;
	float fMax_y = 0.6;

    while(ros::ok())
	{

		srand(time(0));
		//float randnum = ((float)rand() / ((float)RAND_MAX + 1) * rangee)+lower;
		//double randnum = double(rand() / ((double)RAND_MAX) + 1.0) ;
		
		//double randnum_x = fMin_x + ((double)rand() / (double)RAND_MAX) * (fMax_x - fMin_x); 
		//double randnum_y = fMin_y + ((double)rand() / (double)RAND_MAX) * (fMax_y - fMin_y);
		//double randnum_t = 0.0087 + ((double)rand() / (double)RAND_MAX) * (0.35 - 0.0087);
		
		
		if(scanready && (xtemp!=robot.odom_x && ytemp!=robot.odom_y)){
		//if(scanready){
			loop++;
		}

		if(loop %8 == 0){
			//robot.new_post += randnum;
			
			//robot.odom_x += randnum_x;
			//robot.odom_y += randnum_y;
			
			//robot.gyro_z += randnum_t;
			//robot.new_gyro_z = robot.gyro_z * pi / 180;

			//printf("randnum_x = %f\n", randnum_x);
			//printf("randnum_y = %f\n", randnum_y);
			//printf("randnum_t = %f\n", randnum_t);
		}

        if(loop> 0 && scanready){
			
			float angle_step;
			angle_step = (float)270 / (float)scan_size;
			img = 0;
			imgSave[3] = 0;

			//sprintf(file_name, "/home/surya/catkin_ws/src/tes_vrep/test_text.txt");
			tst_text.open("/home/devira/catkin_ws5/src/tes_vrep/tes/test_text.txt", ios::in | ios::out | ios::trunc);
			tst_text_1.open("/home/devira/catkin_ws5/src/tes_vrep/tes/test_text1.txt", ios::in | ios::out | ios::trunc);
			tst_text_2.open("/home/devira/catkin_ws5/src/tes_vrep/tes/test_text2.txt", ios::in | ios::out | ios::trunc);

			for(int i = 0; i <= scan_size; i++){
				degx = cos(((angle_step*(i))-45) * M_PI / 180); //rad
				degy = sin(((angle_step*(i))-45) * M_PI / 180);

				dx = (laserData[i] * scaleFactor)*degx;	//rad * distance	//(0.5*distance) * cos(((angle_step*(i))-45) * M_PI / 180)
				dy = (laserData[i] * scaleFactor)*degy;

				//ddx = (cos(robot.new_post) * (dx - 0)) - (sin(robot.new_post) * (dy - 0)) + (centerX + (robot.odom_x * scaleFactor));
				//ddy = (sin(robot.new_post) * (dx - 0)) + (cos(robot.new_post) * (dy - 0)) - (centerY - (robot.odom_y * scaleFactor));
				//ddy = -ddy;

				x_coo=0;
				y_coo=0;

				if(dx!=0 || dy!=0){
					//x_coo = (cos(robot.new_post) * (dx - 0)) - (sin(robot.new_post) * (dy - 0)) + (centerX + (robot.odom_x * scaleFactor));
					//y_coo = (sin(robot.new_post) * (dx - 0)) + (cos(robot.new_post) * (dy - 0)) - (centerY - (robot.odom_y * scaleFactor));
					
					/////   Rotation then Translation
					//x_coo = (cos(robot.new_post) * (dx - 0)) - (sin(robot.new_post) * (dy - 0)) + 0;
					//y_coo = (sin(robot.new_post) * (dx - 0)) + (cos(robot.new_post) * (dy - 0)) + 0;
					//x_coo += robot.odom_x*scaleFactor;
					//y_coo -= robot.odom_y*scaleFactor;

					//x_coo = (cos(robot.odom_t) * (dx - 0)) - (sin(robot.odom_t) * (dy - 0)) + (centerX - (robot.odom_x * scaleFactor));
					//y_coo = (sin(robot.odom_t) * (dx - 0)) + (cos(robot.odom_t) * (dy - 0)) - (centerY + (robot.odom_y * scaleFactor));

					x_coo = (cos(robot.new_gyro_z) * (dx - 0)) - (sin(robot.new_gyro_z) * (dy - 0)) + (centerX + (robot.odom_x * scaleFactor));
					y_coo = (sin(robot.new_gyro_z) * (dx - 0)) + (cos(robot.new_gyro_z) * (dy - 0)) - (centerY - (robot.odom_y * scaleFactor));


					y_coo = -y_coo;
				}
				else{
					x_coo = 0;
					y_coo = 0;
				}
				
				if(x_coo != 0 && y_coo!= 0){
					++idx;
					if(loop<=2){
						if(loop==1){
							datLaser[0][0][idx] = x_coo;
							datLaser[0][1][idx] = y_coo;
							idx_l1 = idx;
						}

						else if(loop==2){
							datLaser[1][0][idx] = x_coo;
							datLaser[1][1][idx] = y_coo;
							idx_l2 = idx;
						}
					}
					//++idx;
					
					else{
						datLaser[0][0][idx] = datLaser[2][0][idx];
						datLaser[0][1][idx] = datLaser[2][1][idx];
						datLaser[1][0][idx] = x_coo;
						datLaser[1][1][idx] = y_coo;
						idx_l3 = idx;
					}
					
				}
				tst_text << "x = " << x_coo << ", y = " << y_coo << endl;
				
				//tst_text << "x = " << datLaser[0][0][idx] << ", y = " << datLaser[0][1][idx] << endl;
				//tst_text_1 << "x = " << datLaser[1][0][idx] << ", y = " << datLaser[1][1][idx] << endl;

				
				
				//cv::circle(img, cv::Point(x_coo, y_coo), 1, cv::Scalar(0,50,200), -1);
				//printf("dx = %.3f, dy = %.3f \t", dx, dy);
				//dr_x = centerX + (robot.odom_x * scaleFactor);
				//dr_y = centerY - (robot.odom_y * scaleFactor);
				//cv::circle(img, cv::Point(dr_x, dr_y), 1, cv::Scalar(0,250,0), -1);
				

			}
			
			//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
			//cv::imshow("Display window", img);
			//char s = cv::waitKey(1);
        
		}
		
		
		if(loop > 1 && scanready){
			if(xtemp!=robot.odom_x && ytemp!=robot.odom_y){
				printf("loop = %d\n", loop);
				
				std::vector<CvPoint2D32f> ref_points;
				std::vector<CvPoint2D32f> new_points;
				std::vector<CvPoint2D32f> odom_new, odom_last;
				//CvPoint2D32f *ref_points_po = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f)*idx_l1 );
				//CvPoint2D32f *new_points_po = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f)*idx_l1 );

				if(loop == 2){
					for(int g = 0; g <= idx_l1; g++){
					//for(int g = 0; g <= 500; g++){
						ref_points.push_back(cvPoint2D32f(datLaser[0][0][g], datLaser[0][1][g]));
						odom_last.push_back(cvPoint2D32f(datLaser[0][0][g], datLaser[0][1][g]));
						
					}
					for(int g = 0; g <= idx_l2; g++){
					//for(int g = 0; g <= 500; g++){
						new_points.push_back(cvPoint2D32f(datLaser[1][0][g], datLaser[1][1][g]));
						odom_new.push_back(cvPoint2D32f(datLaser[1][0][g], datLaser[1][1][g]));
											
					}
					
				}
				else if(loop>2){
					for(int j = 0; j <= idx_l3; j++){
						ref_points.push_back(cvPoint2D32f(datLaser[0][0][j], datLaser[0][1][j]));
						new_points.push_back(cvPoint2D32f(datLaser[1][0][j], datLaser[1][1][j]));
						odom_last.push_back(cvPoint2D32f(datLaser[0][0][j], datLaser[0][1][j]));
						odom_new.push_back(cvPoint2D32f(datLaser[1][0][j], datLaser[1][1][j]));
						//float xx = new_points[j].x;
						//float yy = new_points[j].y;
						
						//printf("x = %.3f , y = %.3f \t", xx, yy);
					}
				}

				//float R[4] = {1.f,0.f,0.f,1.f}, T[2] = {0.,0.};
				//Mat r = Mat(2,2,CV_32F,R);
				//Mat t = Mat(2,1,CV_32F,T);
//
				///printf("R[] bfr = %f, %f, %f, %f\n",R[0],R[1],R[2],R[3]);
				//printf("T[] bfr = %f, %f\n", T[0],T[1]);

				//float odom_err = err_odom(&odom_new[0],odom_new.size(),
				//			&odom_last[0],odom_last.size(),
				//			cvTermCriteria(CV_TERMCRIT_ITER,1,0.1));

				float err_icp = icp(&new_points[0], new_points.size(), 
							&ref_points[0], ref_points.size(),
							&r,&t,
							R,T,
							cvTermCriteria(CV_TERMCRIT_ITER,100,0.1));
				
				//float err_icp = icp(&new_points[0], new_points.size(), 
				//			&ref_points[0], ref_points.size(),
				//			&r,&t,
				//			R,T,
				//			cvTermCriteria(CV_TERMCRIT_ITER,200,0.1));
				
				//float err_icp = icp_pl(&new_points[0], new_points.size(), 
				//			&ref_points[0], ref_points.size(),
				//			&r,&t,
				//			R,T,
				//			cvTermCriteria(CV_TERMCRIT_ITER,200,0.1));
				total_err_icp += err_icp;
				total_scan++;
				printf("err_icp = %f\n",err_icp);
				printf("total_err_icp = %f\n", total_err_icp);
				printf("total_scan = %d\n", total_scan);

				jumlah_idx.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/jumlah_idx.txt", ios::in | ios::out | ios::app);
				jumlah_idx << idx << endl; //jumlah idx
				jumlah_idx.close();	

				float error=0;
				for(int i = 0; i <= idx; i++){
					float x = new_points[i].x;
					float y = new_points[i].y;
					float X = (R[0]*x + R[1]*y + T[0]);
					float Y = (R[2]*x + R[3]*y + T[1]);
					//new_points_gl[i].x = X;
					//new_points_gl[i].y = Y;
					datLaser[2][0][i] = X;
					datLaser[2][1][i] = Y;

					//error=error+ sqrt(pow((datLaser[2][0][i]-datLaser[0][0][i]),2) + pow((datLaser[2][1][i]-datLaser[0][1][i]),2));
			


				dataslam_x.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/dataslam_x.txt", ios::in | ios::out | ios::app);
				dataslam_x << datLaser[2][0][i] << endl; //X
				dataslam_x.close();


				dataslam_y.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/dataslam_y.txt", ios::in | ios::out | ios::app);
				dataslam_y << datLaser[2][1][i] << endl; //Y
				dataslam_y.close();

					if(loop>1 && loop%8==0){
						cv::circle(imgSave[1], Point(X,Y), 1, Scalar(0,255,255), 1);
						cv::circle(imgSave[2],Point(datLaser[1][0][i],datLaser[1][1][i]),1,Scalar(255,0,0),1);
						
						dr_x = centerX + (robot.odom_x * scaleFactor);
						dr_y = centerY - (robot.odom_y * scaleFactor);

						odom_drx.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/odom_drx.txt", ios::in | ios::out | ios::app);
						odom_drx << dr_x << endl; 
						odom_drx.close();

						odom_dry.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/odom_dry.txt", ios::in | ios::out | ios::app);
						odom_dry << dr_y << endl;
						odom_dry.close();

						cv::circle(imgSave[1], cv::Point(dr_x, dr_y), 1, cv::Scalar(255,53,184), -1);
					}
				}

				//printf("error=%f %f\n",error/(idx+1),(error/(idx+1))/scaleFactor);

				//errorku.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/error.txt", ios::in | ios::out | ios::app);
				//errorku << (error/(idx+1))/scaleFactor << endl;
				//errorku.close();


				printf("R[] aft = %f, %f, %f, %f\n",R[0],R[1],R[2],R[3]);
				printf("T[] aft = %f, %f\n", T[0],T[1]);


				//for(int h=0; h <= idx; h++){
				//	cv::circle(img, cv::Point(datLaser[1][0][h], datLaser[1][1][h]), 1, cv::Scalar(0,50,200), 1);
				//}
			
				//dr_x = centerX + (robot.odom_x * scaleFactor);
				//dr_y = centerY - (robot.odom_y * scaleFactor);
				//cv::circle(img, cv::Point(dr_x, dr_y), 1, cv::Scalar(0,250,0), -1);
				
				//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
				//cv::imshow("Display window", img);
				//cv::namedWindow("no ICP", cv::WINDOW_AUTOSIZE);
				//cv::imshow("no ICP", imgSave[2]);
				cv::namedWindow("with ICP", cv::WINDOW_AUTOSIZE);
				cv::imshow("with ICP", imgSave[1]);

				//cv::namedWindow("inside ICP", cv::WINDOW_AUTOSIZE);
				//cv::imshow("inside ICP", imgSave[3]);
				char s = cv::waitKey(1);

				

			}
			for(int i = 0; i <= idx; i++){
				tst_text_1 << "x = " << datLaser[1][0][i] << "\ty =  " << datLaser[1][1][i] << endl;
				tst_text_2 << "x = " << datLaser[2][0][i] << "\ty =  " << datLaser[2][1][i] << endl;
			}
			
			
			//printf("DONE \t");
			
		}
		xtemp = robot.odom_x;
		ytemp = robot.odom_y;
		
		
		idx = -1;
		//idx = 0;
		tst_text.close();
		tst_text_1.close();
		tst_text_2.close();


        ros::spinOnce();
        rate_1.sleep();
    }
    return 0;
}
