#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
using namespace std;
using namespace cv;
//85 255 0 40 0 60
//bgr 0 0 80  bgr 40 160 255
cv::Mat frame,dst,kernel,origen;
int Bmin=0,Gmin=0,Rmin=85;
int Bmax=76,Gmax=60,Rmax=225;
void Set_RGB(int Rmin,int Rmax,int Gmin,int Gmax,int Bmin,int Bmax){
	std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hireachy;
	Point2f center;
    float radius=20;
    // cv::cvtColor(origen, frame, cv::COLOR_BGR2HSV);//或者使用ＨＳＶ
    frame=origen.clone();
	inRange(frame, Scalar(Bmin,Gmin,Rmin), Scalar(Bmax,Gmax,Rmax), dst); //bgr
	morphologyEx(dst,dst,MORPH_CLOSE,kernel); 
	findContours(dst, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0)); 
	//框选面积最大的边界 
	if (contours.size() > 0){ 
		double maxArea=0; 
		for (int i = 0; i < contours.size(); i++){ 
			double area = contourArea(contours[static_cast<int>(i)]); 
			if (area > maxArea){ 
				maxArea = area; 
				minEnclosingCircle(contours[static_cast<int>(i)], center, radius); 
			}
		}
	} 
	//圆形框 
	circle(frame, Point(center.x,center.y), (int)radius, Scalar(0,255,0), 2);
	imshow("input", frame);
	imshow("output", dst);
	std::cout<<frame.cols<<frame.rows<<std::endl;
	std::cout<<center.x<<"\t"<<center.y<<"\t"<<radius<<std::endl;
}
void On_Change(int,void*){
	Set_RGB(Rmin,Rmax,Gmin,Gmax,Bmin,Bmax);
}

int main(int argc, char const *argv[])
{
	if(argc<2){
		std::cerr<<"./Inrang name.jpg"<<std::endl;
		return -1;
	}
	string name(argv[1]);
	std::cout<<name<<std::endl;
	origen=cv::imread(name,1);
	kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	Set_RGB(Rmin,Rmax,Gmin,Gmax,Bmin,Bmax);
	createTrackbar("Rmin", "output", &Rmin,225,On_Change);
	createTrackbar("Rmax", "output", &Rmax,225,On_Change);
	createTrackbar("Bmin", "output", &Bmin,255,On_Change);
	createTrackbar("Bmax", "output", &Bmax,255,On_Change);
	createTrackbar("Gmin", "output", &Gmin,255,On_Change);
	createTrackbar("Gmax", "output", &Gmax,255,On_Change);
	waitKey(0);

	return 0;
}
