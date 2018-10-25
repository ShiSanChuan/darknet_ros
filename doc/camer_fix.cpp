#include <opencv2/opencv.hpp> 
#include <opencv2/features2d/features2d.hpp>
#include <iostream> 
#include <fstream> 
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string>  
#include <vector> 
using namespace cv; 
using namespace std; 
/**
 * 遍历文件夹获取图片全地址
 * @param path      文件夹路径
 * @param FilesName 文件ｖｅｃｔｏｒ
 */
void showfiles(const char * path,vector<string>&FilesName){
	if(NULL==path)return ;
	struct stat s;
	lstat(path,&s);
	if(!S_ISDIR(s.st_mode)){
		cout<<"dir name is error!"<<endl;
		return;
	}
	struct dirent *filename;
	DIR *dir;
	dir=opendir(path);
	if(NULL==dir){
		cout<<"can't open dir!!"<<endl;
		return ;
	}
	while((filename=readdir(dir))!=NULL){
		if(strcmp(filename->d_name,".")==0||
			strcmp(filename->d_name,"..")==0)
			continue;
		cout<<filename->d_name<<endl;
		string fullpath(path);
		fullpath+="/"+string(filename->d_name);
		FilesName.push_back(fullpath);
	}
}
void m_calibration(vector<string> &FilesName, Size board_size, 
					Size square_size, Mat &cameraMatrix, Mat &distCoeffs, 
					vector<Mat> &rvecsMat, vector<Mat> &tvecsMat) { 
	ofstream fout("caliberation_result.txt"); // 保存标定结果的文件 
	cout << "开始提取角点………………" << endl; 
	int image_count = 0; // 图像数量 
	Size image_size; // 图像的尺寸 
	vector<Point2f> image_points; // 缓存每幅图像上检测到的角点 
	vector<vector<Point2f>> image_points_seq; // 保存检测到的所有角点 
	for (int i = 0;i < FilesName.size();i++) { 
		image_count++; // 用于观察检验输出 
		cout << "image_count = " << image_count << endl; 
		Mat imageInput = imread(FilesName[i]); 
		if (image_count == 1) //读入第一张图片时获取图像宽高信息 
		{ 
			image_size.width = imageInput.cols; 
			image_size.height = imageInput.rows; 
			cout << "image_size.width = " << image_size.width << endl; 
			cout << "image_size.height = " << image_size.height << endl; 
		} /* 提取角点 */ 
		bool ok = findChessboardCorners(imageInput, board_size, image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); 
		if (0 == ok) {
			cout <<"第"<< image_count <<"张照片提取角点失败，请删除后，重新标定！"<<endl; //找不到角点 
			imshow("失败照片", imageInput); 
			waitKey(0); 
		} else { 
			Mat view_gray; 
			cout << "imageInput.channels()=" << imageInput.channels() << endl; 
			cvtColor(imageInput, view_gray, CV_RGB2GRAY); /* 亚像素精确化 */ 
			//find4QuadCornerSubpix(view_gray, image_points, Size(5, 5)); //对粗提取的角点进行精确化 
			cv::cornerSubPix(view_gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), 
							cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 20, 0.01)); 
			image_points_seq.push_back(image_points); //保存亚像素角点 
			/* 在图像上显示角点位置 */ 
			drawChessboardCorners(view_gray, board_size, image_points, true); 
			imshow("demo", view_gray);//显示图片 
			waitKey(500);//暂停0.1S     
		} 
	} 
	cout << "角点提取完成！！！" << endl; 
	/*棋盘三维信息*/ 
	vector<vector<Point3f>> object_points_seq; 
	// 保存标定板上角点的三维坐标 
	for (int t = 0;t < image_count;t++) { 
		vector<Point3f> object_points; 
		for (int i = 0;i < board_size.height;i++) { 
			for (int j = 0;j < board_size.width;j++) { 
				Point3f realPoint; /* 假设标定板放在世界坐标系中z=0的平面上 */ 
				realPoint.x = i*square_size.width; 
				realPoint.y = j*square_size.height; 
				realPoint.z = 0; 
				object_points.push_back(realPoint); 
			} 
		} 
		object_points_seq.push_back(object_points); 
	} 
	/* 运行标定函数 */ 
	double err_first = calibrateCamera(object_points_seq, image_points_seq, image_size, 
						cameraMatrix, distCoeffs, rvecsMat, tvecsMat, CV_CALIB_FIX_K3); 
	fout << "重投影误差1：" << err_first << "像素" << endl << endl; 
	cout << "标定完成！！！" << endl; cout << "开始评价标定结果………………"; 
	double total_err = 0.0; // 所有图像的平均误差的总和 
	double err = 0.0; // 每幅图像的平均误差 
	double totalErr = 0.0; 
	double totalPoints = 0.0; 
	vector<Point2f> image_points_pro; // 保存重新计算得到的投影点 
	for (int i = 0;i < image_count;i++) { 
		projectPoints(object_points_seq[i], rvecsMat[i], tvecsMat[i], 
						cameraMatrix, distCoeffs, image_points_pro); //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算 
		err = norm(Mat(image_points_seq[i]), Mat(image_points_pro), NORM_L2); 
		totalErr += err*err; 
		totalPoints += object_points_seq[i].size(); 
		err /= object_points_seq[i].size(); 
		//fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl; 
		total_err += err; } fout << "重投影误差2：" << sqrt(totalErr / totalPoints) << "像素" << endl << endl; 
		fout << "重投影误差3：" << total_err / image_count << "像素" << endl << endl; //保存定标结果    
		cout << "开始保存定标结果………………" << endl; 
		Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */ 
		fout << "相机内参数矩阵：" << endl; 
		fout << cameraMatrix << endl << endl; 
		fout << "畸变系数：\n"; 
		fout << distCoeffs << endl << endl << endl; 
		for (int i = 0; i < image_count; i++) { 
			fout << "第" << i + 1 << "幅图像的旋转向量：" << endl; 
			fout << rvecsMat[i] << endl; /* 将旋转向量转换为相对应的旋转矩阵 */ 
			Rodrigues(rvecsMat[i], rotation_matrix); 
			fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl; 
			fout << rotation_matrix << endl; 
			fout << "第" << i + 1 << "幅图像的平移向量：" << endl; 
			fout << tvecsMat[i] << endl << endl; 
		} 
		cout << "定标结果完成保存！！！" << endl; 
		fout << endl; 
	} 
void m_undistort(vector<string> &FilesName, Size image_size, Mat &cameraMatrix, Mat &distCoeffs) { 
	Mat mapx = Mat(image_size, CV_32FC1); //X 坐标重映射参数 
	Mat mapy = Mat(image_size, CV_32FC1); //Y 坐标重映射参数 
	Mat R = Mat::eye(3, 3, CV_32F); 
	cout << "保存矫正图像" << endl; 
	string imageFileName; //校正后图像的保存路径 
	stringstream StrStm; 
	string temp; 
	for (int i = 0; i < FilesName.size(); i++) { 
		Mat imageSource = imread(FilesName[i]); 
		Mat newimage = imageSource.clone(); 
		//方法一：使用initUndistortRectifyMap和remap两个函数配合实现 
		//initUndistortRectifyMap(cameraMatrix,distCoeffs,R, Mat(),image_size,CV_32FC1,mapx,mapy); 
		//  remap(imageSource,newimage,mapx, mapy, INTER_LINEAR); 
		//  
		//方法二：不需要转换矩阵的方式，使用undistort函数实现 
		undistort(imageSource, newimage, cameraMatrix, distCoeffs); 
		StrStm << i + 1; 
		StrStm >> temp; 
		imageFileName = "矫正后图像//" + temp + "_d.jpg"; 
		imwrite(imageFileName, newimage); 
		StrStm.clear(); 
		imageFileName.clear(); 
	} 
	std::cout << "保存结束" << endl; 
} 
vector<cv::Point2f> point[2];
std::vector<Point2f> goodfeatures;
int trackingpoints;
static void On_Mouse(int event,int x,int y,int flags,void*){
	if(event==cv::EVENT_LBUTTONDOWN){
		if(point[1].size()<4){
			for(size_t i=0;i<goodfeatures.size();i++)
				if(abs(goodfeatures[i].x-x)+abs(goodfeatures[i].y-y)<3){
					std::cout<<point[1].size()<<std::endl;
					point[1].push_back(goodfeatures[i]);
					trackingpoints++;
					break;
				}
		}
	}
}
int main() { 
	vector<string>FilesName1; //存放文件名的容器 
	const char * path ="picture/";
	showfiles(path,FilesName1);
	Size board_size = Size(8, 6); // 标定板上每行、列的角点数 
	Size square_size = Size(25, 25); // 实际测量得到的标定板上每个棋盘格的物理尺寸，单位mm 
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参数矩阵 
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 摄像机的5个畸变系数：k1,k2,p1,p2,k3 
	vector<Mat> rvecsMat; // 存放所有图像的旋转向量，每一副图像的旋转向量为一个mat 
	vector<Mat> tvecsMat; // 存放所有图像的平移向量，每一副图像的平移向量为一个mat 
	m_calibration(FilesName1, board_size, square_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat); 
	// m_undistort(FilesName1, image_size, cameraMatrix, distCoeffs); 
	//solvePnp
	VideoCapture cap;
	cap.open(1);
	namedWindow("demo",cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("demo", On_Mouse,0);
	cv::Mat frame,image,preimage,gray;
	std::vector<Point3d> model_point{
		Point3d(0,0,0),
		Point3d(0,25,0),
		Point3d(25,0,0),
		Point3d(25,25,0)
	};
	bool quit=false,needToGetgf=true;
	int jpgcount=0;
	Ptr<ORB> orb=ORB::create();
	while(!quit){
		cap>>frame;
		if(frame.empty()){
			cout<<"image error!"<<endl;
			break;
		}
		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);
		if(needToGetgf){//find corner
			goodfeatures.clear();
			vector<KeyPoint> keypoints;
			orb->detect(gray,keypoints);
			for(size_t i=0;i<keypoints.size();i++)
				goodfeatures.push_back(keypoints[i].pt);
			cornerSubPix(gray, goodfeatures, Size(5,5), Size(-1,-1), 
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,20,2));
			for(size_t i=0;i<goodfeatures.size();i++)
				circle(frame, goodfeatures[i], 3, Scalar(0,255,0),-1,8);
		}
		if(!point[0].empty()){
			std::vector<uchar> status;
			std::vector<float> err;
			if(preimage.empty())
				image.copyTo(preimage);
			calcOpticalFlowPyrLK(preimage, image, point[0], point[1], status, err);
			size_t k=0;
			for(int i=0;i<point[1].size();i++){
				if(!status[i])continue;
				point[1][k++]=point[1][i];
				circle(frame, point[1][i], 3, Scalar(0,0,255),-1,8);
			}
			if(k==4){
				needToGetgf=false;
				std::vector<Point2d> image_point;
				for(int i=0;i<k;i++)
					image_point.push_back(Point2d(point[1][i].x,point[1][i].y));
				cv::Mat rotation_vector;
				cv::Mat translation_vector;
				solvePnP(model_point,image_point,cameraMatrix, distCoeffs,rotation_vector,translation_vector);
				Rodrigues(rotation_vector, rotation_vector);
				// std::cout<<rotation_vector<<std::endl;
			}
		}
		if(point[1].size()==4)
			swap(point[0], point[1]);
		swap(preimage,image);

		char commend=char(waitKey(1));
		switch(commend){
			case 'q':quit=true;break;
			case 'p':imwrite("picture/20181020_"+to_string(jpgcount)+".jpg",image);jpgcount++;break;
			case 'c':point[0].clear();point[1].clear();needToGetgf=true;break;
		}
		cv::imshow("demo", frame);
	}

	return 0; 
}
