#include <Windows.h>
//#include <stdio.h>
#include <iostream>
#include <exception>
#include <limits>
#include "opencv2\opencv.hpp"

namespace GT 
{
	namespace Display
	{
		//void drawPoints(std::string imagesFolder, std::vector<cv::Point2d> points);
		bool drawPoints(std::string imagesFolder, std::map<double, std::vector<cv::Point2d>> points, std::map<int,double> timestamps,bool saveImages, bool fe);
	}
	/*namespace Output
	{
		std::vector<std::pair<std::string, cv::Mat>> projectedImages;
		void saveProjectedImages();
	}*/
}