#include "stdafx.h"
#include "GTDisplay.h"

#define TA_DELTA 10
#define GT_FPS 120
#define SECOND_MS 1000
#define OFFSET 0

std::string PICTURE_SUFFIX = "ppm";

template<typename TK, typename TV>
std::vector<TK> get_keys(std::map<TK, TV> const& input_map) {
	std::vector<TK> keys;
	for (auto const& element : input_map) {
		keys.push_back(element.first);
	}
	return keys;
}

std::vector<std::pair<std::string, cv::Mat>> projectedImages;
void saveProjectedImages()
{
	for (auto img : projectedImages)
		cv::imwrite(img.first, img.second);
}

double get_nearest_ts(int* start_point, double device_ts, std::vector<double> const& ot_timestamps)
{
	double nearest_ot_ts = -1;
	double diff = std::numeric_limits<double>::max();

	int loc, index = *start_point;
	size_t i;
	//for (i = index/* - TA_DELTA*/; i < index + TA_DELTA && i < ot_timestamps.size(); i++)
	for (i = 0/* - TA_DELTA*/; i < ot_timestamps.size(); i++)
	{
		if (abs(ot_timestamps[i] - device_ts) < diff)
		{
			diff = abs(ot_timestamps[i] - device_ts);
			nearest_ot_ts = ot_timestamps[i];
			loc = i;

			//if (diff <= (SECOND_MS / GT_FPS / 2))
			//{
			//	
			//	if (i + OFFSET >= 0 && i + OFFSET < ot_timestamps.size())
			//	{
			//		*start_point = i + OFFSET;
			//		return ot_timestamps[i + OFFSET];// nearest_ot_ts;
			//	}
			//	else
			//	{
			//		*start_point = i;
			//		return  nearest_ot_ts;
			//	}
			//}
		}
	}
	return ot_timestamps[loc + OFFSET];// nearest_ot_ts;

	/*for (auto const& ot_ts : ot_timestamps)
	{
		if (abs(ot_ts - device_ts) < diff)
		{
			diff = abs(ot_ts - device_ts);
			nearest_ot_ts = ot_ts;
		}
	}*/
	/**start_point = i - 1;
	return nearest_ot_ts;*/
}

//TODO: add support to fish-eye images display
bool GT::Display::drawPoints(std::string imagesFolder, std::map<double, std::vector<cv::Point2d>> ot_projected_points, std::map<int, double> device_timestamps, bool saveImages, bool fe)
{
	if (fe)
		PICTURE_SUFFIX = "pgm";

	char search_path[200];
	const std::string COLOR_WINDOW_NAME = "Projected";
	int width = 640;
	int height = 480;
	sprintf_s(search_path, "%s\\*.%s", imagesFolder.c_str(),PICTURE_SUFFIX.c_str());
	WIN32_FIND_DATAA fd;
	HANDLE hFind = FindFirstFileA(search_path, &fd);
	cv::Mat src;
	cv::namedWindow(COLOR_WINDOW_NAME, CV_WINDOW_NORMAL);
	cv::resizeWindow(COLOR_WINDOW_NAME, width*3, height*3);
	cv::moveWindow(COLOR_WINDOW_NAME, 100, 100);

	if (hFind == INVALID_HANDLE_VALUE)
	{
		cv::destroyAllWindows();
		return false;
	}

	std::vector<double> ot_timestamps = get_keys(ot_projected_points);//get all ot timestamps
	int index = 0;
	projectedImages.clear();

	/*cv::Mat _img = cv::imread("C:\\Users\\ntuser\\Desktop\\GT_records\\fe\\extracted_fe\\fisheye\\image000100.pgm", CV_8UC3);
	cv::Mat img(_img.size(), CV_8UC3);
	cv::cvtColor(_img, img, CV_GRAY2RGB);
	cv::Mat m = img.clone();
	for each (auto var in ot_projected_points)
	{
		for (size_t i = 0; i < var.second.size(); i++)
		{
			cv::circle(m, var.second[i], 1, cv::Scalar(255, 0, 255), 2);
		}
		cv::imshow(COLOR_WINDOW_NAME, m);
		cv::waitKey(1);
		m = img.clone();
	}*/

	do
	{
		std::string picName = fd.cFileName;
		
		if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
		{
			src = cv::imread(imagesFolder + "\\" + fd.cFileName, fe ? CV_8UC1 : CV_8UC3);
			if (src.data == NULL)
			{
				std::cout << "failed to load image: " << fd.cFileName << std::endl;
				cv::destroyAllWindows();
				return false;
			}
			
			picName = picName.substr(picName.find_last_of('e') + 1, picName.find_first_of('.'));
			int frameNum = atoi(picName.c_str());
			
			double device_ts = device_timestamps[frameNum];
			double nearest_ot_ts = get_nearest_ts(&index, device_ts, ot_timestamps);
			std::cout << "index: " << index << " device ts: " << device_ts << " ot ts: " << ot_timestamps[index] << " nearest ts: " << nearest_ot_ts << std::endl;
			
			//cv::Mat img_rgb(src.size(), CV_8UC3);
			// convert grayscale to color image
			if(fe)
				cv::cvtColor(src, src, CV_GRAY2RGB);
			for (size_t j = 0; j < ot_projected_points[nearest_ot_ts].size(); j++)
				{
					cv::circle(src, ot_projected_points[nearest_ot_ts][j], 1, cv::Scalar(255, 0, 255), 2);
				}
			cv::putText(src, "device ts: " + std::to_string(device_ts), cv::Point(30, 200), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1, 1);
			cv::putText(src, picName, cv::Point(30, 100), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1, 1);

			if (saveImages)
				projectedImages.push_back(std::make_pair(imagesFolder + "\\projected\\" + std::to_string(frameNum) + ".ppm", src.clone()));
			
			cv::imshow(COLOR_WINDOW_NAME, src);
			cv::waitKey(1);
		}
	} while (FindNextFileA(hFind, &fd));
	
	FindClose(hFind);
	saveProjectedImages();
	return true;
	
}
