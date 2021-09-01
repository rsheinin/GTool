#pragma once
#include <fstream>
#include "opencv2\opencv.hpp"
#include <wchar.h>
#include <comdef.h>
#include <string>
#include <stdint.h>
#include <conio.h>
#include <direct.h>
#include <regex>
#include <fstream>
//#include <iomanip>
#include <map>
//#include "IOini.h"
#include "GTDisplay.h"
#include "..\..\..\Common\GTUtils\GenericUtils.h"
#include "..\..\..\Common\GTUtils\IOini.h"

#define NOTYET -1
#define NOTFOUND -1
#define timestamp_length 18
#define TEMPORAL_ALIGNED_FOLDER "temporalAlignment"
#define CURRENT_DIRECTORY_NAME "projection"
#define TEMPORAL_ALIGNED_FILE_PREFIX "TA_"
#define INTERPOLATED_FILE_PREFIX "INTER_"
#define RIGIDBODY_FILE_EXTENSION ""//"_RigidBody"

std::string file_prefix = INTERPOLATED_FILE_PREFIX;
std::string folder;
std::string outFile;
std::string object_name;
std::string head_name;
std::string iniSettingsFile;
std::string imagesFolder;
std::string rcName;
std::string srcName;
std::string srcPath;
std::ofstream logger;
std::string headFile;
std::string folderToLoopPrefix = TEMPORAL_ALIGNED_FOLDER;
bool org_element = false, rel_element = false;
bool display = true;
bool saveImages = false;

std::vector<std::string> files;

double first_ts = NOTYET;
int OTPreviousMarkerFrameN = NOTYET;
double OTPreviousMarkerTS = NOTYET;

std::map<std::string, std::string> ghc_map;
const std::string relative_projection_suffix = "relative";
const std::string global_projection_suffix = "global";
const std::string input_file_extension = ".csv";
const std::string output_file_extension = ".tum";
const std::string source_file_extension = ".txt";

void readUserArgs(int argc, char ** argv);

std::string getInformation();

std::string helpMenu();

namespace Pose
{
	const std::string invalidPose = "NAN";
	enum PoseType
	{
		GROUNDTRUTH = 0,
		ALGORITHM
	};

	struct PoseStruct
	{
		int id = -1;
		double ts;
		int frameNumber;
		bool tracked;
		cv::Mat rotationMat = cv::Mat::eye(4, 4, CV_64F);
		//double error;
		QuaternionUtils::Quat q;
	};
	
	cv::Mat HMatrix2cv(HMatrix& M, QuaternionUtils::AxisData translation = { 0,0,0 })
	{
		cv::Mat cvMat(4, 4, CV_64F, M);
		cvMat.at<double>(3) = translation.m_x;
		cvMat.at<double>(7) = translation.m_y;
		cvMat.at<double>(11) = translation.m_z;
		return cvMat.clone();
	}
	void quat2cvMat(QuaternionUtils::Quat quaternion, cv::Mat& cvMat)
	{
		HMatrix hm;
		Mat_FromQuat(quaternion.m_quat.toNN_Quat(), hm);
		cvMat = HMatrix2cv(hm, quaternion.m_trans);
	}
	void quatToMatrix(QuaternionUtils::Quat q, double m[4][4])
	{
		double sqw = q.m_quat.m_w * q.m_quat.m_w;
		double sqx = q.m_quat.m_x * q.m_quat.m_x;
		double sqy = q.m_quat.m_y * q.m_quat.m_y;
		double sqz = q.m_quat.m_z * q.m_quat.m_z;
		if (sqx + sqy + sqz + sqw == 0)
			return;
		//invs (inverse square length) is only required if quaternion is not already normalised
		double invs = 1 / (sqx + sqy + sqz + sqw);
		m[0][0] = (sqx - sqy - sqz + sqw)*invs;
		m[1][1] = (-sqx + sqy - sqz + sqw)*invs;
		m[2][2] = (-sqx - sqy + sqz + sqw)*invs;

		double xy = q.m_quat.m_x * q.m_quat.m_y;
		double zw = q.m_quat.m_z * q.m_quat.m_w;
		m[1][0] = 2.0 * (xy + zw)*invs;
		m[0][1] = 2.0 * (xy - zw)*invs;

		double xz = q.m_quat.m_x * q.m_quat.m_z;
		double yw = q.m_quat.m_y * q.m_quat.m_w;
		m[2][0] = 2.0 * (xz - yw)*invs;
		m[0][2] = 2.0 * (xz + yw)*invs;

		double yz = q.m_quat.m_y * q.m_quat.m_z;
		double xw = q.m_quat.m_x * q.m_quat.m_w;
		m[2][1] = 2.0 * (yz + xw)*invs;
		m[1][2] = 2.0 * (yz - xw)*invs;

		m[0][3] = q.m_quat.m_x;
		m[1][3] = q.m_quat.m_y;
		m[2][3] = q.m_quat.m_z;

		m[3][0] = m[3][1] = m[3][2] = 0;
		m[3][3] = 1;
		return;
	}
	bool strVec2Matrix(std::vector<std::string> tokens, cv::Mat& mat, char delimiter = ',', int start_index = 0)
	{
		if (tokens.size() < start_index + 12)
		{
			return false;
		}

		mat = cv::Mat::eye(4, 4, CV_64F);
		for (int i = 0; i < 12; i++)
		{
			mat.at<double>(i) = std::stod(tokens[i + start_index]);
		}

		return true;
	}

	namespace GT
	{
		const char pose_delimiter = ',';
		typedef enum GTPoseEnum
		{
			ID = 0,
			TIMESTAMP,
			FRAMENUMBER,
			ISTRACKED,

			R0,
			R1,
			R2,
			TX,
			R4,
			R5,
			R6,
			TY,
			R8,
			R9,
			R10,
			TZ,
			ERR,

			QX,
			QY,
			QZ,
			QW,

			POSESIZE,

			EX,
			EY,
			EZ

		} GTPose;
		PoseStruct str2pose(std::string str, char delimiter = GT::pose_delimiter)
		{
			Pose::PoseStruct p;
			std::vector<std::string> tokens;
			StringUtils::str2tokens(str, tokens);
			if (tokens.size() < Pose::GT::POSESIZE)
				return p;

			p.id = std::stoi(tokens[Pose::GT::ID]);
			p.ts = std::stod(tokens[Pose::GT::TIMESTAMP]);
			p.frameNumber = std::stoi(tokens[Pose::GT::FRAMENUMBER]);
			std::istringstream(tokens[Pose::GT::ISTRACKED]) >> p.tracked;

			std::vector<std::string> sub(&tokens[Pose::GT::R0], &tokens[Pose::GT::TZ] + 1);
			Pose::strVec2Matrix(sub, p.rotationMat);
			p.q = QuaternionUtils::Quat(std::stod(tokens[Pose::GT::QX]), std::stod(tokens[Pose::GT::QY]), std::stod(tokens[Pose::GT::QZ]), std::stod(tokens[Pose::GT::QW]),
				std::stod(tokens[Pose::GT::TX]), std::stod(tokens[Pose::GT::TY]), std::stod(tokens[Pose::GT::TZ]));
			//p.error = std::stod(tokens[Pose::GTPose::ERR]);
			return p;
		}
		bool str2Quat(std::string str, cv::Mat& mat, char delimiter = GT::pose_delimiter)
		{
			std::istringstream iss(str);
			std::vector<std::string> tokens;
			while (iss.good())
			{
				std::string substr;
				std::getline(iss, substr, delimiter);
				tokens.push_back(substr + delimiter);
			}
			if (tokens.size() < POSESIZE - ISTRACKED)
			{
				return false;
			}

			mat = cv::Mat::eye(7, 1, CV_64F);
			mat.at<double>(QuaternionUtils::QX) = std::stod(tokens[QX - R0]);
			mat.at<double>(QuaternionUtils::QY) = std::stod(tokens[QY - R0]);
			mat.at<double>(QuaternionUtils::QZ) = std::stod(tokens[QZ - R0]);
			mat.at<double>(QuaternionUtils::QW) = std::stod(tokens[QW - R0]);

			mat.at<double>(QuaternionUtils::TX) = std::stod(tokens[TX - R0]);
			mat.at<double>(QuaternionUtils::TY) = std::stod(tokens[TY - R0]);
			mat.at<double>(QuaternionUtils::TZ) = std::stod(tokens[TZ - R0]);

			return true;
		}
	}
	namespace Source
	{
		const int initPhase = 10;
		const char pose_delimiter = ' ';
		typedef enum SrcPoseEnum
		{
			TIMESTAMP = 0,
			TX,
			TY,
			TZ,
			QX,
			QY,
			QZ,
			QW,
			POSESIZE,
			ID = 8,
			CONFIDENCE,
			ARR_TS

		} SrcPose;
		PoseStruct str2Pose(std::string str, char delimiter = Source::pose_delimiter)
		{
			Pose::PoseStruct p;
			std::vector<std::string> tokens;
			StringUtils::str2tokens(str, tokens, delimiter);
			if (tokens.size() < Pose::Source::POSESIZE)
				return p;

			if (tokens.size() > Pose::Source::ID)
				p.id = std::stoi(tokens[Pose::Source::ID]);
			else
				p.id = TM2::HMD;
			p.ts = std::stod(tokens[Pose::Source::TIMESTAMP]) * 1000;
			p.q = QuaternionUtils::Quat(std::stod(tokens[Pose::Source::QX]), std::stod(tokens[Pose::Source::QY]), std::stod(tokens[Pose::Source::QZ]), std::stod(tokens[Pose::Source::QW]),
				std::stod(tokens[Pose::Source::TX]) * 1000, std::stod(tokens[Pose::Source::TY]) * 1000, std::stod(tokens[Pose::Source::TZ]) * 1000);
			quat2cvMat(p.q, p.rotationMat);
			return p;
		}
	}	
	std::string mat2str(cv::Mat m, char delimiter = GT::pose_delimiter)
	{
		std::string str;

		for (int i = 0; i < 12; i++)
		{
			std::stringstream s1;
			s1 << std::fixed << std::setprecision(16) << m.at<double>(i);
			str += s1.str() + delimiter;
		}
		return str;
	}
	std::string mat2str(cv::Mat m, float q)
	{
		return mat2str(m) + " " + std::to_string(q);
	}
	
	bool str2Matrix(std::string str, cv::Mat& mat, char delimiter = GT::pose_delimiter, int start_index = 0)
	{
		std::istringstream iss(str);
		std::vector<std::string> tokens/*{ std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>{} }*/;
		while (iss.good())
		{
			std::string substr;
			std::getline(iss, substr, delimiter);
			tokens.push_back(substr + delimiter);
		}
		/*if (tokens.size() < start_index + 12)
		{
		return false;
		}

		mat = cv::Mat::eye(4, 4, CV_64F);
		for (int i = 0; i < 12; i++)
		{
		mat.at<double>(i) = atof(tokens[i + start_index].c_str());
		}

		return true;*/
		return strVec2Matrix(tokens, mat, delimiter, start_index);
	}
	
}

namespace Projection
{
	typedef std::map<int, double> timestamps;
	//typedef std::vector<std::pair<std::pair<int64_t, std::string>,bool>> poses;
	//typedef std::vector<std::pair<std::pair<double, std::string>, bool>> poses;
	typedef std::vector<Pose::PoseStruct> poses;

	struct CameraIntrinsics
	{
		double fx;
		double fy;
		double px;
		double py;
		double d_w;
		uint32_t w;
		uint32_t h;
		double k[5];
		CameraIntrinsics() : fx(0), fy(0), px(0), py(0), w(0), h(0)
		{
			k[0] = k[1] = k[2] = k[3] = k[4] = 0;
		}
	};

	enum Stream
	{
		COLOR = 0,
		FE
	};

	enum Type
	{
		PROJECTION_2D = 0b01,
		PROJECTION_3D = 0b10
	};

	void updateProjectedObjectName(std::string markerFile);
	timestamps ReadDeviceTS(std::string path);

	std::map<double, std::vector<cv::Point2d>> imageColorSpacePoints; //vector for 2d points. intended for projected points to DS color-image space, to be drawn on cv images
	std::vector<cv::Point2d> imagePoints_tmp; //vector of 2d projected points for single frame. contains all markers observed in one frame
	std::map < int, std::vector<cv::Point2d> > alignedFrames;



	cv::Point2f distortPoint(cv::Point2f p);

	namespace Fisheye
	{
		const float FOVModelEpsilon = 1e-5f;
		void distortFisheyePoints(cv::InputArray undistorted, cv::OutputArray distorted, cv::Size imageSize, cv::InputArray K, float w, cv::InputArray R);
	}
}

int stream = Projection::COLOR;

int getIndexOfNearestTS(double ts, Projection::poses& cameraPoses);
QuaternionUtils::Quat getFirstSrcPose(double& ts);
//QuaternionUtils::Quat getFirstSrcPose(double& ts);
bool getReferencePoseIndexes(const Projection::poses& srcPoses, const Projection::poses& gtPoses, int* srcIndex, int* gtIndex);


namespace GHCUtils 
{
	std::string getGhcByRBName(std::string);
}