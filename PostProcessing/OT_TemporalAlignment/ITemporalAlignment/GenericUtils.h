#pragma once

#ifndef UTILS_H
#define UTILS_H

#include <conio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <Windows.h>
#include <array>
#define M_PI       3.14159265358979323846

typedef std::array<double, 4> quaternion;
typedef std::array<double, 3> translation;

namespace FormatUtils
{
	const std::string initialValue = "-1";
	const char delimiter = ',';
	const std::string convertedPrefix = "c_";
	
	typedef enum PoseStruct
	{
		RB_ID = 0,
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
		POSESIZE,//move to end?

		QX,
		QY,
		QZ,
		QW,

		EX,
		EY,
		EZ,

		
	}PoseIndex;

	static std::string to_string(PoseIndex val)
	{
		switch (val)
		{
		case FormatUtils::RB_ID:
			return "id";
		case FormatUtils::TIMESTAMP:
			return "timestamp";
		case FormatUtils::FRAMENUMBER:
			return "frame number";
		case FormatUtils::ISTRACKED:
			return "status";
		case FormatUtils::R0:
			return "r_o";
		case FormatUtils::R1:
			return "r_1";
		case FormatUtils::R2:
			return "r_2";
		case FormatUtils::TX:
			return "t_x";
		case FormatUtils::R4:
			return "r_4";
		case FormatUtils::R5:
			return "r_5";
		case FormatUtils::R6:
			return "r_6";
		case FormatUtils::TY:
			return "t_y";
		case FormatUtils::R8:
			return "r_8";
		case FormatUtils::R9:
			return "r_9";
		case FormatUtils::R10:
			return "r_10";
		case FormatUtils::TZ:
			return "t_z";
		case FormatUtils::QX:
			return "q_x";
		case FormatUtils::QY:
			return "q_y";
		case FormatUtils::QZ:
			return "q_z";
		case FormatUtils::QW:
			return "q_w";
		case FormatUtils::EX:
			return "e_x";
		case FormatUtils::EY:
			return "e_y";
		case FormatUtils::EZ:
			return "e_z";
		case FormatUtils::ERR:
			return "err";
		case FormatUtils::POSESIZE:
			return std::to_string(POSESIZE);
		default:
			break;
		}
		return "";
	}

	typedef size_t sourceIndex;
	typedef int exp;
	typedef std::tuple<sourceIndex, PoseIndex, exp> exchangeRule;

	enum TupleStruct
	{
		SOURCE_INDEX = 0,
		POSE_INDEX,
		EXPONENT
	};

	static bool writeTitles(std::ofstream* file)
	{
		if (file->is_open())
		{
			for (size_t i = 0; i < POSESIZE; i++)
			{
				*file << to_string((PoseIndex)i) << delimiter;
			}
			*file << std::endl;
			return true;
		}
		return false;
	}
	template <typename T>
	std::string to_string_with_precision(const T a_value, const int n = 16)
	{
		std::ostringstream out;
		out << std::setprecision(n) << a_value;
		return out.str();
	}

	static bool convert2KnownFormat(std::string path, std::vector<exchangeRule>/*std::vector<std::pair<int, int>>*/ exchangeRules, bool hasTitle = true)
	{
		std::ifstream sourceFile(path);
		std::string sourceFileName;
		int loc = path.find_last_of("\\");
		sourceFileName = path.substr(loc + 1);
		std::string folderPath = path.substr(0, loc);
		std::ofstream file(folderPath + "\\" + convertedPrefix + sourceFileName);
		if (!sourceFile.is_open())
		{
			std::cout << "Cann't open " << sourceFileName << " file" << std::endl;
			return NULL;
		} 

		std::string sourceLine;
		if (hasTitle)
		{
			std::getline(sourceFile, sourceLine);
			if (!writeTitles(&file))
				file << std::endl;
		}
		while (std::getline(sourceFile, sourceLine))//read a line from source file
		{
			std::istringstream iss(sourceLine);
			std::vector<std::string> lineSep;

			while (iss.good())//divide line to values by a specific delimiter (default: comma)
			{
				std::string substr;
				std::getline(iss, substr, delimiter);
				lineSep.push_back(substr);
			}

			if (lineSep.size() == 0)//or a known size of line (get as parameter)?
				break;
			std::vector<std::string> newLine(POSESIZE, initialValue);
			for each (auto rule in exchangeRules)//set up new line with source values in correct locations
			{
				if (lineSep.size() <= std::get<SOURCE_INDEX>(rule))
					return false;
				newLine[std::get<POSE_INDEX>(rule)] = to_string_with_precision(stod(lineSep[std::get<SOURCE_INDEX>(rule)]) * pow(10, std::get<EXPONENT>(rule)));
			}
			std::string line = "";
			for each (std::string value in newLine)
			{
				line += value;
				line += delimiter;
			}
			file << line << std::endl;
			
		}
		sourceFile.close();
		file.close();
		return true;
	}
}

namespace OutputUtils
{
	static HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	enum TextColor
	{
		BLACK = 0,
		BLUE,
		GREEN,
		LIGHTBLUE,
		RED,
		PINK,
		YRLLOW,
		WHITE,
		GRAY,
		ERROR_COLOR = 12,
		DEFAULT_COLOR = 15,
		WARNING_COLOR = 14,
		SHINING = 8

	};

	static void printColoredText(std::string str, int color = DEFAULT_COLOR)
	{
		SetConsoleTextAttribute(hConsole, color);
		std::cout << str << std::endl;
		SetConsoleTextAttribute(hConsole, DEFAULT_COLOR);
	}

	//help function for fatal messages
	static void errorMessage(std::string msg)
	{
		//std::cerr << msg << std::endl << "Press any key to exit" << std::endl;
		printColoredText(msg, ERROR_COLOR);
		//std::cerr << "Press any key to exit" << std::endl;
		//_getch();
		exit(0);
	}

	static void tune()
	{
		std::cout << '\a';
		Beep(523, 500); // 523 hertz (C5) for 500 milliseconds
	}
	
	
}

namespace StringUtils
{
	static void str2tokens(std::string str, std::vector<std::string> &results, char delimiter = ',')
	{
		std::stringstream ss(str);

		while (ss.good())
		{
			std::string substr;
			getline(ss, substr, delimiter);
			results.push_back(substr);
		}
	}
}

namespace QuaternionUtils
{
	enum quaternionStruct
	{
		QX = 0,
		QY,
		QZ,
		QW,
		Q_ROTATIONSIZE,
		TX = Q_ROTATIONSIZE,
		TY,
		TZ,
		Q_TRANSLATIONSIZE,
		QUATERNIONSIZE = Q_TRANSLATIONSIZE
	};
	//typedef std::array<double, 4> quaternion;
	//mutiply the inverse of qa with qb
	static quaternion mutiply(quaternion qa, quaternion qb)
	{
		quaternion res;
		res[QX] = qb[QX] * qa[QW] - qb[QY] * qa[QZ] + qb[QZ] * qa[QY] - qb[QW] * qa[QX];
		res[QY] = qb[QX] * qa[QZ] + qb[QY] * qa[QW] - qb[QZ] * qa[QX] - qb[QW] * qa[QY];
		res[QZ] = -qb[QX] * qa[QY] + qb[QY] * qa[QX] + qb[QZ] * qa[QW] - qb[QW] * qa[QZ];
		res[QW] = qb[QX] * qa[QX] + qb[QY] * qa[QY] + qb[QZ] * qa[QZ] + qb[QW] * qa[QW];
		return res;
	}
	static quaternion inverse(quaternion q)
	{
		quaternion q_;
		q_[QX] = -q[QX];
		q_[QY] = -q[QY];
		q_[QZ] = -q[QZ];
		q_[QW] = q[QW];
		return q_;
	}
	static double magnitude(quaternion q)
	{
		if (q[QuaternionUtils::QW] > 1 && q[QuaternionUtils::QW] < 1.000019)//round to 1 to enable arcos
			q[QuaternionUtils::QW] = 1;
		return 2 * acos(q[QuaternionUtils::QW]) < 2 * M_PI - 2 * acos(q[QuaternionUtils::QW]) ? 2 * acos(q[QuaternionUtils::QW]) : 2 * M_PI - 2 * acos(q[QuaternionUtils::QW]);
	}
	static quaternion arr2quaternion(double arr[Q_ROTATIONSIZE] )
	{
		quaternion q;
		for (size_t i = 0; i < Q_ROTATIONSIZE; i++)
		{
			q[i] = arr[i];
		}
		return q;
	}

	static double dot(quaternion qa, quaternion qb)
	{
		/*if (qa.size() != qb.size())
			return NAN;*/

		double res = 0.0;
		for (size_t i = 0; i < Q_ROTATIONSIZE; i++)
		{
			res += (qa[i] * qb[i]);
		}
		return res;
	}

	static quaternion linearInterpolation(quaternion q1, quaternion q2, double t)
	{
		quaternion q;
		for (size_t i = 0; i < Q_ROTATIONSIZE; i++)
		{
			q[i] = q1[i] + t*(q2[i] - q1[i]);
		}
		return q;
	}

	static quaternion negative(quaternion q)
	{
		quaternion nq;
		for (size_t i = 0; i < Q_ROTATIONSIZE; i++)
		{
			nq[i] = -q[i];
		}
		return nq;
	}
	class Quat
	{
		double m_x;
		double m_y;
		double m_z;
		double m_w;
		//quaternion quat;
		NN_Quat quat;
		translation tran;
	public:
		/*Quat()
		{
			m_x = 0;
			m_y = 0;
			m_z = 0;
			m_w = 0;
		}*/
		Quat(double x = 0.0, double y = 0.0, double z = 0.0, double w = 0.0)
		{
			m_x = x;
			m_y = y;
			m_z = z;
			m_w = w;
		}
		Quat(quaternion q)
		{
			m_x = q[QX];
			m_y = q[QY];
			m_z = q[QZ];
			m_w = q[QW];
		}
		Quat(HMatrix M)
		{
			EulerAngles ea = Eul_FromHMatrix(M, EulOrdXYZr);
			NN_Quat nn_q = Eul_ToQuat(ea);
			m_x = nn_q.x;
			m_y = nn_q.y;
			m_z = nn_q.z;
			m_w = nn_q.w;

			quat.x = nn_q.x;
			quat.y = nn_q.y;
			quat.z = nn_q.z;
			quat.w = nn_q.w;

			for (size_t i = 0; i < 3; i++)
			{
				tran[i] = M[i][3];
			}
		}
		Quat(std::string str, char delimiter = ',')
		{
			std::vector<std::string> pose;
			StringUtils::str2tokens(str, pose, delimiter);
			if (pose.size() < 12)
				return;
			if (pose.size() == 12)
			{
				std::vector<std::string> pose_end = { "0","0","0","1" };
				pose.insert(std::end(pose), std::begin(pose_end), std::end(pose_end));
			}
			HMatrix M;
			for (int i = 0; i <= 3; i++)
			{
				for (int j = 0; j <= 3; j++)
				{
					M[i][j] = stof(pose[i * 4 + j]);
				}
			}
			*this = Quat(M);
			/*EulerAngles ea = Eul_FromHMatrix(M, EulOrdXYZr);
			NN_Quat nn_q = Eul_ToQuat(ea);
			m_x = nn_q.x;
			m_y = nn_q.y;
			m_z = nn_q.z;
			m_w = nn_q.w;
			for (size_t i = 0; i < 3; i++)
			{
				tran[i] = M[i][3];
			}*/
		}
		Quat(translation t)
		{
			m_x = t[QX];
			m_y = t[QY];
			m_z = t[QZ];
		}
		// operator overloading
		bool operator==(const Quat &q) const
		{
			return (m_x == q.m_x) && (m_y == q.m_y)
				&& (m_z == q.m_z) && (m_w == q.m_w);
		}
		Quat operator*(const Quat &q) const
		{
			Quat temp(m_x * q.m_x, m_y * q.m_y,
				m_z * q.m_z, m_w * q.m_w);
			return temp;
		}
		Quat operator*(const double v) const
		{
			Quat temp(m_x * v, m_y * v,
				m_z * v, m_w * v);
			return temp;
		}
		Quat operator+(const Quat &q) const
		{
			Quat temp(m_x + q.m_x, m_y + q.m_y,
				m_z + q.m_z, m_w + q.m_w);
			return temp;
		}
		Quat operator-(const Quat &q) const
		{
			Quat temp1(1, 2,
				3, 4);
			Quat temp(m_x - q.m_x, m_y - q.m_y,
				m_z - q.m_z, m_w - q.m_w);
			return temp;
		}
		Quat  operator-() const      // negative
		{
			return Quat(-m_x, -m_y, -m_z, -m_w);
		}
		double dot(const Quat &q)
		{
			return m_x * q.m_x + m_y * q.m_y +
				m_z * q.m_z + m_w * q.m_w;
		}
		quaternion quat2quaternion()
		{
			quaternion q = { m_x,m_y,m_z,m_w };
			return q;
		}
		translation quat2translation()
		{
			translation t = { m_x,m_y,m_z };
			return t;
		}
		Quat linearInterpolation(Quat q2, double t)
		{
			return *this + (q2 - *this)*t;
		}

	};
}

//namespace PoseUtils
//{
//# define ROT_MAT_SIZE 3
//# define ERROR_CODE -2
//	cv::Mat rotMat2RelativeRotMat(cv::Mat curr, cv::Mat prev)
//	{
//		/*double mat[3][3] = {1,0,0,0,1,0,0,0,1};
//		cv::Mat rot((cv::Mat_<double>(ROT_MAT_SIZE, ROT_MAT_SIZE) << mat[0][0], mat[0][0], mat[0][0], mat[0][0], mat[0][0], mat[0][0], mat[0][0], mat[0][0], mat[0][0]));
//		return cv::Mat();*/
//		return prev.inv() * curr;
//	}
//
//	double magnitude(cv::Mat rotation)
//	{
//		if(rotation.size().width == ROT_MAT_SIZE && rotation.size().height == ROT_MAT_SIZE)
//			return acos((rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2) - 1) / 2);
//		return ERROR_CODE;
//	}
//
//}
#endif // UTILS_H