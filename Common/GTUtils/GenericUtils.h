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
#include "PoseUtils.h"
#include <map>
#define M_PI       3.14159265358979323846

typedef std::array<double, 4> quaternion;
typedef std::array<double, 3> translation;

template <typename T>
const T linearInterpolate(const T a, const T b, double time)
{
	return a + (b - a)*time;
}

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

	static void printColoredText(std::string str, std::ofstream& logger, int color = DEFAULT_COLOR)
	{
		logger << str << std::endl;
		printColoredText(str, color);
	}
	
	//help function for warning messages
	static void warningMessage(std::string msg)
	{
		printColoredText(msg, WARNING_COLOR);
	}

	static void warningMessage(std::string msg, std::ofstream& logger)
	{
		printColoredText(msg, logger, WARNING_COLOR);
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

	static void errorMessage(std::string msg, std::ofstream& logger)
	{
		logger << "ERROR: " << msg << std::endl;
		errorMessage(msg);
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
	static std::vector<std::string> str2tokens(std::string str, char delimiter = ',')
	{
		std::vector<std::string> results;
		std::stringstream ss(str);

		while (ss.good())
		{
			std::string substr;
			getline(ss, substr, delimiter);
			if(!substr.empty())
				results.push_back(substr);
		}
		return results;
	}
}


namespace QuaternionUtils
{
	/*enum quaternionStruct
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
	};*/
	enum quaternionStruct
	{
		QX = 0,
		QY,
		QZ,
		QW,
		QUATERNIONSIZE
	};
	enum transkationStruct
	{
		TX = 0,
		TY,
		TZ,
		TRANSLATIONSIZE
	};
	enum quaternionOrder
	{
		QxyzwTxyz = 0,
		QwxyzTxyz,
		TxyzQxyzw,
		TxyzQwxyz
	};
	//typedef std::array<double, 4> quaternion;
	//mutiply the inverse of qa with qb
	static quaternion mutiply_inv(quaternion qa, quaternion qb)
	{
		quaternion res;
		res[QX] = qb[QX] * qa[QW] - qb[QY] * qa[QZ] + qb[QZ] * qa[QY] - qb[QW] * qa[QX];
		res[QY] = qb[QX] * qa[QZ] + qb[QY] * qa[QW] - qb[QZ] * qa[QX] - qb[QW] * qa[QY];
		res[QZ] = -qb[QX] * qa[QY] + qb[QY] * qa[QX] + qb[QZ] * qa[QW] - qb[QW] * qa[QZ];
		res[QW] = qb[QX] * qa[QX] + qb[QY] * qa[QY] + qb[QZ] * qa[QZ] + qb[QW] * qa[QW];
		return res;
	}
	static quaternion mutiply(quaternion qa, quaternion qb)
	{
		quaternion res;

		res[QX] = qa[QX] * qb[QW] + qa[QY] * qb[QZ] - qa[QZ] * qb[QY] + qa[QW] * qb[QX];
		res[QY] = -qa[QX] * qb[QZ] + qa[QY] * qb[QW] + qa[QZ] * qb[QX] + qa[QW] * qb[QY];
		res[QZ] = qa[QX] * qb[QY] - qa[QY] * qb[QX] + qa[QZ] * qb[QW] + qa[QW] * qb[QZ];
		res[QW] = -qa[QX] * qb[QX] - qa[QY] * qb[QY] - qa[QZ] * qb[QZ] + qa[QW] * qb[QW];

		/*res[QX] = qb[QX] * qa[QW] + qb[QY] * qa[QZ] - qb[QZ] * qa[QY] + qb[QW] * qa[QX];
		res[QY] = -qb[QX] * qa[QZ] + qb[QY] * qa[QW] + qb[QZ] * qa[QX] + qb[QW] * qa[QY];
		res[QZ] = qb[QX] * qa[QY] - qb[QY] * qa[QX] + qb[QZ] * qa[QW] + qb[QW] * qa[QZ];
		res[QW] = -qb[QX] * qa[QX] - qb[QY] * qa[QY] - qb[QZ] * qa[QZ] + qb[QW] * qa[QW];*/
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
	static quaternion arr2quaternion(double arr[QUATERNIONSIZE] )
	{
		quaternion q;
		for (size_t i = 0; i < QUATERNIONSIZE; i++)
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
		for (size_t i = 0; i < QUATERNIONSIZE; i++)
		{
			res += (qa[i] * qb[i]);
		}
		return res;
	}

	static quaternion linearInterpolation(quaternion q1, quaternion q2, double t)
	{
		quaternion q;
		for (size_t i = 0; i < QUATERNIONSIZE; i++)
		{
			q[i] = q1[i] + t*(q2[i] - q1[i]);
		}
		return q;
	}

	static quaternion negative(quaternion q)
	{
		quaternion nq;
		for (size_t i = 0; i < QUATERNIONSIZE; i++)
		{
			nq[i] = -q[i];
		}
		return nq;
	}

	class AxisData
	{
	public:

		double m_x;
		double m_y;
		double m_z;
		double m_w;

		AxisData(double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 0.0)
		{
			m_x = qx;
			m_y = qy;
			m_z = qz;
			m_w = qw;
		}
		AxisData(quaternion q)
		{
			m_x = q[QX];
			m_y = q[QY];
			m_z = q[QZ];
			m_w = q[QW];
		}

		AxisData(translation t)
		{
			m_x = t[QX];
			m_y = t[QY];
			m_z = t[QZ];
		}

		// operator overloading
		bool operator==(const AxisData &ad) const
		{
			return (m_x == ad.m_x) && (m_y == ad.m_y)
			&& (m_z == ad.m_z) && (m_w == ad.m_w);
		}
		AxisData operator*(const AxisData &ad) const
		{
			AxisData temp(m_x * ad.m_x, m_y * ad.m_y,
			m_z * ad.m_z, m_w * ad.m_w);
			return temp;
		}
		AxisData operator*(const double v) const
		{
			AxisData temp(m_x * v, m_y * v,
			m_z * v, m_w * v);
			return temp;
		}
		AxisData operator+(const AxisData &ad) const
		{
			AxisData temp(m_x + ad.m_x, m_y + ad.m_y,
			m_z + ad.m_z, m_w + ad.m_w);
			return temp;
		}
		/*friend std::ostream& operator<<(std::ostream& os, const AxisData& ad)
		{
			os << "qx: " << ad.m_x << ", qy: " << ad.m_y << ", qz: " << ad.m_z << ", qw: " << ad.m_w;
			return os;
		}*/
		AxisData operator-(const AxisData &ad) const
		{
			AxisData temp(m_x - ad.m_x, m_y - ad.m_y,
			m_z - ad.m_z, m_w - ad.m_w);
			return temp;
		}
		AxisData  operator-() const      // negative
		{
			return AxisData(-m_x, -m_y, -m_z, -m_w);
		}

		double dot(const AxisData &ad)
		{
			return m_x * ad.m_x + m_y * ad.m_y +
				m_z * ad.m_z + m_w * ad.m_w;
		}
		quaternion axisData2quaternion()
		{
			quaternion q = { m_x,m_y,m_z,m_w };
			return q;
		}
		translation axisData2translation()
		{
			translation t = { m_x,m_y,m_z };
			return t;
		}
		AxisData linearInterpolation(AxisData ad2, double t)
		{
			//return *this + (ad2 - *this)*t;
			return linearInterpolate(*this, ad2, t);
		}
		NN_Quat toNN_Quat()
		{
			NN_Quat nnq = {m_x,m_y,m_z,m_w};
			return nnq;
		}

	private:

	};

	class Quat
	{
	public:
		AxisData m_quat;
		AxisData m_trans;
		quaternionOrder order = QxyzwTxyz;

		Quat(double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0,
			 double tx = 0.0, double ty = 0.0, double tz = 0.0)
		{
			m_quat = AxisData(qx, qy, qz, qw);
			m_trans = AxisData(tx, ty, tz);
		}
		Quat(quaternion q, translation t = {0,0,0})
		{
			m_quat = AxisData(q[QX], q[QY], q[QZ], q[QW]);
			m_trans = AxisData(t[TX], t[TY], t[TZ]);
		}
		Quat(std::string str, quaternionOrder order = QxyzwTxyz, char delimiter = ',')
		{
			std::vector<std::string> tokens;
			StringUtils::str2tokens(str, tokens, delimiter);
			if (tokens.size() != QUATERNIONSIZE + TRANSLATIONSIZE)
			{
				OutputUtils::errorMessage("invalid string for quaternion (string is: " + str + ")");
			}
			switch (order)
			{
			case QuaternionUtils::QxyzwTxyz:
				m_quat = AxisData(std::stod(tokens[QX]), std::stod(tokens[QY]), std::stod(tokens[QZ]), std::stod(tokens[QW]));
				m_trans = AxisData(std::stod(tokens[QUATERNIONSIZE + TX]), std::stod(tokens[QUATERNIONSIZE + TY]), std::stod(tokens[QUATERNIONSIZE + TZ]));
				break;
			case QuaternionUtils::QwxyzTxyz:
				m_quat = AxisData(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[0]));
				m_trans = AxisData(std::stod(tokens[QUATERNIONSIZE + TX]), std::stod(tokens[QUATERNIONSIZE + TY]), std::stod(tokens[QUATERNIONSIZE + TZ]));
				break;
			case QuaternionUtils::TxyzQxyzw:
				m_quat = AxisData(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]));
				m_trans = AxisData(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
				break;
			case QuaternionUtils::TxyzQwxyz:
				m_quat = AxisData(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[3]));
				m_trans = AxisData(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2]));
				break;
			}
		}
		Quat(HMatrix M)
		{
			EulerAngles ea = Eul_FromHMatrix(M, EulOrdXYZr);
			NN_Quat nn_q = Eul_ToQuat(ea);
			m_quat = AxisData(nn_q.x, nn_q.y, nn_q.z, nn_q.w);
			m_trans = AxisData(M[TX][TRANSLATIONSIZE], M[TY][TRANSLATIONSIZE], M[TZ][TRANSLATIONSIZE]);
		}
		
		// operator overloading
		bool operator==(const Quat &q) const
		{
			return (m_quat == q.m_quat) && (m_trans == q.m_trans);
		}
		Quat operator*(const Quat &q) const
		{
			Quat res;
			/*res.m_quat.m_x = q.m_quat.m_x * this->m_quat.m_w + q.m_quat.m_y * this->m_quat.m_z - q.m_quat.m_z * this->m_quat.m_y + q.m_quat.m_w * this->m_quat.m_x;
			res.m_quat.m_y = -q.m_quat.m_x * this->m_quat.m_z + q.m_quat.m_y * this->m_quat.m_w + q.m_quat.m_z * this->m_quat.m_x + q.m_quat.m_w * this->m_quat.m_y;
			res.m_quat.m_z = q.m_quat.m_x * this->m_quat.m_y - q.m_quat.m_y * this->m_quat.m_x + q.m_quat.m_z * this->m_quat.m_w + q.m_quat.m_w * this->m_quat.m_z;
			res.m_quat.m_w = -q.m_quat.m_x * this->m_quat.m_x - q.m_quat.m_y * this->m_quat.m_y - q.m_quat.m_z * this->m_quat.m_z + q.m_quat.m_w * this->m_quat.m_w;*/

			if (*this == zeros() || q == zeros())
				return zeros();

			res.m_trans.m_x = m_trans.m_x + m_quat.m_w*m_quat.m_w*q.m_trans.m_x + 2 * m_quat.m_y*m_quat.m_w*q.m_trans.m_z - 2 * m_quat.m_z*m_quat.m_w*q.m_trans.m_y + m_quat.m_x*m_quat.m_x*q.m_trans.m_x + 2 * m_quat.m_y*m_quat.m_x*q.m_trans.m_y + 2 * m_quat.m_z*m_quat.m_x*q.m_trans.m_z - m_quat.m_z*m_quat.m_z*q.m_trans.m_x - m_quat.m_y*m_quat.m_y*q.m_trans.m_x;
			res.m_trans.m_y = m_trans.m_y + 2 * m_quat.m_x*m_quat.m_y*q.m_trans.m_x + m_quat.m_y*m_quat.m_y*q.m_trans.m_y + 2 * m_quat.m_z*m_quat.m_y*q.m_trans.m_z + 2 * m_quat.m_w*m_quat.m_z*q.m_trans.m_x - m_quat.m_z*m_quat.m_z*q.m_trans.m_y + m_quat.m_w*m_quat.m_w*q.m_trans.m_y - 2 * m_quat.m_x*m_quat.m_w*q.m_trans.m_z - m_quat.m_x*m_quat.m_x*q.m_trans.m_y;
			res.m_trans.m_z = m_trans.m_z + 2 * m_quat.m_x*m_quat.m_z*q.m_trans.m_x + 2 * m_quat.m_y*m_quat.m_z*q.m_trans.m_y + m_quat.m_z*m_quat.m_z*q.m_trans.m_z - 2 * m_quat.m_w*m_quat.m_y*q.m_trans.m_x - m_quat.m_y*m_quat.m_y*q.m_trans.m_z + 2 * m_quat.m_w*m_quat.m_x*q.m_trans.m_y - m_quat.m_x*m_quat.m_x*q.m_trans.m_z + m_quat.m_w*m_quat.m_w*q.m_trans.m_z;

			res.m_quat.m_x = m_quat.m_x * q.m_quat.m_w + m_quat.m_y * q.m_quat.m_z - m_quat.m_z * q.m_quat.m_y + m_quat.m_w * q.m_quat.m_x;
			res.m_quat.m_y = -m_quat.m_x * q.m_quat.m_z + m_quat.m_y * q.m_quat.m_w + m_quat.m_z * q.m_quat.m_x + m_quat.m_w * q.m_quat.m_y;
			res.m_quat.m_z = m_quat.m_x * q.m_quat.m_y - m_quat.m_y * q.m_quat.m_x + m_quat.m_z * q.m_quat.m_w + m_quat.m_w * q.m_quat.m_z;
			res.m_quat.m_w = -m_quat.m_x * q.m_quat.m_x - m_quat.m_y * q.m_quat.m_y - m_quat.m_z * q.m_quat.m_z + m_quat.m_w * q.m_quat.m_w;
			
			
			return res;
		}

		friend std::ostream& operator<<(std::ostream& os, const Quat& q)
		{
			os << "qx: " << q.m_quat.m_x << ", qy: " << q.m_quat.m_y << ", qz: " << q.m_quat.m_z << ", qw: " << q.m_quat.m_w << std::endl
				<< "tx: " << q.m_trans.m_x << ", ty: " << q.m_trans.m_y << ", tz: " << q.m_trans.m_z;
			return os;
		}
		/*Quat operator*(const double v) const
		{
			Quat temp(m_quat * v, m_trans  * v);
			return temp;
		}
		Quat operator+(const Quat &q) const
		{
			Quat temp(m_quat + q.m_quat, m_trans  + q.m_trans);
			return temp;
		}
		Quat operator-(const Quat &q) const
		{
			Quat temp(m_quat - q.m_quat, m_trans - q.m_trans);
			return temp;
		}*/
		Quat inv()
		{
			Quat res;
			res.m_quat.m_x = -m_quat.m_x;
			res.m_quat.m_y = -m_quat.m_y;
			res.m_quat.m_z = -m_quat.m_z;
			res.m_quat.m_w = m_quat.m_w;

			res.m_trans.m_x = -(m_quat.m_w*m_quat.m_w*m_trans.m_x - 2 * m_quat.m_y*m_quat.m_w*m_trans.m_z + 2 * m_quat.m_z*m_quat.m_w*m_trans.m_y + m_quat.m_x*m_quat.m_x*m_trans.m_x + 2 * m_quat.m_y*m_quat.m_x*m_trans.m_y + 2 * m_quat.m_z*m_quat.m_x*m_trans.m_z - m_quat.m_z*m_quat.m_z*m_trans.m_x - m_quat.m_y*m_quat.m_y*m_trans.m_x);
			res.m_trans.m_y = -(2 * m_quat.m_x*m_quat.m_y*m_trans.m_x + m_quat.m_y*m_quat.m_y*m_trans.m_y + 2 * m_quat.m_z*m_quat.m_y*m_trans.m_z - 2 * m_quat.m_w*m_quat.m_z*m_trans.m_x - m_quat.m_z*m_quat.m_z*m_trans.m_y + m_quat.m_w*m_quat.m_w*m_trans.m_y + 2 * m_quat.m_x*m_quat.m_w*m_trans.m_z - m_quat.m_x*m_quat.m_x*m_trans.m_y);
			res.m_trans.m_z = -(2 * m_quat.m_x*m_quat.m_z*m_trans.m_x + 2 * m_quat.m_y*m_quat.m_z*m_trans.m_y + m_quat.m_z*m_quat.m_z*m_trans.m_z + 2 * m_quat.m_w*m_quat.m_y*m_trans.m_x - m_quat.m_y*m_quat.m_y*m_trans.m_z - 2 * m_quat.m_w*m_quat.m_x*m_trans.m_y - m_quat.m_x*m_quat.m_x*m_trans.m_z + m_quat.m_w*m_quat.m_w*m_trans.m_z);
			return res;
		}

		std::string toStr(char delimiter = ',')
		{
			std::string str;
			std::stringstream s1;
			s1 << std::fixed << std::setprecision(16) << m_quat.m_x << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_quat.m_y << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_quat.m_z << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_quat.m_w << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_trans.m_x << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_trans.m_y << delimiter;
			s1 << std::fixed << std::setprecision(16) << m_trans.m_z;
			str += s1.str();

			return str;
		}

		static Quat matStr2Quat(std::string str, char delimiter = ' ')
		{
			std::vector<std::string> pose;
			StringUtils::str2tokens(str, pose, delimiter);
			if (pose.size() < 12)
			{
				OutputUtils::errorMessage("too short string for quaternion (string is: " + str + ")");
			}
			if (pose[pose.size() - 1] == "")
				pose.pop_back();
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
			return Quat(M);
		}

		static Quat eye()
		{
			return Quat(0, 0, 0, 1, 0, 0, 0);
		}
		static Quat zeros()
		{
			return Quat(0, 0, 0, 0, 0, 0, 0);
		}
	};
}

namespace TM2
{
	const std::string hmd_suffix = "hmd";
	const std::string ctrl_suffix = "ctrl1";
	const std::string ctrl2_suffix = "ctrl2";
	const std::string all = "all";
	enum object
	{
		HMD = 0,
		CTRL1,
		CTRL2,
		ALL
	};
	static std::string to_string(object obj)
	{
		switch (obj)
		{
		case HMD:
			return hmd_suffix;
		case CTRL1:
			return ctrl_suffix;
		case CTRL2:
			return ctrl2_suffix;
		case ALL:
			return all;
		default:
			return std::to_string(obj);
		}
	}
	static std::map<std::string, object> objects = { {to_string(HMD), HMD },{ to_string(CTRL1), CTRL1 },{ to_string(CTRL2), CTRL2 } };
	//std::map<std::string, object> objects = { { hmd_suffix, HMD },{ ctrl_suffix, CTRL },{ ctrl2_suffix, CTRL2 } };
	enum confidence
	{
		excellent = 1,
		good,
		bad
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