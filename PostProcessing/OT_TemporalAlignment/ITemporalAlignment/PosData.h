#include "IData.h"

#define RIGHT_STR "RIGHT" 
#define LEFT_STR "LEFT"

#pragma once

class OTData : public IData
{
public:

	typedef enum PoseStruct
	{
		ID = 0,
		TIMESTAMP,
		FRAMENUMBER,
		STATUS,
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

	}PoseIndex;

	typedef enum Status
	{
		UNTRACKED = 0,
		TRACKED,
		MARKER,
		EXCESS_ERROR
	} Status;

	enum Hands
	{
		RIGHT, LEFT
	};

	OTData();
	~OTData();

	std::string getFileExtension() override;

	std::string  dataToString() override;

	IData*  stringToData(std::string str) override;

	std::string getInnerDefPath(std::string mainFolder, void* = nullptr) override;

	static std::vector<std::string> getNextLineAndSplitIntoTokens(std::string line, char delimiter = ',');

	IData* linearInterpolation(IData* next, double timeStamp, int src_id) override;

	double poseInterpolation(quaternion q_prev, quaternion q_next, translation t_prev, translation t_next, double t/*, quaternion & q_res, translation & t_res*/);

	int m_frameNum;
	int m_status;
	double m_rotationsMat[rotMatSize][rotMatSize];
	double m_error;
	quaternion m_quaternion;
	translation m_transVector;

	std::string getName() override;
	bool hasTitles() override;

private:

	int OTData::parseObjectId(std::string id);
};

#pragma once
