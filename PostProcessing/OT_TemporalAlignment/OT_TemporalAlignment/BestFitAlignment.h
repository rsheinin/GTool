//#pragma once

#include "CsvFilesTemporalAlignment.h"
#include "PosData.h"
#include <opencv2/opencv.hpp>
#include "..\Common\GTUtils\PoseUtils.h"
#include "..\Common\GTUtils\GenericUtils.h"
#include "..\Common\GTUtils\IOini.h"

namespace processArguments
{
	typedef std::map<std::string, std::string> procArgs;

	static const std::string IO_PARAMS = "io";
	static const std::string GT_PARAMS = "gt";
	static const std::string ALIGNMENT_PARAMS = "ta";
	static const std::string INTERPOLATION_PARAMS = "interp";
	static const std::string CLOCKDRIFT_PARAMS = "cdc";
	static const std::string OTHER_PARAMS = "o";

	//processArguments argsNames = { {"IO_PARAMS", "io"}, { "GT_PARAMS", "gt" }, { "ALIGNMENT_PARAMS", "ta" }, { "INTERPOLATION_PARAMS", "interp" }, { "CLOCKDRIFT_PARAMS", "cdc" }, { "OTHER_PARAMS", "other" } };
}

static const std::string OPTITRACK = "ot";
static const std::string GYRO = "gyro";
static const std::string SIXDOF = "6dof";

enum sourceType
{
	GYROTYPE = 0,
	SIXDOFTYPE
};
static std::string sourceType2str(sourceType st)
{
	switch (st)
	{
	case GYROTYPE:
		return GYRO;
	case SIXDOFTYPE:
		return SIXDOF;
	default:
		return std::to_string(st);
	}
}

std::string getFileSuffix(std::string path);

class MagnitudeAlignment :public CsvFilesTemporalAlignment
{
#define ERROR_CODE -2//TODO: change to a logical value
	

public:	

	MagnitudeAlignment()
	{

	}
	MagnitudeAlignment(processArguments::procArgs params);
	MagnitudeAlignment(std::string configFilePath);
	void setObject(std::string obj_str, TM2::object &object);
	void setType(std::string type_str, IData* &type);
	double calculateDifference(int algIndx, int gtNextIndx);
	double exactDelta(double orgDelta);
	void readData(std::string path, Type fileType, TM2::object obj = TM2::object::ALL) override;
	std::string getAlignmentSourceName() { return m_source; }
	
protected:

	std::string m_source = SIXDOF;
	cv::Mat rotMat2RelativeRotMat(cv::Mat curr, cv::Mat prev);
	double magnitude(cv::Mat rotation);

private:
	bool preProcess_src();
	int cleanupSpikesByTrace_GT(std::vector<IData*>& gtData);
	double tempDelta = NAN;
	int m_slidingWindowSize = 100;//length of the sliding window in frames number
	int m_compareSectionLength_ms = 100000;//length of the section for comparison in milliseconds
	std::vector<std::pair<quaternion, int>> src_vec;
	std::vector<std::pair<quaternion, int>> gt_vec;
	static const double startMovement;
	bool Init(std::string io_params, std::string gt_params, std::string ta_params, std::string interp_params, std::string cdc_params, std::string other_params);
	char paramsDelimiter = ',';
};

class GyroData : public IData
{
	static const int AXIS = 3;
	static const int POSELINESIZE = 10;
	typedef enum VelocityStruct
	{
		VX = 0,
		VY,
		VZ,
		V_SIZE
	}VelocityIndex;
	
	IData*  stringToData(std::string str) override;
	std::string getInnerDefPath(std::string mainFolder, void* = nullptr) override;
	std::string  dataToString() override;
	IData* linearInterpolation(IData* next, double timeStamp, int src_id) override;
public:
	double m_velocity[AXIS];
	double m_magnitude = 0;
	quaternion comulativeQuat;
	std::string getName();
	bool hasTitles();
	
};
class _6dofData : public IData
{
	static const int AXIS = 3;
	typedef enum _6dofStruct
	{
		TIMESTAMP = 0,		
		TX,
		TY,
		TZ,
		QX,
		QY,
		QZ,
		QW,
		IDENTIFIER,
		POSESIZE
	}_6dofIndex;

	enum object
	{
		HMD = 0,
		CTRL1,
		CTRL2
	};

	IData*  stringToData(std::string str) override;
	std::string getInnerDefPath(std::string mainFolder, void* = nullptr) override;
	std::string  dataToString() override;
	IData* linearInterpolation(IData* next, double timeStamp, int src_id) override;
	
public:
	double m_translation[AXIS];
	quaternion m_quaternion;
	double m_magnitude = 0;
	std::string getName();
	bool hasTitles();
};
static const std::map<std::string, IData*> srcTypes = { { GYRO, new GyroData() },{ SIXDOF, new _6dofData() },{ OPTITRACK, new OTData() } };
