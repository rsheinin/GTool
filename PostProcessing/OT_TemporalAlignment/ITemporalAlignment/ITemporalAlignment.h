#include <direct.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <windows.h> 
#include "IData.h"

#pragma once

class clockDriftCompensator;

class ITemporalAlignment abstract
{
public:

#define TA_FOLDER "temporalAlignment"
#define TA_FILEPREFIX "TA_"
#define INTER_FILEPREFIX "INTER_"

#define BIG_JUMP 100
#define SMALL_JUMP 0.1

	ITemporalAlignment();
	~ITemporalAlignment() {}

	enum Type
	{
		ALG, GT
	};

	static std::string to_string(Type type)
	{
		switch (type)
		{
		case ALG:
			return "algorithm";
		case GT:
			return "ground-truth";
		default:
			return std::to_string(type);
		}
	}

	enum TextColor
	{
		ERROR_COLOR = 12,
		DEFAULT_COLOR = 15,
		WARNING_COLOR = 14
	};

	double temporalAlignment();

	//read the csv file and put the data in a data vector
	virtual void readData(std::string path, Type fileType, TM2::object obj) = 0;

	virtual double calculateDifference(int algIndx, int gtIndx) = 0;

	virtual void exportAlignedFiles() = 0;

	void changeGTTimeStamps();

	void interpolation();

	int findPrevTimeStamp(int algindx, int gtStartIndx);

	std::string getInformation();
	
	bool loopFolder = false;

	void setCDCAmplitude(double size);

protected:

	//gt parameters
	std::vector<IData*> m_gtData;
	std::vector<IData*> m_alignedGtData;
	std::string m_gtHeader = "";
	std::string m_gtPath;
	std::string m_gtName;
	double m_fpsGT;
	TM2::object m_gtObject = TM2::ALL;

	//alg (source) parameters
	std::vector<IData*> m_algData;
	std::string m_algHeader = "";
	std::string m_algPath;
	std::string m_algName;
	IData* m_algDataType;
	double m_algFPS;
	TM2::object m_algObject = TM2::ALL;
	
	//alignment parameters
	std::string m_outFolderPath = "";
	std::string m_outFolderName = "";
	double m_bestDelta;
	double m_bestDeltaDiffsAvg;	
	IData* m_gtDataType;
	double m_currDelta = NAN;	
	int m_srcStartIndex = 0;
	int m_comparisonFramesNum = 6500;
	int m_initialUserDelta = 0;
	int m_amplitudeSize_ms = 30000;		

	//interpolation parameters
	std::vector<IData*> m_interpolatedGtData;
	std::string m_interpolationDataPath;
	IData* m_interpolationDataType;
	IData::InterpolationType m_interpolation;
	TM2::object m_interpolationObject = TM2::ALL;
	std::string m_interpolationFileSuffix;

	//for delta fine-tuning 
	virtual double exactDelta(double orgDelta) = 0;

	std::ofstream* m_resultsFile = NULL;

	clockDriftCompensator *m_cd_compensator;

private:

	void printResults();

	void printProcessData();
	
	int findNextClosestGtPose(double alg_ts, int gtStartIndx, double delta);

	void findBestDelta();

	void findBestFactor();

	void findSpecificFactor(double initialValue, double interval);

	void findSpecificDelta(double delta, int deltaSize, double jumpSize);

	double initialDelta();

	void createOutputFolder();

	void closeFiles();

	std::ofstream* m_diffsFile = NULL;

	double const m_minError = 0.001, m_maxError = 0.08;

};

class clockDriftCompensator
{
public:
	double m_initialFactor = 1;//1.000035
	double m_initialInterval = 0.001;
	double m_minInterval = m_initialInterval / 1000;
	double m_previousFactor = 1.0;
	double m_bestFactor = m_initialFactor;
	int m_devisionStep = 10;
	std::ofstream *m_outFile;
	std::string m_outFileName = "cdc.txt";
	int m_amplitudeSize = 9;
	void multiplyTimestampsByFactor(std::vector<IData*> &poses, double factor);
	std::string m_headerLine = "diff,delta,difference,isCounted,status";
private:
	
};



