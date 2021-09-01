#pragma once

#include <algorithm>
#include <math.h> 
#include <sstream>
#include <vector>
//#include "GenericUtils.h"
#include "..\..\..\Common\GTUtils\GenericUtils.h"

 class OTData;

 class IData abstract
{
public:

	static const int NOTYET = -1;

	enum InterpolationType
	{
		LINEAR, NEAREST, NONE
	};
	static std::string to_string(InterpolationType t)
	{
		switch (t)
		{
		case LINEAR:
			return "linear";
		case NEAREST:
			return "nearest";
		case NONE:
			return "none";
		default:
			return std::to_string(t);
		}
	}

	virtual std::string getName() = 0;
	
	virtual bool hasTitles() = 0;

	virtual std::string getFileExtension() { return ".txt"; }

	const static int rotMatSize = 3;

	virtual std::string  dataToString() = 0;

	virtual IData*  stringToData(std::string str) = 0;

	virtual std::string getInnerDefPath(std::string mainFolder, void* = nullptr) = 0;

	virtual IData* linearInterpolation(IData* next, double timeStamp, int src_id)=0;

	double m_timeStamp;
	int m_id = NOTYET;

	IData * IData::nearestInterpolation(IData * next, double timeStamp)
	{
		if (std::abs(this->m_timeStamp - timeStamp) <= std::abs(next->m_timeStamp - timeStamp))
			return this;
		else
			return next;
	}

};

