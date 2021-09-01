#pragma once

#include "ITemporalAlignment.h"

//a class of specific temporal alignment for csv files
class CsvFilesTemporalAlignment : public ITemporalAlignment
{
public:
	void readData(std::string path, Type fileType, TM2::object obj = TM2::object::ALL) override;
	void exportAlignedFiles();
	void exportAlignedGtFile(std::string path);
	virtual double calculateDifference(int algIndx, int gtIndx) = 0;
	
};
