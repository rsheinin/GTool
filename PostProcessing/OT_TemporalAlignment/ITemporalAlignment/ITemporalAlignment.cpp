#include "ITemporalAlignment.h"

ITemporalAlignment::ITemporalAlignment()
{
	m_cd_compensator = new clockDriftCompensator();
}

double ITemporalAlignment::temporalAlignment()
{	
	createOutputFolder();

	//read alg and gt files
	readData(m_algPath, ITemporalAlignment::ALG, m_algObject);
	if (m_algData.size() == 0)
		OutputUtils::errorMessage("bad alg file", *m_resultsFile);

	readData(m_gtPath, ITemporalAlignment::GT, TM2::ALL);
	if (m_gtData.size() == 0)
		OutputUtils::errorMessage("bad gt file", *m_resultsFile);

	printProcessData();

	//find the best factor for compensation the clock drift between 2 systems
	findBestFactor();
	
	//find the best delta
	findBestDelta();
		
	printResults();

	//after the best delta was found -> change the time stamps of all the gt files and interpolate according to wanted timestamps list
	exportAlignedFiles();

	closeFiles();

	return m_bestDeltaDiffsAvg;
}

void ITemporalAlignment::findBestDelta()
{	
	m_diffsFile = new std::ofstream(m_outFolderPath + "\\diffs.csv");	
	*m_diffsFile << std::endl << m_cd_compensator->m_headerLine << std::endl;//print header to diffs file
	
	m_cd_compensator->multiplyTimestampsByFactor(m_gtData, (m_cd_compensator->m_bestFactor / m_cd_compensator->m_previousFactor));
	findSpecificDelta(initialDelta() + m_initialUserDelta, m_amplitudeSize_ms, BIG_JUMP);
}

void ITemporalAlignment::findBestFactor()
{
	if (m_cd_compensator->m_amplitudeSize > 0)
	{
		m_cd_compensator->m_outFile = new std::ofstream(m_outFolderPath + "\\" + m_cd_compensator->m_outFileName);
		findSpecificFactor(m_cd_compensator->m_initialFactor, m_cd_compensator->m_initialInterval);
		if (m_bestDeltaDiffsAvg > m_maxError || m_bestDeltaDiffsAvg < m_minError)//found factor is not good enough
		{
			m_cd_compensator->m_bestFactor = m_cd_compensator->m_initialFactor;
			OutputUtils::printColoredText("WARNING: good factor for clock drift compensation has not been found. The initial value of: " + std::to_string(m_cd_compensator->m_initialFactor) + " will be used for compensation factor.", *m_resultsFile, OutputUtils::WARNING_COLOR);
		}
	}
}

void ITemporalAlignment::findSpecificDelta(double delta, int deltaSize, double jumpSize)
{
	if (jumpSize < SMALL_JUMP)
		return;

	float minFrac = 0.95;// the minimum fraction of m_comparisonFramesNum
	float minGoodFrames = 0.7;
	int currentGtIndx;
	double bestDelta;
	double bestDeltaDiffsAvg;
	bool first = true;
	bool isDeltaFound = false;

	double minDelta = delta - deltaSize;
	double maxDelta = delta + deltaSize;

	int framesNumber = m_algData.size();

	//first delta:
	double allDiffs = 0.0;
	currentGtIndx = 0;
	
	int gtIndx;

	//go over all optional deltas and find the best one
	for (double delta = minDelta; delta <= maxDelta; delta = delta + jumpSize)
	{
		m_currDelta = delta;
		double allDiffs = 0.0;
		currentGtIndx = 0;
		int srcIndex = m_srcStartIndex;

		do
		{
			gtIndx = findNextClosestGtPose(m_algData[srcIndex]->m_timeStamp, currentGtIndx, delta);
			srcIndex++;
		} while (currentGtIndx == gtIndx && srcIndex < framesNumber);

		int countGoodFrames = 0;
		int countFrames = 0;
		for (srcIndex = srcIndex - 1; srcIndex < framesNumber && (gtIndx < (m_gtData).size() - 1) && countFrames < m_comparisonFramesNum; srcIndex++)
		{
			gtIndx = findNextClosestGtPose(m_algData[srcIndex]->m_timeStamp, currentGtIndx, delta);
			double diff = calculateDifference(srcIndex, gtIndx);
			if (!(std::isnan(diff)))
			{
				allDiffs += diff;
				countGoodFrames++;
			}
			currentGtIndx = gtIndx - 100 >= 0 ? gtIndx - 100 : 0; //the gt index to start the search from the next time, go 100 frames back because the data can be not ordered
			countFrames++;
		}

		//found a delta with a smaller diff
		if ((first == true || (allDiffs / countGoodFrames < bestDeltaDiffsAvg)) && (double)countGoodFrames / countFrames >= minGoodFrames)//if it's a good delta and was checked on more than 80% good frames
		{			
			if (countGoodFrames != 0)
			{
				//delta must be checked for at least minFrac*m_comparisonFramesNum frames number
				if (countFrames >= minFrac*m_comparisonFramesNum)
				{
					bestDelta = delta;
					bestDeltaDiffsAvg = allDiffs / countGoodFrames;				

					if (first == true)
					{
						first = false;
						isDeltaFound = true;
					}
				}
			}
		}

		std::string diff = delta > minDelta ? std::to_string(jumpSize) : "";

		if (m_diffsFile != NULL)
		{
			//couldn't find any fitness for given deltas
			if (first == true && countFrames >= minFrac*m_comparisonFramesNum)
				*m_diffsFile << diff << "," << FormatUtils::to_string_with_precision(delta) << ",NAN," << false << ",not found" << std::endl;
			else
			{
				if ((double)countGoodFrames / countFrames < minGoodFrames)
					*m_diffsFile << diff << "," << FormatUtils::to_string_with_precision(delta) << "," << allDiffs / countGoodFrames << "," << false << ",not enough good frames" << std::endl;
				else
				{
					if (countFrames < minFrac*m_comparisonFramesNum)
						*m_diffsFile << diff << "," << FormatUtils::to_string_with_precision(delta) << "," << allDiffs / countGoodFrames << "," << false << ",too short comparison section" << std::endl;
					else
						*m_diffsFile << diff << "," << FormatUtils::to_string_with_precision(delta) << "," << allDiffs / countGoodFrames << "," << true << ",good" << std::endl;
				}
			}
		}

	}

	if (isDeltaFound == false)//no good delta was found
	{
		OutputUtils::errorMessage("No delta was found. Is the delta size too small?", *m_resultsFile);
	}

	m_bestDelta = bestDelta;
	m_bestDeltaDiffsAvg = bestDeltaDiffsAvg;

	if(jumpSize <= 10)
		findSpecificDelta(bestDelta, jumpSize * 6, jumpSize / 10);
	else
		findSpecificDelta(bestDelta, jumpSize * 2, jumpSize / 10);

}

double ITemporalAlignment::initialDelta()
{
	return (m_algData[0]->m_timeStamp - m_gtData[0]->m_timeStamp);
}

void ITemporalAlignment::createOutputFolder()
{
	this->m_gtName = m_gtPath.substr(m_gtPath.find_last_of("\\") + 1, m_gtPath.find_last_of('.') - (m_gtPath.find_last_of("\\") + 1));
	this->m_algName = m_algPath.substr(m_algPath.find_last_of("\\") + 1, m_algPath.find_last_of('.') - (m_algPath.find_last_of("\\") + 1));
	m_outFolderPath = m_gtPath.substr(0, m_gtPath.find_last_of("\\")) + "\\";
	if(m_outFolderName.empty())
		m_outFolderName = (loopFolder ? "val_" : "dev_") + m_algDataType->getName() + "_" + TA_FOLDER + "_" + m_gtName;
	m_outFolderPath += m_outFolderName;
	auto err = _mkdir(m_outFolderPath.c_str()) != 0;
	char message[200];
	strerror_s(message, 200, errno);
	std::cout << "creating folder for output files: " << message << " << " << m_outFolderPath << std::endl;
	m_resultsFile = new std::ofstream(m_outFolderPath + "\\logger.txt");
}

void ITemporalAlignment::closeFiles()
{
	if (m_cd_compensator->m_outFile != nullptr)
	{
		m_cd_compensator->m_outFile->flush();
		m_cd_compensator->m_outFile->close();
	}
	m_resultsFile->flush();
	m_resultsFile->close();
	m_diffsFile->flush();
	m_diffsFile->close();
}

void ITemporalAlignment::setCDCAmplitude(double size)
{
	m_cd_compensator->m_amplitudeSize = size;
}

//print the best delta data to results file and to console
void ITemporalAlignment::printResults()
{
	OutputUtils::printColoredText("-------------------Results-------------------", *m_resultsFile);
	OutputUtils::printColoredText("best factor: " + FormatUtils::to_string_with_precision(m_cd_compensator->m_bestFactor), *m_resultsFile);
	OutputUtils::printColoredText("best delta: " + FormatUtils::to_string_with_precision(m_bestDelta), *m_resultsFile);
	OutputUtils::printColoredText("diffs average: " + FormatUtils::to_string_with_precision(m_bestDeltaDiffsAvg), *m_resultsFile);
	double absDeltaFirstTS = m_bestDelta + (m_gtData[0]->m_timeStamp - (m_gtData[0]->m_timeStamp / m_cd_compensator->m_bestFactor)),
		absDeltaLastTS = m_bestDelta + (m_gtData.back()->m_timeStamp - (m_gtData.back()->m_timeStamp / m_cd_compensator->m_bestFactor));
	OutputUtils::printColoredText("absolute delta fot first ts: " + FormatUtils::to_string_with_precision(absDeltaFirstTS), *m_resultsFile);
	OutputUtils::printColoredText("absolute delta fot Last ts: " + FormatUtils::to_string_with_precision(absDeltaLastTS), *m_resultsFile);

	//provide a file with process summary: Succeeded or failed
	std::ofstream summaryFile;
	//check if the best delta average is too big
	if (m_bestDeltaDiffsAvg > m_maxError || m_bestDeltaDiffsAvg < m_minError)
	{
		OutputUtils::printColoredText("warning: the diffs average of the best delta is too big/small (" + FormatUtils::to_string_with_precision(m_bestDeltaDiffsAvg) + ")", *m_resultsFile, OutputUtils::WARNING_COLOR + OutputUtils::SHINING);
		summaryFile = std::ofstream(m_outFolderPath + "\\Failed!!!.txt");
		OutputUtils::printColoredText("Failed:(", OutputUtils::ERROR_COLOR + OutputUtils::SHINING);
	}
	else
	{
		summaryFile = std::ofstream(m_outFolderPath + "\\Succeeded!!!.txt");
		OutputUtils::printColoredText("Succeeded:)", OutputUtils::GREEN + OutputUtils::SHINING);
	}
}

void ITemporalAlignment::printProcessData()
{
	OutputUtils::printColoredText(getInformation(), *m_resultsFile, OutputUtils::TextColor::WHITE + OutputUtils::TextColor::SHINING);
}

int ITemporalAlignment::findNextClosestGtPose(double alg_ts, int gtStartIndx, double delta)
{
	int indx = gtStartIndx;

	while (indx < (m_gtData).size() && alg_ts >= (m_gtData[indx]->m_timeStamp) + delta)
	{
		indx++;
	}

	if (indx != gtStartIndx &&  indx != (m_gtData).size())//if we are not in the first gtline or the last one
	{
		if (std::abs(alg_ts - ((m_gtData[indx]->m_timeStamp) + delta)) <=
			std::abs(alg_ts - ((m_gtData[indx - 1]->m_timeStamp) + delta)))
			return indx;
		return indx - 1;
	}
	else
	{
		if (indx == (m_gtData).size())
			return indx - 1;
		else
			return indx;
	}
}

void ITemporalAlignment::changeGTTimeStamps()
{
	std::vector<IData*> alignedGtData;
	int framesNumber = m_gtData.size();

	for (int gtIndx = 0; gtIndx < framesNumber; gtIndx++)
	{
		IData* gt = m_gtData[gtIndx];
		gt->m_timeStamp = (gt->m_timeStamp) + m_bestDelta;

		//push to the aligned gt vector
		alignedGtData.push_back(gt);
	}
	m_alignedGtData = alignedGtData;
}

void ITemporalAlignment::interpolation()
{
	//read source file for interpolation only for the first time of interpolation (comparison of pointers), and only if this file is different than alg file
	if (m_interpolationDataPath != m_algPath && m_algDataType != m_interpolationDataType)
	{
		m_algDataType = m_interpolationDataType;
		readData(m_interpolationDataPath, ITemporalAlignment::ALG, m_interpolationObject);
	}
	std::vector<IData*> interpolatedData;
	int gtIndx = 0;

	for (int algIndx = 0; algIndx < m_algData.size(); algIndx++)
	{
		if (m_interpolationObject == TM2::ALL || m_algData[algIndx]->m_id == m_interpolationObject)//interpolate only for wanted object timestamps
		{
			int prevGtIndx = findPrevTimeStamp(algIndx, gtIndx);
			if (prevGtIndx < 0)
			{
				OutputUtils::printColoredText("Failed to find GT poses for interpolation", *m_resultsFile, OutputUtils::WARNING_COLOR);
				return;
			}

			//if we have the same time stamp in the gt
			if (m_algData[algIndx]->m_timeStamp == m_alignedGtData[prevGtIndx]->m_timeStamp)
			{
				interpolatedData.push_back(m_alignedGtData[prevGtIndx]);
			}
			else
			{
				//TODO:: look for closest next tracked pose
				int nextGtIndx = prevGtIndx + 1;
				IData* interpolatedGt;
				
				//set id for new IData, based the wanted object for interpolation
				int id;
				if (m_algData[algIndx]->m_id != IData::NOTYET)
					id = m_algData[algIndx]->m_id;
				else if (TM2::objects.count(m_interpolationFileSuffix) > 0) {
					// m_interpolation file suffix exist in objects map, can decide what object we interpolate for
					id = (TM2::object)TM2::objects.at(m_interpolationFileSuffix);
				}
				else
					id = TM2::HMD;

				switch (m_interpolation)
				{
				case IData::InterpolationType::LINEAR:
					interpolatedGt = m_alignedGtData[prevGtIndx]->linearInterpolation(m_alignedGtData[nextGtIndx], m_algData[algIndx]->m_timeStamp, id);
					break;
				case IData::InterpolationType::NEAREST:
					interpolatedGt = m_alignedGtData[prevGtIndx]->nearestInterpolation(m_alignedGtData[nextGtIndx], m_algData[algIndx]->m_timeStamp);
					break;
				}
				interpolatedData.push_back(interpolatedGt);
			}
			gtIndx = prevGtIndx <= 100 ? gtIndx : prevGtIndx - 100;
		}
	}
	m_interpolatedGtData = interpolatedData;
}

int ITemporalAlignment::findPrevTimeStamp(int algindx, int gtStartIndx)
{
	//TODO:: look for closest previos tracked pose
	int indx = gtStartIndx;

	while (indx < m_alignedGtData.size() && m_algData[algindx]->m_timeStamp >(m_alignedGtData[indx]->m_timeStamp))
	{
		indx++;
	}

	//if it's the same time stamp like the alg
	if (indx != m_alignedGtData.size() && m_algData[algindx]->m_timeStamp == (m_alignedGtData[indx]->m_timeStamp))
		return indx;

	//else- if the index is not in the begin or end of the vector
	if (indx < m_alignedGtData.size() - 1 && indx > 0)//if we are not in the first gt timestamp or the last one
	{
		return indx - 1;
	}
	return -1;
}

std::string ITemporalAlignment::getInformation()
{
	return "\n-----------------Alignment Information----------------\nGT data type is: " + m_gtDataType->getName()
		+ "\nOutput folder is: " + m_outFolderPath
		+ "\nGT file is: " + m_gtPath
		+ "\nGT fps is: " + std::to_string(m_fpsGT)
		+ "\nALG data type is: " + m_algDataType->getName()
		+ "\nALG file is: " + m_algPath
		+ "\nAlg fps is: " + std::to_string(m_algFPS)
		+ "\nComparison section length in frames number is: " + std::to_string(m_comparisonFramesNum)
		+ "\nComparison section length in seconds is: " + std::to_string((m_comparisonFramesNum * (1000 / m_algFPS)) / 1000)
		+ "\nStarts from index: " + std::to_string(m_srcStartIndex)
		+ "\nInitial delta (user): " + std::to_string(m_initialUserDelta)
		+ "\nAutomatic initial delta: " + std::to_string(initialDelta())
		+ "\nTotal: " + std::to_string(m_initialUserDelta + initialDelta())
		+ "\nAmplitude size for any direction in milliseconds: " + std::to_string(m_amplitudeSize_ms)
		+ "\nAlignment is based: " + TM2::to_string(m_algObject)
		+ "\n"
		+ "\n---Interpolation Information---"
		+ "\nloop all files in input folder: " + (loopFolder ? "true" : "false")
		+ "\nMethod is: " + IData::to_string(m_interpolation)
		+ "\nSource type is: " + m_interpolationDataType->getName()
		+ "\nSource path is: " + m_interpolationDataPath
		+ "\nInterpolation is based: " + TM2::to_string(m_interpolationObject)
		+ "\n"
		+ "\n------Clock Drift Compensation Information------"
		+ "\nInitial factor is: " + std::to_string(m_cd_compensator->m_initialFactor)
		+ "\nAmplitude size is: " + std::to_string(m_cd_compensator->m_amplitudeSize)
		+ "\nInitial Interval size is: " + std::to_string(m_cd_compensator->m_initialInterval)
		+ "\nClock drift compensation logger is: " + m_cd_compensator->m_outFileName
		+ "\n\n";
}

void ITemporalAlignment::findSpecificFactor(double initialValue, double interval)
{
	if (interval < m_cd_compensator->m_minInterval)
		return;
	double minAvg = DBL_MAX;
	int amplitudeSize = (interval == m_cd_compensator->m_initialInterval ? m_cd_compensator->m_amplitudeSize / 2 : m_cd_compensator->m_amplitudeSize);
	for (int i = -amplitudeSize; i <= amplitudeSize; i++)
	{
		double f = initialValue + (interval * i);
		m_cd_compensator->multiplyTimestampsByFactor(m_gtData, (f / m_cd_compensator->m_previousFactor));
		findSpecificDelta(initialDelta() + m_initialUserDelta, m_amplitudeSize_ms, BIG_JUMP);
		OutputUtils::printColoredText("clock_drift_factor: , " + FormatUtils::to_string_with_precision(f) + " , best_delta: , " + FormatUtils::to_string_with_precision(m_bestDelta) + " , diffs average: , " + FormatUtils::to_string_with_precision(m_bestDeltaDiffsAvg), *m_cd_compensator->m_outFile);
		m_cd_compensator->m_previousFactor = f;
		if (m_bestDeltaDiffsAvg < minAvg)
		{
			minAvg = m_bestDeltaDiffsAvg;
			m_cd_compensator->m_bestFactor = f;
		}
	}
	OutputUtils::printColoredText("\n", *m_cd_compensator->m_outFile);
	findSpecificFactor(m_cd_compensator->m_bestFactor, interval / m_cd_compensator->m_devisionStep);
}

void clockDriftCompensator::multiplyTimestampsByFactor(std::vector<IData*> &poses, double factor)
{	
	for each(auto &pose in poses)
	{
		pose->m_timeStamp *= factor;
	}
}
