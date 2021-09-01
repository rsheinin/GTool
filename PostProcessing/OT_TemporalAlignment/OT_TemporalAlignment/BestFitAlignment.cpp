#include "BestFitAlignment.h"


std::string getFileSuffix(std::string path)
{
	return path.substr(path.find_last_of("_") + 1, path.find_last_of('.') - (path.find_last_of("_") + 1));
}

const double MagnitudeAlignment::startMovement = 0.7;

MagnitudeAlignment::MagnitudeAlignment(processArguments::procArgs params)
{
	std::string io_params, gt_params, ta_params, interp_params, cdc_params, other_params;
	if (params.count(processArguments::IO_PARAMS))
	{
		io_params = params.at(processArguments::IO_PARAMS);
		params.erase(processArguments::IO_PARAMS);
	}
	if (params.count(processArguments::GT_PARAMS))
	{
		gt_params = params.at(processArguments::GT_PARAMS);
		params.erase(processArguments::GT_PARAMS);
	}
	if (params.count(processArguments::ALIGNMENT_PARAMS))
	{
		ta_params = params.at(processArguments::ALIGNMENT_PARAMS);
		params.erase(processArguments::ALIGNMENT_PARAMS);
	}
	if (params.count(processArguments::INTERPOLATION_PARAMS))
	{
		interp_params = params.at(processArguments::INTERPOLATION_PARAMS);
		params.erase(processArguments::INTERPOLATION_PARAMS);
	}
	if (params.count(processArguments::CLOCKDRIFT_PARAMS))
	{
		cdc_params = params.at(processArguments::CLOCKDRIFT_PARAMS);
		params.erase(processArguments::CLOCKDRIFT_PARAMS);
	}
	if (params.count(processArguments::OTHER_PARAMS))
	{
		other_params = params.at(processArguments::OTHER_PARAMS);
		params.erase(processArguments::OTHER_PARAMS);
	}
	for each (auto var in params)
	{
		OutputUtils::printColoredText("Unknown argument: " + var.first, OutputUtils::WARNING_COLOR);
	}

	Init(io_params, gt_params, ta_params, interp_params, cdc_params, other_params);
}

MagnitudeAlignment::MagnitudeAlignment(std::string configFilePath)
{
	IOini::CIOini iniSettings;
	iniSettings.setFileName(configFilePath);
	std::string val;
	std::string section;

	section = "IO";
	std::string io_params = "";
	if (iniSettings.getKeyValue("in_folder_path", section, val, configFilePath))
	{
		io_params += val + paramsDelimiter;
		if (iniSettings.getKeyValue("out_folder_name", section, val, configFilePath))
			io_params += val + paramsDelimiter;
	}
	
	section = "GT";
	std::string gt_params = "";
	if (iniSettings.getKeyValue("gt_file_name", section, val, configFilePath))
	{
		gt_params += val + paramsDelimiter;
		if (iniSettings.getKeyValue("gt_type", section, val, configFilePath))
		{
			gt_params += val + paramsDelimiter;
			if (iniSettings.getKeyValue("object", section, val, configFilePath))
				gt_params += val + paramsDelimiter;
		}
	}

	section = "TemporalAlignment";
	std::string ta_params = "";
	if (iniSettings.getKeyValue("src_file_name", section, val, configFilePath))
	{
		ta_params += val + paramsDelimiter;
		if (iniSettings.getKeyValue("src_type", section, val, configFilePath))
		{
			ta_params += val + paramsDelimiter;
			if (iniSettings.getKeyValue("object", section, val, configFilePath))
			{
				ta_params += val + paramsDelimiter;
				if (iniSettings.getKeyValue("amplitude", section, val, configFilePath))
				{
					ta_params += val + paramsDelimiter;
					if (iniSettings.getKeyValue("inital_delta", section, val, configFilePath))
					{
						ta_params += val + paramsDelimiter;
						if (iniSettings.getKeyValue("comparison_section_length", section, val, configFilePath))
							ta_params += val + paramsDelimiter;
					}
				}
			}
		}
	}


	section = "Interpolation";
	std::string interp_params = "";
	if (iniSettings.getKeyValue("src_file_name", section, val, configFilePath))
	{
		interp_params += val + paramsDelimiter;
		if (iniSettings.getKeyValue("src_type", section, val, configFilePath))
		{
			interp_params += val + paramsDelimiter;
			if (iniSettings.getKeyValue("object", section, val, configFilePath))
			{
				interp_params += val + paramsDelimiter;
				if (iniSettings.getKeyValue("method", section, val, configFilePath))
					interp_params += val + paramsDelimiter;
			}
		}
	}
	
	section = "ClockDrift";
	std::string cdc_params = "";
	if (iniSettings.getKeyValue("amplitude", section, val, configFilePath))
	{
		cdc_params += val + paramsDelimiter;
		if (iniSettings.getKeyValue("initial_factor", section, val, configFilePath))
		{
			cdc_params += val + paramsDelimiter;
			if (iniSettings.getKeyValue("initial_interval", section, val, configFilePath))
			{
				cdc_params += val + paramsDelimiter;
				if (iniSettings.getKeyValue("devision_step", section, val, configFilePath))
				{
					cdc_params += val + paramsDelimiter;
					if (iniSettings.getKeyValue("levels_num", section, val, configFilePath))
						cdc_params += val + paramsDelimiter;
				}
			}
		}
	}

	section = "Other";
	std::string other_params = "";
	if (iniSettings.getKeyValue("loop_files_in_folder", section, val, configFilePath))
	{
		cdc_params += val + paramsDelimiter;
	}

	Init(io_params, gt_params, ta_params, interp_params, cdc_params, other_params);
}

double MagnitudeAlignment::calculateDifference(int algIndx, int gtNextIndx)
{
	OTData gt_data = *((OTData*)m_gtData[gtNextIndx]);
	
	if(gt_data.m_status != OTData::TRACKED || algIndx < 0 || gtNextIndx < 0)//current gt pose was untracked or index is invalid
		return NAN;
		
	if (ITemporalAlignment::m_currDelta != MagnitudeAlignment::tempDelta)//this is a new delta, clear all temp vectors
	{
		tempDelta = m_currDelta;
		src_vec.clear();
		gt_vec.clear();
	}
	
	if (gtNextIndx > 0)
	{
		OTData gt_prev_data = *(OTData*)m_gtData[gtNextIndx - 1];
		if (gt_prev_data.m_status != OTData::TRACKED)
			return NAN;
		IData* gt_interp_data = gt_prev_data.linearInterpolation((IData*)&gt_data, m_algData[algIndx]->m_timeStamp - m_currDelta, gt_data.m_id);
		
		auto q = ((OTData*)gt_interp_data)->m_quaternion;
		free(gt_interp_data);
		gt_vec.push_back(std::make_pair(q, gtNextIndx));
	}
	else
	{
		quaternion gt_pose = gt_data.m_quaternion;
		gt_vec.push_back(std::make_pair(gt_pose, gtNextIndx));
	}	
	
	quaternion src_pose;
	if (dynamic_cast<GyroData*>(m_algDataType))
	{
		GyroData* gyro_data = (GyroData*)m_algData[algIndx];
		src_pose = gyro_data->comulativeQuat;
		src_vec.push_back(std::make_pair(src_pose, algIndx));
	}
	else if (dynamic_cast<_6dofData*>(m_algDataType))
	{
		_6dofData* sixdof_data = (_6dofData*)m_algData[algIndx];
		src_pose = sixdof_data->m_quaternion;
		src_vec.push_back(std::make_pair(src_pose, algIndx));
	}
	if (gt_vec.size() <= m_slidingWindowSize)
		return NAN;

	quaternion gtQuat = QuaternionUtils::mutiply_inv(gt_vec[gt_vec.size() - m_slidingWindowSize].first, gt_vec.back().first);
	quaternion srcQuat = QuaternionUtils::mutiply_inv(src_vec[src_vec.size() - m_slidingWindowSize].first, src_pose);
	double gtMag = QuaternionUtils::magnitude(gtQuat);
	double srcMag = QuaternionUtils::magnitude(srcQuat);
	double diff = abs(srcMag - gtMag);

	if (gtMag == ERROR_CODE || srcMag == ERROR_CODE || std::isnan(diff))
		return NAN;

	return diff;
}

double MagnitudeAlignment::exactDelta(double orgDelta)
{
	std::cout << "I'm here!!" << std::endl;
	if (m_source == GYRO)
		return orgDelta + ((1000 / m_algFPS) / 2);
	return orgDelta;
}

void MagnitudeAlignment::setObject(std::string obj_str, TM2::object &object)
{
	if (TM2::objects.count(obj_str) > 0) {
		// object exist in objects map
		object = (TM2::object)TM2::objects.at(obj_str);
	}
	else
		object = TM2::ALL;
}

void MagnitudeAlignment::setType(std::string type_str, IData* &type)
{
	auto pose_type = srcTypes.find(type_str);
	if (pose_type != srcTypes.end()) //type_str exist in types list
		type = srcTypes.at(type_str);
	else
		type = srcTypes.at(OPTITRACK);//default
}

void MagnitudeAlignment::readData(std::string path, Type fileType, TM2::object obj)
{
	CsvFilesTemporalAlignment::readData(path, fileType, obj);
	if (fileType == ALG && m_algData.size() > 0)
		preProcess_src();
	else if (fileType == GT && m_gtData.size() > 0)
		int removeFrames = cleanupSpikesByTrace_GT(m_gtData);
}

bool MagnitudeAlignment::preProcess_src()
{
	if (m_algData.size() == 0)
		return false;

	double sectionLength_ms = m_algData.back()->m_timeStamp - m_algData[0]->m_timeStamp;

	quaternion refPose;
	if(m_source == SIXDOF)
		refPose = ((_6dofData*)m_algData[0])->m_quaternion;
	else if(m_source == GYRO)
		refPose = { 0,0,0,1 };

	double magnitude;
	//loop over source data objects vector and calculate the magnitude value for each one 
	for (size_t i = 1; i < m_algData.size(); i++)
	{
		double timeDiff = m_algData[i]->m_timeStamp - m_algData[i - 1]->m_timeStamp;
		if (m_source == SIXDOF)
		{
			magnitude = QuaternionUtils::magnitude(QuaternionUtils::mutiply_inv(refPose, ((_6dofData*)m_algData[i])->m_quaternion));
		}
		else if (m_source == GYRO)
		{
			EulerAngles gyroEuler;
			gyroEuler.w = EulOrdXYZr;
			double ts_diff_sec = timeDiff / 1000;
			gyroEuler.x = ((GyroData*)m_algData[i])->m_velocity[0] * ts_diff_sec;
			gyroEuler.y = ((GyroData*)m_algData[i])->m_velocity[1] * ts_diff_sec;
			gyroEuler.z = ((GyroData*)m_algData[i])->m_velocity[2] * ts_diff_sec;

			NN_Quat q = Eul_ToQuat(gyroEuler);
			quaternion gyro_q = { q.x, q.y, q.z, q.w };

			refPose = QuaternionUtils::mutiply(refPose, gyro_q);
			((GyroData*)m_algData[i])->comulativeQuat = refPose;
			magnitude = QuaternionUtils::magnitude(refPose);
		}
		//find the frame of start movement point
		if (m_srcStartIndex == 0 && magnitude > MagnitudeAlignment::startMovement)
			m_srcStartIndex = i;
	}
	
	if (m_srcStartIndex >= m_algData.size())//in case user asked for start index which is not exist, set start index to 0
		m_srcStartIndex = 0;
	
	m_srcStartIndex = m_srcStartIndex > m_algFPS ? m_srcStartIndex - m_algFPS : m_srcStartIndex / 2; //if possible: go back 1 sec before start movement and start compare from that point.
	
	float f = 1;
	double maxLength2compare_s = (m_algData.back()->m_timeStamp - m_algData[m_srcStartIndex]->m_timeStamp) / 1000;
	if ((m_compareSectionLength_ms/1000) > f * maxLength2compare_s)
		m_compareSectionLength_ms = (f * maxLength2compare_s) *1000;

	
	m_slidingWindowSize = m_algFPS;
	m_comparisonFramesNum = m_slidingWindowSize * (m_compareSectionLength_ms / 1000);
	
	return true;
}

//TODO: change to magnitude based quaternion instead of rotation mat trace
int MagnitudeAlignment::cleanupSpikesByTrace_GT(std::vector<IData*>& gtData)
{
	int count = 0;	

	auto resetFrame = [&](IData* pose) {
		count++;
		(*(OTData*)pose).m_status = OTData::Status::UNTRACKED;
		OTData gtPose = *(OTData*)pose;
	};

	auto empty = [&](IData* pose) {
		OTData gtPose = *(OTData*)pose;
		return !gtPose.m_status;
	};

	auto _trace = [&](IData* pose){
		OTData gtPose = *(OTData*)pose;
		double trace = gtPose.m_rotationsMat[0][0] + gtPose.m_rotationsMat[1][1] + gtPose.m_rotationsMat[2][2];
		return trace;
	};

	double thresholdValidGT = 1.0;
	unsigned long long timeDistThreshold = 1000000; //1 sec
	unsigned int index;
	int lastValidIndex, lastInvalidIndex = -1;
	double lastValidTrace, lastInvalidTrace;

	for (index = 0; index < gtData.size(); index++)
	{
		while (index < gtData.size() && empty(gtData[index]))index++;

		if (index == gtData.size())
			return 0;
		double trace_tmp = acos((_trace(gtData[index]) - 1) / 2);

		if (lastInvalidIndex != -1 && abs(lastInvalidTrace - trace_tmp) < thresholdValidGT)
		{

			resetFrame(gtData[index]);

			lastInvalidIndex = index;
			lastInvalidTrace = _trace(gtData[lastInvalidIndex]);
			//lastInvalidTrace = acos((cv::trace(allGtData[lastInvalidIndex].poseMat)[0] - 2) / 2);
		}

		if (lastInvalidIndex == -1 || abs(lastInvalidTrace - trace_tmp)  > thresholdValidGT)
		{
			lastValidIndex = index;
			lastValidTrace = _trace(gtData[lastValidIndex]);
			//lastValidTrace = acos((cv::trace(allGtData[lastValidIndex].poseMat)[0] - 2) / 2);
		}


		for (++index; index < gtData.size() && !empty(gtData[index]); index++)
		{
			double trace = _trace(gtData[index]);
			//double trace = acos((cv::trace(allGtData[index].poseMat)[0] - 2) / 2);
			if (abs(trace - lastValidTrace) < thresholdValidGT)
			{
				lastValidIndex = index;
				lastValidTrace = trace;
			}
			else
			{
				resetFrame(gtData[index]);

				lastInvalidIndex = index;
				lastInvalidTrace = _trace(gtData[lastInvalidIndex]);
				//lastInvalidTrace = acos((cv::trace(allGtData[lastInvalidIndex].poseMat)[0] - 2) / 2);
			}
		}
	}

	OutputUtils::printColoredText(std::to_string(count) + " frames were removed from GT repo by cleaning up spikes", *m_resultsFile, OutputUtils::WARNING_COLOR + OutputUtils::SHINING);
	return count;
}

bool MagnitudeAlignment::Init(std::string io_params, std::string gt_params, std::string ta_params, std::string interp_params, std::string cdc_params, std::string other_params)
{
	auto warningMessage = [&](std::string param, std::string default) {
		OutputUtils::printColoredText("parameter " + param + " has not been set by user choice. Default value is: " + default, OutputUtils::GRAY);
	};
	auto errorMessage = [&](std::string param) {
		OutputUtils::errorMessage("parameter " + param + " is required!");
	};
	auto feedbackMessage = [&](std::string param, std::string choice) {
		OutputUtils::printColoredText("parameter " + param + " has been set to: " + choice, OutputUtils::GREEN + OutputUtils::SHINING);
	};
	
	std::vector<std::string> params;

	OutputUtils::printColoredText("--------------------Initializing Parameters------------------", OutputUtils::LIGHTBLUE + OutputUtils::SHINING);
	
	try
	{
		//IO
		std::cout << processArguments::IO_PARAMS << std::endl;
		params = StringUtils::str2tokens(io_params, paramsDelimiter);
		std::string folder;
		if (params.size() > 0)
		{
			folder = params[0];
			feedbackMessage("input folder path", folder);
		}
		else
			errorMessage("input folder path");
		if (params.size() > 1)
		{
			m_outFolderName = params[1];
			feedbackMessage("output folder", m_outFolderName);
		}
		else
			warningMessage("output folder", "automatically");
		params.clear();

		//GT
		std::cout << processArguments::GT_PARAMS << std::endl;
		params = StringUtils::str2tokens(gt_params, paramsDelimiter);
		std::string gt_name;
		if (params.size() > 0)
		{
			gt_name = params[0];
			feedbackMessage("GT file name", gt_name);
		}
		else
			errorMessage("GT file name");
		if (params.size() > 1)
		{
			setType(params[1], m_gtDataType);
			feedbackMessage("GT type", m_gtDataType->getName());
		}
		else
		{
			setType(OPTITRACK, m_gtDataType);
			warningMessage("GT type", m_gtDataType->getName());
		}
		if (params.size() > 2)
		{
			setObject(params[2], m_gtObject);
			feedbackMessage("GT object", TM2::to_string(m_gtObject));
		}
		else
		{
			setObject("all", m_gtObject);
			warningMessage("GT object", TM2::to_string(m_gtObject));
		}
		params.clear();

		//TA
		std::cout << processArguments::ALIGNMENT_PARAMS << std::endl;
		params = StringUtils::str2tokens(ta_params, paramsDelimiter);
		std::string src_name;
		if (params.size() > 0)
		{
			src_name = params[0];
			feedbackMessage("source file name", src_name);
		}
		else
			errorMessage("source file name");
		if (params.size() > 1)
		{
			setType(params[1], m_algDataType);
			feedbackMessage("source type", m_algDataType->getName());
		}
		else
		{
			setType(SIXDOF, m_algDataType);
			warningMessage("source type", m_algDataType->getName());
		}
		if (params.size() > 2)
		{
			setObject(params[2], m_algObject);
			feedbackMessage("source object", TM2::to_string(m_algObject));
		}
		else
		{
			setObject(TM2::to_string(TM2::HMD), m_algObject);
			warningMessage("source object", TM2::to_string(m_algObject));
		}
		if (params.size() > 3)
		{
			m_amplitudeSize_ms = stoi(params[3]) * 1000;//convert from seconds to milliseconds;
			feedbackMessage("amplitude size in ms", std::to_string(m_amplitudeSize_ms));
		}
		else
			warningMessage("length of comparison section in ms", std::to_string(m_compareSectionLength_ms));
		if (params.size() > 4)
		{
			m_initialUserDelta = stoi(params[4]) * 1000;//convert from seconds to milliseconds;
			feedbackMessage("initial delta in ms", std::to_string(m_initialUserDelta));
		}
		else
			warningMessage("initial delta in ms", std::to_string(m_initialUserDelta));
		if (params.size() > 5)
		{
			m_compareSectionLength_ms = stoi(params[5]) * 1000;//convert from seconds to milliseconds;
			feedbackMessage("length of comparison section in ms", std::to_string(m_compareSectionLength_ms));
			
			
		}
		else
			warningMessage("amplitude size in ms", std::to_string(m_amplitudeSize_ms));
		params.clear();

		//INTERPOLATION
		std::cout << processArguments::INTERPOLATION_PARAMS << std::endl;
		params = StringUtils::str2tokens(interp_params, paramsDelimiter);
		std::string interp_src_name;
		if (params.size() > 0)
			interp_src_name = params[0];
		else
			errorMessage("source file name for interpolation");
		if (params.size() > 1)
		{	
			setType(params[1], m_interpolationDataType);
			feedbackMessage("interpolation type", m_interpolationDataType->getName());
		}
		else
		{
			setType(SIXDOF, m_interpolationDataType);
			warningMessage("interpolation type", m_interpolationDataType->getName());
		}
		if (params.size() > 2)
		{
			setObject(params[2], m_interpolationObject);
			feedbackMessage("source object of interpolation", std::to_string(m_interpolationObject));
		}
		else
		{
			setObject(TM2::to_string(TM2::HMD), m_interpolationObject);
			warningMessage("source object of interpolation", std::to_string(m_interpolationObject));
		}
		if (params.size() > 3)
		{
			std::string method = params[3];
			if (method == IData::to_string(IData::InterpolationType::LINEAR))
				m_interpolation = IData::InterpolationType::LINEAR;
			else if (method == IData::to_string(IData::InterpolationType::NEAREST))
				m_interpolation = IData::InterpolationType::NEAREST;
			else if (method == IData::to_string(IData::InterpolationType::NONE))
				m_interpolation = IData::InterpolationType::NONE;
			feedbackMessage("interpolation method", IData::to_string(m_interpolation));
		}
		else
		{
			m_interpolation = IData::InterpolationType::LINEAR;
			warningMessage("interpolation method", IData::to_string(m_interpolation));
		}
		params.clear();

		//CLOCK DRIFT COMPENSATION
		std::cout << processArguments::CLOCKDRIFT_PARAMS << std::endl;
		params = StringUtils::str2tokens(cdc_params, paramsDelimiter);
		if (params.size() > 0)
		{
			m_cd_compensator->m_amplitudeSize = stod(params[0]);
			feedbackMessage("amplitude size of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_amplitudeSize));
		}
		else
			warningMessage("amplitude size of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_amplitudeSize));
		if (params.size() > 1)
		{
			m_cd_compensator->m_initialFactor = stod(params[1]);
			feedbackMessage("initial factor of clock drift compensation", std::to_string(m_cd_compensator->m_initialFactor));
		}
		else
			warningMessage("initial factor of clock drift compensation", std::to_string(m_cd_compensator->m_initialFactor));
		if (params.size() > 2)
		{
			m_cd_compensator->m_initialInterval = stod(params[2]);
			feedbackMessage("initial interval of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_initialInterval));
		}
		else
			warningMessage("initial interval of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_initialInterval));
		if (params.size() > 3)
		{
			m_cd_compensator->m_devisionStep = stod(params[3]);
			feedbackMessage("devision step in searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_devisionStep));
		}
		else
			warningMessage("devision step in searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_devisionStep));
		if (params.size() > 4)
		{
			m_cd_compensator->m_minInterval = m_cd_compensator->m_initialInterval / pow(m_cd_compensator->m_devisionStep, (stod(params[4]) - 1));
			feedbackMessage("initial factor of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_minInterval));
		}
		else
			warningMessage("initial factor of searching for clock drift compensation best factor", std::to_string(m_cd_compensator->m_minInterval));
		params.clear();

		//OTHER
		std::cout << processArguments::OTHER_PARAMS << std::endl;
		params = StringUtils::str2tokens(other_params, paramsDelimiter);
		if (params.size() > 0)
		{
			std::istringstream(params[0]) >> std::boolalpha >> loopFolder;
			feedbackMessage("loop folder for all csv files", std::to_string(loopFolder));
		}
		else
			warningMessage("loop folder for all csv files", std::to_string(loopFolder));

		params.clear();

		if (folder.empty() || src_name.empty() || gt_name.empty() || interp_src_name.empty())
			OutputUtils::errorMessage("Required arguments are missing. Exiting");

		m_algPath = m_algDataType->getInnerDefPath(folder, &src_name);
		m_gtPath = m_gtDataType->getInnerDefPath(folder, &gt_name);
		m_interpolationDataPath = m_interpolationDataType->getInnerDefPath(folder, &interp_src_name);
	}
	catch (const std::exception& e)
	{
		OutputUtils::errorMessage("Failed to parse arguments: " + std::string(e.what()) + " Exiting");
	}

	std::cout << std::endl;
	OutputUtils::printColoredText("--------------------End------------------", OutputUtils::LIGHTBLUE + OutputUtils::SHINING);

	return true;
}

cv::Mat MagnitudeAlignment::rotMat2RelativeRotMat(cv::Mat curr, cv::Mat prev)
{
	return prev.inv() * curr;
}

double MagnitudeAlignment::magnitude(cv::Mat rotation)
{
	if (rotation.size().width == IData::rotMatSize && rotation.size().height == IData::rotMatSize)
	{
		double trace = rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2);
		double upperLimit = 3.00001;
		double lowerLimit = -1.000009;

		if (trace > 3 && trace < upperLimit)
			trace = 3;
		if (trace < -1 && trace > lowerLimit)
			trace = -1;
		if (trace < -1 || trace > 3)
		{
			OutputUtils::printColoredText("invalid trace: " + FormatUtils::to_string_with_precision(trace), OutputUtils::WARNING_COLOR);
			return ERROR_CODE;
		}
		return acos((trace - 1) / 2);
	}
	return ERROR_CODE;
}


//gyro class

IData* GyroData::stringToData(std::string str)
{
	GyroData* pose = new GyroData();
	std::vector<std::string> row = OTData::getNextLineAndSplitIntoTokens(str);
	
	if (row.size() != V_SIZE + 1)//velocity+timestamp
		return NULL;

	pose->m_timeStamp = std::stod(row[0]);
	pose->m_velocity[0] = std::stod(row[1]);
	pose->m_velocity[1] = std::stod(row[2]);
	pose->m_velocity[2] = std::stod(row[3]);

	//for testing: adding scaling and bias factors (positive bias)
	pose->m_velocity[0] = pose->m_velocity[0] * 1.000102459568873 + 0.002296852872423916;
	pose->m_velocity[0] = pose->m_velocity[0] * 0.9983255754187433 - 0.004078792517149523;
	pose->m_velocity[0] = pose->m_velocity[0] * 0.9985571106325701 + 0.0001688109594681406;
	
	return pose;
}

std::string GyroData::getInnerDefPath(std::string folder, void *srcName)
{
	if ((*((std::string*)srcName)).empty())
	{
		char search_path[200];
		sprintf_s(search_path, "%s\\*extract*", folder.c_str());
		WIN32_FIND_DATAA fd;
		HANDLE hFind = FindFirstFileA(search_path, &fd);
		if (hFind != INVALID_HANDLE_VALUE)
		{
			return folder + "\\" + fd.cFileName + "\\" + this->getName() + this->getFileExtension();
		}
		return std::string();
	}
	return folder + "\\" + *((std::string*)srcName) + this->getFileExtension();
}

std::string GyroData::dataToString()
{
	return std::to_string(m_timeStamp) + "," + FormatUtils::to_string_with_precision(m_velocity[0]) + "," + FormatUtils::to_string_with_precision(m_velocity[1]) + "," + FormatUtils::to_string_with_precision(m_velocity[2]) + "," + FormatUtils::to_string_with_precision(m_magnitude);
}

IData * GyroData::linearInterpolation(IData * next, double timeStamp, int src_id)
{
	return nullptr;
}

std::string GyroData::getName()
{
	return GYRO;
}

bool GyroData::hasTitles()
{
	return false;
}


//_6dofData class

IData* _6dofData::stringToData(std::string str)
{
	_6dofData* pose = new _6dofData();
	std::vector<std::string> row = OTData::getNextLineAndSplitIntoTokens(str, ' ');

	if (row.size() < _6dofData::POSESIZE - 1)//invalid row from 6dof file
		return NULL;

	if(std::stod(row[_6dofData::TIMESTAMP]) == 0)//ignore lines of zeros in 6-dof file
		return NULL;

	pose->m_timeStamp = std::stod(row[_6dofData::TIMESTAMP]) * 1000;//convert from sec unit to ms unit
	
	try
	{
		if (std::isnan(std::stod(row[_6dofData::TX])))//nan pose in 6dof file
			return NULL;
	}
	catch (const std::exception&)
	{
		return NULL;
	}

	//for controllers only
	/*if (row.size() >= _6dofData::POSESIZE)
		if (std::stoi(row[_6dofData::IDENTIFIER]) == 0)
			return NULL;*/

	pose->m_translation[0] = std::stod(row[_6dofData::TX]) * 1000;//convert from meters unit to milimeters unit
	pose->m_translation[1] = std::stod(row[_6dofData::TY]) * 1000;//convert from meters unit to milimeters unit
	pose->m_translation[2] = std::stod(row[_6dofData::TZ]) * 1000;//convert from meters unit to milimeters unit

	pose->m_quaternion[0] = std::stod(row[_6dofData::QX]);
	pose->m_quaternion[1] = std::stod(row[_6dofData::QY]);
	pose->m_quaternion[2] = std::stod(row[_6dofData::QZ]);
	pose->m_quaternion[3] = std::stod(row[_6dofData::QW]);

	//if there is an identifier
	if (row.size() > IDENTIFIER)
		pose->m_id = std::stoi(row[_6dofData::IDENTIFIER]);

	return pose;
}

std::string _6dofData::getInnerDefPath(std::string folder, void *srcName)
{
	if ((*((std::string*)srcName)).empty())
	{
		char search_path[200];
		sprintf_s(search_path, "%s\\*%s", folder.c_str(), this->getFileExtension().c_str());
		WIN32_FIND_DATAA fd;
		HANDLE hFind = FindFirstFileA(search_path, &fd);
		std::string prevFile = "";
		if (hFind == INVALID_HANDLE_VALUE)
		{
			OutputUtils::printColoredText("Can not find a " + this->getName() + " file. (Searching for path: " + std::string(search_path) + ")");
			return "";
		}
		while ((std::string(fd.cFileName).find("rotmat_for_comp") != std::string::npos
			|| std::string(fd.cFileName).find("ot_gt") != std::string::npos
			|| std::string(fd.cFileName).find("ot_pose") != std::string::npos)
			&& fd.cFileName != prevFile)
		{
			prevFile = fd.cFileName;
			FindNextFileA(hFind, &fd);
		}
		if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && fd.cFileName != prevFile)
		{
			return folder + "\\" + fd.cFileName;
		}
		return std::string();
	}
	return folder + "\\" + *((std::string*)srcName) + this->getFileExtension();
}

std::string _6dofData::dataToString()
{
	return std::to_string(m_timeStamp) + "," 
		+ FormatUtils::to_string_with_precision(m_translation[0]) + "," + FormatUtils::to_string_with_precision(m_translation[1]) + "," + FormatUtils::to_string_with_precision(m_translation[2]) + ","
		+ FormatUtils::to_string_with_precision(m_quaternion[0]) + "," + FormatUtils::to_string_with_precision(m_quaternion[1]) + "," + FormatUtils::to_string_with_precision(m_quaternion[2]) + "," + FormatUtils::to_string_with_precision(m_quaternion[3]) + std::to_string(m_id);
}

IData * _6dofData::linearInterpolation(IData * next, double timeStamp, int src_id)
{
	return nullptr;
}

std::string _6dofData::getName()
{
	return SIXDOF;
}

bool _6dofData::hasTitles()
{
	return false;
}
