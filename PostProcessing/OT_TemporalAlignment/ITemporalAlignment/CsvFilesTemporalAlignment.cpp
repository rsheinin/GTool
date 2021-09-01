#include "CsvFilesTemporalAlignment.h"

void CsvFilesTemporalAlignment::readData(std::string path, Type fileType, TM2::object wantedObj)
{
	std::cout << "read data: path " << path << " file type: " << to_string(fileType) << std::endl;
	std::ifstream file;
	file.open(path);
	if (!file.is_open())
		OutputUtils::errorMessage("failed to open file: " + path + " of type: " + to_string(fileType), *m_resultsFile);
	std::string line;
	std::vector<IData*> fileData;

	if (fileType == ALG)
	{
		double sum = 0;
		m_algPath = path;
		if (m_algDataType->hasTitles())//if there is a titles line in alg file, skip this line
		{
			if (!std::getline(file, m_algHeader))
			{
				OutputUtils::errorMessage("can not read ALG file. path: " + path, *m_resultsFile);
			}
		}
		
		while (std::getline(file, line))
		{
			try
			{
				IData *lineData = m_algDataType->stringToData(line);
				if (lineData != NULL && fileData.size() > 0 && lineData->m_timeStamp == fileData.back()->m_timeStamp)//ignore duplicate pose
					fileData.pop_back();
				if (lineData != NULL && (wantedObj == TM2::ALL || lineData->m_id == wantedObj))//valid data and belongs to the wanted object 
				{
					fileData.push_back(lineData);
					if (fileData.size() > 1)
						sum += (fileData.back()->m_timeStamp - fileData[fileData.size() - 2]->m_timeStamp);
				}
			}
			catch (const std::exception&)
			{
				std::cout << "error" << std::endl;
			}
		}		
		if (fileData.size() > 0)
		{
			m_algFPS = fileData.size() / ((fileData.back()->m_timeStamp - fileData[0]->m_timeStamp) / 1000);
		}
		else
			m_algFPS = -1;
		m_algData = fileData;
	}

	else //if (fileType == GT)
	{
		double sum = 0;
		m_gtPath = path;
		if (m_gtDataType->hasTitles())//if there is a titles line in the alg file, skip this line
		{
			if (!std::getline(file, m_gtHeader))
			{
				OutputUtils::errorMessage("can not read GT file. path: " + path, *m_resultsFile);
			}
		}
		while (std::getline(file, line))
		{
			IData *lineData = m_gtDataType->stringToData(line);
			if (lineData != NULL && fileData.size() > 0 && lineData->m_timeStamp == fileData.back()->m_timeStamp)//ignore duplicate pose
				fileData.pop_back();
			if (lineData != NULL)
			{
				fileData.push_back(lineData);
				if (fileData.size() > 1)
					sum += (fileData.back()->m_timeStamp - fileData[fileData.size() - 2]->m_timeStamp);
			}
		}
		if (fileData.size() > 0)
		{
			m_fpsGT = fileData.size() / ((fileData.back()->m_timeStamp - fileData[0]->m_timeStamp) / 1000);
		}
		else
			m_fpsGT = -1;
		m_gtData = fileData;
	}
	file.close();
}

void CsvFilesTemporalAlignment::exportAlignedGtFile(std::string fileName)
{
	//export aligned gt file
	std::ofstream  *gtFile;
	std::string originalGtPath = m_gtPath;

	std::size_t gtFound = originalGtPath.find_last_of("/\\");
	std::string gtPath = originalGtPath.substr(0, gtFound);
	std::string gtName = TA_FILEPREFIX + fileName;

	gtFile = new std::ofstream(m_outFolderPath + "\\" + gtName);

	//print to file
	*gtFile << m_gtHeader << std::endl;

	for (int i = 0; i < m_alignedGtData.size(); i++)
	{
		*gtFile << m_alignedGtData[i]->dataToString();
	}

	gtFile->flush();
	gtFile->close();


	if (m_interpolation != IData::InterpolationType::NONE)
	{
		//export interpolated gt file
		std::ofstream  *gtInterFile;

		std::string gtInterName = INTER_FILEPREFIX + fileName;

		gtInterFile = new std::ofstream(m_outFolderPath + "\\" + gtInterName);

		//print to file
		*gtInterFile << m_gtHeader << std::endl;

		for (int i = 0; i < m_interpolatedGtData.size(); i++)
		{
			*gtInterFile << m_interpolatedGtData[i]->dataToString();
		}

		gtInterFile->flush();
		gtInterFile->close();
	}
}

void CsvFilesTemporalAlignment::exportAlignedFiles()
{

	if (loopFolder)
	{
		//find gt files paths
		std::string originalGtPath = m_gtPath;
		std::size_t gtFound = originalGtPath.find_last_of("/\\");
		std::string gtFolderPath = originalGtPath.substr(0, gtFound + 1);

		// loop in the folder and read all the gt files. 
		char search_path[200];
		sprintf_s(search_path, "%s/*%s", gtFolderPath.c_str(), m_gtDataType->getFileExtension().c_str());
		WIN32_FIND_DATAA fd;
		HANDLE hFind = FindFirstFileA(search_path, &fd);

		if (hFind != INVALID_HANDLE_VALUE)
		{
			do
			{
				if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
				{
					OutputUtils::printColoredText("---------------Interpolation: " + IData::to_string(m_interpolation) + std::string("-----------------"), OutputUtils::TextColor::LIGHTBLUE + OutputUtils::TextColor::SHINING);
					//update each file
					readData(gtFolderPath + "\\" + fd.cFileName, ITemporalAlignment::GT);
					m_cd_compensator->multiplyTimestampsByFactor(m_gtData, m_cd_compensator->m_bestFactor);
					changeGTTimeStamps();
					if (m_interpolation != IData::InterpolationType::NONE)
						interpolation();
					exportAlignedGtFile(fd.cFileName);
				}
			} while (FindNextFileA(hFind, &fd));
			FindClose(hFind);
		}
		else
			throw std::exception("hFind = INVALID_HANDLE_VALUE");
	}

	else
	{
		OutputUtils::printColoredText("---------------Interpolation: " + IData::to_string(m_interpolation) + std::string("-----------------"), OutputUtils::TextColor::LIGHTBLUE + OutputUtils::TextColor::SHINING);
		changeGTTimeStamps();
		if (m_interpolation != IData::InterpolationType::NONE)
			interpolation();
		exportAlignedGtFile(m_gtName + m_gtDataType->getFileExtension());
	}
}
