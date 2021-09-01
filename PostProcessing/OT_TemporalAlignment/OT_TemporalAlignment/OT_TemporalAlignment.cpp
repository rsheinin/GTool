// OT_TemporalAlignment.cpp : Defines the entry point for the console application.
//

#include <Windows.h>
#include <iostream>
#include <direct.h>
#include <conio.h>
#include "BestFitAlignment.h"


#pragma once

#define NOTYET -1

processArguments::procArgs args;
std::string configFile = "";

void printHelp()
{
	std::cout << "Usage:" << std::endl;
	std::cout << "-config <path to config file in ini format>" << std::endl;
	
	TCHAR full_path[MAX_PATH];
	std::string rel_path_to_sample_config_file = "..\\processArguments_sample.ini";
	std::wstring abs_path_to_sample_config_file = std::wstring(rel_path_to_sample_config_file.begin(), rel_path_to_sample_config_file.end());
	GetFullPathName(abs_path_to_sample_config_file.c_str(), MAX_PATH, full_path, NULL);
	std::ifstream f(full_path);

	if (f.is_open())
	{
		std::cout << std::endl <<"-----------Config File Sample------------" << std::endl;
		std::cout << f.rdbuf();
		f.close();
	}
	
}

int main(int argc, char ** argv)
{
	OutputUtils::printColoredText("------------------Temporal Alignment-----------------", OutputUtils::TextColor::PINK + OutputUtils::TextColor::SHINING);
	
	MagnitudeAlignment* ma = NULL;

	try
	{
		//read user arguments
		for (int i = 1; i < argc; i++)
		{
			if (strcmp(argv[i], "-config") == 0)
			{
				if(argc > i + 1)
					configFile = std::string(argv[++i]);
			}
			if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0)
			{
				printHelp();
				exit(0);
			}
			else
			{
				if (argc > i + 1)
				{
					std::string key = std::string(argv[i]).erase(0,1);//remove the '-' character
					std::string val = std::string(argv[++i]);
					args.insert(std::make_pair(key, val));
				}
			}
		}
	}
	catch (...)
	{
		OutputUtils::errorMessage("Invalid line of arguments. Exiting");
	}
	
	if (!configFile.empty())
		ma = new MagnitudeAlignment(configFile);
	else
		ma = new MagnitudeAlignment(args);
	double res = ma->temporalAlignment();
	std::cout << res << std::endl;
	
	return 0;
}
