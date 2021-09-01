#pragma once

#include <string>
#include <Windows.h>
#include <atlstr.h>

#define MAX_ARR 2000

static wchar_t* charArrayToLPCWSTR(const char* charArray)
{
	wchar_t* wString = new wchar_t[4096];
	MultiByteToWideChar(CP_ACP, 0, charArray, -1, wString, 4096);
	return wString;
}

namespace IOini
{
	class CIOini
	{
	private:
		std::string m_strFileName;

	public:

		bool getKeyValue(std::string key, std::string section, std::string& value, std::string filePath)
		{
			wchar_t cResult[MAX_ARR];
			auto nResult = GetPrivateProfileString(charArrayToLPCWSTR(section.c_str()), charArrayToLPCWSTR(key.c_str()),
			L"", cResult, MAX_ARR, charArrayToLPCWSTR(filePath.c_str()));

			if (nResult == 0 || GetLastError() == ERROR_FILE_NOT_FOUND)
			{
				return false;
			}

			std::wstring val(cResult);
			value = CW2A(cResult);
		
			return true;
		}

		bool setKey(std::string key, std::string section, std::string value)
		{
			auto res = WritePrivateProfileString(charArrayToLPCWSTR(section.c_str()), charArrayToLPCWSTR(key.c_str()), charArrayToLPCWSTR(value.c_str()), charArrayToLPCWSTR(m_strFileName.c_str()));

			if (res == 0)
			{
				return false;
			}

			return true;
		}


		void setFileName(std::string filename)
		{
			char fullPathBuffer[4096];
			GetFullPathName(charArrayToLPCWSTR(filename.c_str()), 4096, charArrayToLPCWSTR(fullPathBuffer), NULL);
			m_strFileName = std::string(fullPathBuffer);
		}

		std::string getFileName();



	};
}