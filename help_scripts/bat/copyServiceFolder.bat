@echo off
setlocal enabledelayedexpansion
shift
set GToolFolder=%1
shift
set outFolder=%2
:: fake-command /gt %GToolFolder% /o %outFolder% 

set GToolPath="%GToolFolder%"
@echo GTool folder is: %GToolFolder%

set TIMESTAMP=%DATE:~10,4%-%DATE:~4,2%-%DATE:~7,2%-%TIME:~0,2%-%TIME:~3,2%-%TIME:~6,2%
@echo timestamp is: %TIMESTAMP%
set outFolderPath="%outFolder%\GTool_%TIMESTAMP%"
@echo out folder is: %outFolderPath%

:: optitrack recorder dll
xcopy %GToolPath%\Common\GTUtils\Release\OptitrackLibWrapper.dll %outFolderPath%\Common\GTUtils\Release\ /Y /I

:: scripts involved in process (convert brekel file format to GT file format)
xcopy %GToolPath%\help_scripts\python\convertBrekel2Tum.py %outFolderPath%\help_scripts\python\ /Y /I
xcopy %GToolPath%\3rdparty\python\transformations.py %outFolderPath%\3rdparty\python\ /Y /I

:: H&I calibration results for all rigid-bodies + calibration script
xcopy "%GToolPath%\H&Icalibration\GHCFiles\*" "%outFolderPath%\H&Icalibration\GHCFiles\" /Y /I
xcopy "%GToolPath%\H&Icalibration\GHC_Creator.py" "%outFolderPath%\H&Icalibration\" /Y /I

:: prediction files
xcopy %GToolPath%\Common\python\PosesReader\PosesReader.py %outFolderPath%\Common\python\PosesReader\ /Y /I
xcopy %GToolPath%\Common\python\PosesReader\KnownFormats\host_sdk.ini %outFolderPath%\Common\python\PosesReader\KnownFormats\ /Y /I
xcopy %GToolPath%\Common\python\PosesReader\KnownFormats\TUM.ini %outFolderPath%\Common\python\PosesReader\KnownFormats\ /Y /I
xcopy %GToolPath%\Common\python\PosesReader\KnownFormats\GT.ini %outFolderPath%\Common\python\PosesReader\KnownFormats\ /Y /I
xcopy %GToolPath%\PostProcessing\Prediction\predictPoses.py %outFolderPath%\PostProcessing\Prediction\ /Y /I 

:: temporal alignment files
xcopy %GToolPath%\PostProcessing\OT_TemporalAlignment\x64\Release\opencv_world310.dll %outFolderPath%\PostProcessing\OT_TemporalAlignment\x64\Release\ /Y /I
xcopy %GToolPath%\PostProcessing\OT_TemporalAlignment\x64\Release\OT_TemporalAlignment.exe %outFolderPath%\PostProcessing\OT_TemporalAlignment\x64\Release\ /Y /I

:: projector files
xcopy %GToolPath%\PostProcessing\Projector\x64\Release\opencv_world310.dll %outFolderPath%\PostProcessing\Projector\x64\Release\ /Y /I
xcopy %GToolPath%\PostProcessing\Projector\x64\Release\OT_Projector.exe %outFolderPath%\PostProcessing\Projector\x64\Release\ /Y /I

:: service files
xcopy %GToolPath%\RecordService\RecorderHost\bin\x86\Release\RecorderHost.exe %outFolderPath%\RecordService\RecorderHost\bin\x86\Release\ /Y /I
xcopy %GToolPath%\RecordService\RecorderHost\bin\x86\Release\RecorderService.dll %outFolderPath%\RecordService\RecorderHost\bin\x86\Release\ /Y /I
