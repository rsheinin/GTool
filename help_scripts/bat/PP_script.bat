@ECHO OFF
TITLE GTool script
ECHO this script will run temporal alignment and projector tools on a given folder
ECHO this script need to get 4 arguments: folder path, camera rb name, temporal alignment mode and markers-object name to be projected. && echo.for example: "C:\Users\ntuser\Documents\TFS\CVL\Tools\GTool\PostProcessing\OT_TemporalAlignment\without rendering", "DoubleDome", "color", "Table_RigidBody"
ECHO the script need to run from the the "PostProcessing" directory - the current directory where this script is located according to TFS
PAUSE

IF "%~1"=="" GOTO error REM first argument is missing
SET folder=%1
IF "%~2"=="" GOTO error REM second argument is missing
SET camera_rb=%2
IF "%~3"=="" GOTO error REM third argument is missing
SET temporal_mode=-%3
IF "%~4"=="" GOTO error REM fourth argument is missing
SET object=%4

ECHO folder path = %1
ECHO camera rigid-body name = %2
ECHO temporal alignment tool mode = %3
ECHO projected object = %4
REM PAUSE
GOTO continue

REM :parse
REM IF "%~1"=="" GOTO endparse REM finished to parse arguments
REM IF "%~1"=="-a" REM do something
REM IF "%~1"=="-b" REM do something else
REM SHIFT
REM GOTO parse
REM :endparse
REM REM ready for action!

:error
ECHO not enough arguments were passed to script. && echo.please specify the needed arguments as following: folder path, camera rb name, temporal alignment mode and markers-object name to be projected. && echo.for example: "C:\Users\ntuser\Documents\TFS\CVL\Tools\GTool\PostProcessing\OT_TemporalAlignment\without rendering", "DoubleDome", "color", "Table_RigidBody"
PAUSE
EXIT

:continue
CD OT_TemporalAlignment\x64\Release
set temporal_params=-folder %folder% -rb %camera_rb% %temporal_mode%
ECHO temporal tool parameters: %temporal_params%
REM PAUSE
START /WAIT "TemporalAlignment" "OT_TemporalAlignment.exe" %temporal_params%
REM PAUSE

CD ..\..\..\Projector\x64\Release
set projector_params=-folder %folder% -marker %object%
ECHO projector tool parameters: %projector_params%
REM PAUSE
START /WAIT "Projector" "OT_Projector.exe" %projector_params%
PAUSE