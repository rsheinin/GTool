@echo off
rem Y:
for /f "delims=" %%i in ('dir /b /a-d /od "\\edebby-perc\Brekel_OpenVR_Recorder\files\VR*.txt"') do set "LatestModifiedFile=%%~i"
echo copy "\\edebby-perc\Brekel_OpenVR_Recorder\files\%LatestModifiedFile%" "C:\Users\ntuser\Desktop\tests\dest"
copy "\\edebby-perc\Brekel_OpenVR_Recorder\files\%LatestModifiedFile%" "C:\Users\ntuser\Desktop\tests\dest"
rem & EXIT