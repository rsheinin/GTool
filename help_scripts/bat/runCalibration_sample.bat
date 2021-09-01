@echo off
setlocal enabledelayedexpansion

set calib_script_path="C:\Users\ntuser\Documents\GIT\GT_Team\H&Icalibration\GHC_Creator_YS.py"
set "clip=camera_calibration_"
set "folder=Z:\forChavyLopiansky\21_12_2017_calibration\ES3_21_12_calibration\calibration_clips\%clip%"

set RBname=ES3_20_12
set x=-3.5
set y=4.1
set z=-19

FOR /L %%N IN (1,1,5) DO (
	set "inner_folder=%folder%%%N"
	echo folder is: !inner_folder!
	set "TA_6dof_folder=temporalAlignment"
	xcopy "!inner_folder!\!TA_6dof_folder!\INTER_%RBname%.csv" "!inner_folder!\6dof_calibration\" /Y /I
	xcopy "!inner_folder!\%clip%%%N.txt" "!inner_folder!\6dof_calibration\" /Y /I
	python %calib_script_path% --folderPath "!inner_folder!\6dof_calibration" --gtFile INTER_%RBname%.csv --poseFile %clip%%%N.txt --txyz %x% %y% %z%	
	pause
)

