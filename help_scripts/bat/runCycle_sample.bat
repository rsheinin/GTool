@echo off
setlocal ENABLEDELAYEDEXPANSION

set "root_folder=Z:\forChavyLopiansky\21_12_2017_calibration"
set "rb_name=ES3_20_12"

for /f "tokens=*" %%A in ('dir /b /a:d "%root_folder%\*"') do (

	for /f "tokens=*" %%B in ('dir /b /a:d "%root_folder%\%%A\*"') do (
	
		for /f "tokens=*" %%C in ('dir /b /a:d "%root_folder%\%%A\%%B\*"') do (
			
			set "src_folder=%root_folder%\%%A\%%B\%%C"
			echo src folder is: !src_folder!
			
			rem delete previous results of temporal alignmnet and projection
			rd !src_folder!\dev_6dof_temporalAlignment_%rb_name% /Q /S
			rd !src_folder!\val_projection /Q /S
			rd !src_folder!\projection /Q /S
			rem pause
			
			rem prepare destination folder by cleaning up old files			
			set "dst_folder=C:\Users\ntuser\Desktop\projection_ww_50"
			echo dest folder is: !dst_folder!
			cd !dst_folder!
			del * /S /Q
			rd !dst_folder!\dev_6dof_temporalAlignment_%rb_name% /Q
			rd !dst_folder!\projection /Q
			rem pause
			
			rem copy files to a local directory to improve performance
			xcopy !src_folder!\%rb_name%.csv !dst_folder!\ /Y /I
			xcopy !src_folder!\%%C.txt !dst_folder!\ /Y /I
			rem pause
			
			rem run temporal alignment + interpolation
			set "ta_command=-io !dst_folder!,temporalAlignment -gt %rb_name%,ot -ta %%C,hmd,6dof,300,0,30 -interp %%C,hmd,6dof,linear -cdc 1.000035,0.001,10,4,0 -other false"
			echo ta_command is !ta_command!
			C:\Users\ntuser\Documents\GIT\GT_Team\PostProcessing\OT_TemporalAlignment\x64\Release\OT_TemporalAlignment.exe -io !dst_folder!,temporalAlignment -gt %rb_name%,ot -ta %%C,hmd,6dof,300,0,30 -interp %%C,hmd,6dof,linear -cdc 1,0.001,10,4,9 -other false
			rem pause
			
			rem run projection
			set "projection_command=-folder  !dst_folder! -o projection -head %rb_namer% -src %%C -ta temporalAlignment -hmd -ghc %rb_namer%,"0.97350593620929 0.07057043903590 -0.21749943746951 1.01455733523530 -0.00160027747617 0.95326357581627 0.30213572138027 -1.82659707421186 0.22865614200644 -0.29378285885371 0.92812068211331 -19.63909917847560""
			echo projection_command is !projection_command!
			C:\Users\ntuser\Documents\GIT\GT_Team\PostProcessing\Projector\x64\Release\OT_Projector.exe !projection_command!
			rem pause
			
			rem copy results back to source folder
			xcopy !dst_folder!\projection\* !src_folder!\projection\ /Y /I
			xcopy !dst_folder!\temporalAlignment\* !src_folder!\temporalAlignment\ /Y /I
			xcopy !dst_folder!\%%C.rc.tum !src_folder!\ /Y /I			
			rem pause
			
		) 
	)   
)