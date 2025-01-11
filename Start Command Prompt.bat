@echo off
echo ===================================================================================
echo ==== Setup batch file for FRC2025 ====
echo =
echo    Usage:
echo    To deploy, "robotpy deploy" when you are in that same folder.
echo    To deploy, "robotpy --main src\<foldername>\robotname.py deploy" when you are in the FRC2025 folder.
echo = 
echo    For more details about updating and deploying, see the docs folder.
echo =
echo ===================================================================================

cd %USERPROFILE%\Desktop\FRC2025
poetry env activate > temp.bat
call temp.bat
del temp.bat
cmd /k
@echo on
