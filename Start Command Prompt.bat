:: Batch file to change to the proper folder and set up the environment.
:: This file assumes that python poetry is on your computer and you have already
:: run "poetry install" to create a virtual environment for the project.
:: The FRC2025 repository should be on your desktop.
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

:: Change to the proper directory, the git repository for the FRC2025 code.
cd %USERPROFILE%\Desktop\FRC2025

:: Activate the virtual environment set up for robotpy.
call .\.venv\Scripts\activate.bat

:: Stop in a command prompt.
cmd /k
@echo on
