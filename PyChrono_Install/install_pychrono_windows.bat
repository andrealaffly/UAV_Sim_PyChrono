@echo off
setlocal


:: Edison Melendez
:: Date: 7/18/25

:: === SETUP ===
set ENV_NAME=chrono
set PYTHON_VERSION=3.10
set CHRONO_TARBALL=pychrono-8.0.0-py310_0.tar.bz2
set DOWNLOAD_DIR=%USERPROFILE%\Downloads
set FILE_PATH=%DOWNLOAD_DIR%\%CHRONO_TARBALL%

echo.
echo [1/5] Adding conda-forge channel...
call conda config --add channels http://conda.anaconda.org/conda-forge

echo.
echo [2/5] Creating Conda environment...
call conda create -y -n %ENV_NAME% python=%PYTHON_VERSION%

echo.
echo [3/5] Installing dependencies into %ENV_NAME%...
call conda install -y -n %ENV_NAME% -c conda-forge numpy=1.24.0 matplotlib irrlicht=1.8.5 pytz scipy

echo.
echo [4/5] Checking for manually downloaded tarball...
if not exist "%FILE_PATH%" (
    echo  ERROR: PyChrono tarball not found in:
    echo     %FILE_PATH%
    echo Please download it from conda-forge and place it in your Downloads folder.
    exit /b 1
)

echo.
echo [5/5] Installing PyChrono tarball into %ENV_NAME%...
call conda install -y -n %ENV_NAME% "%FILE_PATH%"

echo.
echo  All done!
echo To activate the environment manually, run:
echo     conda activate %ENV_NAME%

endlocal
pause
