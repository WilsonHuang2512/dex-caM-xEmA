@echo off
@echo Start update...

if "%~1"=="" (
    echo Usage: %~nx0 [ip] [firmware path]
    echo Example: %~nx0 192.168.15.29 .\
    exit /b 1
)

if "%~2"=="" (
    echo Usage: %~nx0 [ip] [firmware path]
    echo Example: %~nx0 192.168.15.29 .\
    exit /b 1
)

set ip=%~1
set fpath=%~2

if not exist "%fpath%\camera_server" (
    echo Error: firmware file not found: "%fpath%\camera_server"
    exit /b 1
)

set pathTmpFile=pathTmp.txt
set errTmpFile=errTmp.txt
set pathFile=path.txt
set errFile=err.txt

del /f /s /q %pathTmpFile% %errTmpFile% %pathFile% %errFile% > nul

echo.
echo Auto-accepting SSH host key...
echo y | plink -ssh dexforce@%ip% -pw dexforce "exit" > nul 2>&1

echo.

powershell -c "$runPath = $(plink -ssh dexforce@%ip% -pw dexforce 'echo dexforce | sudo -S readlink /proc/$(pidof camera_server)/cwd'); if ($runPath -notlike '') {echo $runPath > %pathTmpFile%} else {echo 'The camera_server is not running, please check!' > %errTmpFile%}"

if exist %pathTmpFile% (
    PowerShell -C "get-content %pathTmpFile% -encoding unicode | set-content %pathFile% -encoding string"
    for /f "tokens=*" %%p in (%pathFile%) do (
        set runPath=%%p
    )
)

if exist %errTmpFile% (
    PowerShell -C "get-content %errTmpFile% -encoding unicode | set-content %errFile% -encoding string"
    for /f "tokens=*" %%e in (%errFile%) do (
        echo %%e
        pause > nul
        Exit
    )
)

del /f /s /q %pathTmpFile% %errTmpFile% %pathFile% %errFile% > nul

echo camera_server running path: %runPath%
timeout /T 1 /NOBREAK > nul

echo.
@echo 1. Terminate the camera_server process...
plink -ssh dexforce@%ip% -pw dexforce "echo dexforce | sudo -S kill -9 $(pidof camera_server)"
timeout /T 1 /NOBREAK > nul

echo.
@echo 2. Delete the camera_server firmware...
plink -ssh dexforce@%ip% -pw dexforce "echo dexforce | sudo -S rm -f %runPath%/camera_server"
timeout /T 1 /NOBREAK > nul

echo.
@echo 3. Copy the update camera_server into device...
pscp -l dexforce -pw dexforce %fpath%\camera_server %ip%:%runPath%
timeout /T 1 /NOBREAK > nul

echo.
@echo 4. Add the executable permission...
plink -ssh dexforce@%ip% -pw dexforce "chmod +x %runPath%/camera_server"
timeout /T 1 /NOBREAK > nul

echo.
@echo 5. Reboot the device...
plink -ssh dexforce@%ip% -pw dexforce "echo dexforce | sudo -S reboot"

echo.
@echo Update finished. Wait for the device to reboot...
