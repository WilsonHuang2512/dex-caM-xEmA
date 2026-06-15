echo off

:: 下载 3rdparty.zip
curl -o 3rdparty.zip http://192.168.3.120:80/DexSense/dependencies/cameras/win/XEMA/3rdparty_v1.4.zip

if not exist "temp_extract" mkdir "temp_extract"

:: 解压到临时目录
powershell -Command "Expand-Archive -Path '3rdparty.zip' -DestinationPath 'temp_extract' -Force"

:: 创建 XEMA\3rdparty 目录
if not exist "3rdparty" mkdir "3rdparty"

:: 移动解压内容到 XEMA\3rdparty
robocopy temp_extract\3rdparty 3rdparty /E /MOVE

rmdir temp_extract
del 3rdparty.zip

cd version
call build.bat

:: 进入 cpp 文件夹
cd ..

:: 创建 build 目录
if not exist build_all mkdir build_all

:: 进入 build 目录
cd build_all

:: 运行 CMake 配置
cmake ..

:: 编译 Release
cmake --build . --config Release
cmake --install . --config Release

cd ../cpp

python neutral.py

cd ..

if not exist XEMA_Release mkdir XEMA_Release

cd XEMA_Release

::创建三种SDK包
if not exist XEMA_neutral mkdir XEMA_neutral
if not exist XEMA_core mkdir XEMA_core
if not exist XEMA_pickwiz mkdir XEMA_pickwiz
if not exist XEMA_unit_test mkdir XEMA_unit_test

::开始XEMA_neutral的文件夹创建打包
cd XEMA_neutral
if not exist lib mkdir lib
if not exist GUI mkdir GUI 
if not exist doc mkdir doc
if not exist example mkdir example
cd lib 
if not exist cpp mkdir cpp
if not exist python mkdir python
if not exist c# mkdir c# 
cd ../example
if not exist cpp_example mkdir cpp_example
if not exist python_example mkdir python_example
if not exist c#_example mkdir c#_example

::开始XEMA_core的文件夹创建打包
cd ../..
cd XEMA_core
if not exist lib mkdir lib
if not exist GUI mkdir GUI 
if not exist doc mkdir doc
if not exist example mkdir example
cd lib 
if not exist cpp mkdir cpp
if not exist python mkdir python
if not exist c# mkdir c# 
cd ../example
if not exist cpp_example mkdir cpp_example
if not exist python_example mkdir python_example
if not exist c#_example mkdir c#_example

::开始XEMA_pickwiz的文件夹创建打包
cd ../..
cd XEMA_pickwiz
if not exist lib mkdir lib
if not exist example mkdir example

::开始文件copy
copy /Y "%~dp0\build_all\install\lib\xema_core.lib" "%~dp0XEMA_Release\XEMA_core\lib\cpp\"
copy /Y "%~dp0\build_all\install\bin\xema_core.dll" "%~dp0XEMA_Release\XEMA_core\lib\cpp\"
copy /Y "%~dp0\build_all\install\lib\xema_core.lib" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\build_all\install\bin\xema_core.dll" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\cpp\xcamera.h" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\firmware\camera_param.h" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.cpp" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.filters" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.user" "%~dp0XEMA_Release\XEMA_core\example\cpp_example\"
::开始文件copy，其他的工具到GUI文件夹
robocopy %~dp0\build_all\install\bin\ %~dp0XEMA_Release\XEMA_core\GUI\ /E

copy /Y "%~dp0\build_all\install\lib\camera.lib" "%~dp0XEMA_Release\XEMA_neutral\lib\cpp\"
copy /Y "%~dp0\build_all\install\bin\camera.dll" "%~dp0XEMA_Release\XEMA_neutral\lib\cpp\"
copy /Y "%~dp0\build_all\install\lib\camera.lib" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\build_all\install\bin\camera.dll" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\cpp\xcamera_neutral.h" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\firmware\camera_param.h" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.cpp" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.filters" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.user" "%~dp0XEMA_Release\XEMA_neutral\example\cpp_example\"

copy /Y "%~dp0\build_all\install\lib\xemacpp.lib" "%~dp0XEMA_Release\XEMA_pickwiz\lib\"
copy /Y "%~dp0\build_all\install\bin\xemacpp.dll" "%~dp0XEMA_Release\XEMA_pickwiz\lib\"
copy /Y "%~dp0\build_all\install\lib\xemacpp.lib" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\build_all\install\bin\xemacpp.dll" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\cpp\xcamera.h" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\firmware\camera_param.h" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.cpp" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.filters" "%~dp0XEMA_Release\XEMA_pickwiz\example\"
copy /Y "%~dp0\cpp_examples\cpp_example\cpp_example.vcxproj.user" "%~dp0XEMA_Release\XEMA_pickwiz\example\"

copy /Y "%~dp0build_all\install\bin\xemacpp.dll" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp03rdparty\ConfiguringIP\enumerate.lib" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp03rdparty\ConfiguringIP\enumerate.dll" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp03rdparty\ConfiguringIP\configuring_ip_info.cfg" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp03rdparty\tbb_2021_12_shared_md\tbb12.dll" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp0build_all\install\bin\utest_xema.exe" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp0build_all\install\config\utest_xema_config.json" "%~dp0XEMA_Release\XEMA_unit_test\"
copy /Y "%~dp0tools\xema_update_firmware.bat" "%~dp0XEMA_Release\XEMA_unit_test\"

::修改部分文件
cd /d "%~dp0"
set "CPP_FILE=XEMA_Release\XEMA_neutral\example\cpp_example\cpp_example.cpp"
set "VCXPROJ_FILE=XEMA_Release\XEMA_neutral\example\cpp_example\cpp_example.vcxproj"
powershell -Command "(Get-Content '%CPP_FILE%') -replace '#include \"xcamera.h\"', '#include \"xcamera_neutral.h\"' -replace 'XemaEngine', 'Engine' -replace 'XemaColor', 'Color' -replace 'XEMA', 'CAMERA' | Set-Content '%CPP_FILE%'"
powershell -Command "(Get-Content '%VCXPROJ_FILE%') -replace 'xemacpp.lib', 'camera.lib' | Set-Content '%VCXPROJ_FILE%'"

cd /d "%~dp0"
set "VCXPROJ_FILE=XEMA_Release\XEMA_core\example\cpp_example\cpp_example.vcxproj"
powershell -Command "(Get-Content '%VCXPROJ_FILE%') -replace 'xemacpp.lib', 'XEMA_core.lib' | Set-Content '%VCXPROJ_FILE%'"

cd /d "%~dp0"
set "VCXPROJ_FILE=XEMA_Release\XEMA_pickwiz\example\cpp_example.vcxproj"
powershell -Command "(Get-Content '%VCXPROJ_FILE%') -replace 'NDEBUG;', 'NDEBUG;USE_OPENCV;' | Set-Content '%VCXPROJ_FILE%'"


cd /d "%~dp0"
echo wait 10 seconds for file copy to complete...
powershell -Command "Start-Sleep -Seconds 10"
::打包
powershell -Command "Compress-Archive -Path 'XEMA_Release\*' -DestinationPath 'XEMA_Release.zip' -Force"


echo finish
