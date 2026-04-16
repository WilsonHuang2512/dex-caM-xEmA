cd version
./build.sh

cd ../sdk
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp libopen_cam3d_sdk.so ../../example

cd ../../cpp
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp libxema_sdk.so ../../cpp_example

cd ../../cpp_example
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../example
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../cmd
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../calibration
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../test
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../gui
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp ../../tools/pack.sh ./bin
cp ../../tools/open_cam3d_gui.sh ./bin
cd ./bin
./pack.sh
rm pack.sh

cd ../../../
mkdir release_camera
cd release_camera
cp ../gui/build/bin -r ./
cp ../cmd/build/open_cam3d ./bin/
cp ../calibration/build/calibration ./bin/
cp ../example/build/example ./bin/
cp ../test/build/open_cam3d_test ./bin/
cp ../tools/install/install.sh ./bin/
cp ../tools/install/xema_config.png ./bin/
cp ../tools/install/xema_logo.png ./bin/



