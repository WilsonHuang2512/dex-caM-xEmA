cd version
./build.sh

cd ../sdk
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp libopen_cam3d_sdk.so ../../c_examples/example
cp libopen_cam3d_sdk.so ../../c_examples/param_json
cp ../open_cam3d_sdk.h ../../c_examples/param_json
cp ../open_cam3d_sdk.h ../../c_examples/example

cd ../../cpp
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp libxema_sdk.so ../../cpp_examples/cpp_example
cp libxema_sdk.so ../../cpp_examples/cpp_param_json
cp ../xcamera.h ../../cpp_examples/cpp_example
cp ../xcamera.h ../../cpp_examples/cpp_param_json

cd ../../cpp_examples/cpp_example
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cd ../../../cpp_examples/cpp_param_json
rm -r build
mkdir build
cd build 
cmake ..
make -j4


cd ../../../c_examples/example
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../param_json
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../../cmd
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
rm -rf release_camera
mkdir release_camera
cd release_camera
cp ../gui/build/bin -r ./
cp ../cmd/build/open_cam3d ./bin/
cp ../calibration/build/calibration ./bin/
cp ../c_examples/example/build/example ./bin/
cp ../test/build/open_cam3d_test ./bin/
cp ../tools/install/install.sh ./bin/
cp ../tools/install/xema_config.png ./bin/
cp ../tools/install/xema_logo.png ./bin/



