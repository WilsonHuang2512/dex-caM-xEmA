#拉取依赖
wget -O 3rdparty.zip http://192.168.3.120:80/DexSense/dependencies/cameras/firmware/XEMA/3rdparty.zip

mkdir temp
unzip 3rdparty.zip -d temp
mkdir 3rdparty
mv temp/3rdparty/* ./3rdparty/
rm -r temp


#生成版本号
cd version
./build.sh

cd ../firmware 

rm -r build 
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

if [ -f "./camera_server" ]; then
    echo "✅ build success: camera_server generated."
     cp ./camera_server ../../3rdparty/update/ && \
        cd ../../3rdparty/update && \
        zip -r ../../firmware/build/camera_server.zip ./* && \
        rm -f ./camera_server
    echo "✅ camera_server.zip created."
else
    echo "❌ build failed: camera_server not found."
fi
