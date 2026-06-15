@echo off
set PATH=C:/cygwin64/bin;%PATH%
cd

dos2unix "./build.sh"
bash "./build.sh"
