$simulatorMappingDir = Get-Location/..
cd ~
git clone https://github.com/microsoft/vcpkg
.\vcpkg\bootstrap-vcpkg.bat
.\vcpkg\vcpkg install cmake --triplet=x64-windows
cd "$simulatorMappingDir\ThirdParty\"
rm Pangolin
git clone https://github.com/tzukpolinsky/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..  -DCMAKE_TOOLCHAIN_FILE=~\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake
cmake --build . --target ALL_BUILD
cd ~
.\vcpkg\vcpkg install opencv3 --triplet=x64-windows
cd "$simulatorMappingDir\ThirdParty\DBoW2"
mkdir build
cd build
cmake ..  -DCMAKE_TOOLCHAIN_FILE=~\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake
cmake --build . --target ALL_BUILD



