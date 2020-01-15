#!/bin/bash
sudo apt-get -y install git
sudo apt-get -y install python3-scipy python3-pip
sudo apt-get -y install cmake
yes | sudo pip3 install pyserial PyQt5==5.10 shapely zeroconf trimesh
yes | sudo pip3 uninstall sip
cd sip-4.19.20
python3 configure.py
make
sudo make install
cd ..
#---------------------------------------------------
#--------------install protobuf---------------------
#---------------------------------------------------
cd ./protobuf
mkdir build-dir 
cd build-dir
cmake ../cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_CXX_FLAGS=-fPIC
echo "CMAKE_CXX_FLAGS:FLAGS=-fPIC">>CMakeCache.txt
make
sudo make install

cd ../..
#---------------------------------------------------
#--------------install Arcus------------------------
#---------------------------------------------------
cd libArcus
mkdir build 
cd build
cmake ..
make -j4
sudo make install
sudo ln -s /usr/local/lib/python3/dist-packages/Arcus.so /usr/local/lib/python3.6/dist-packages/Arcus.so

cd ../..
#---------------------------------------------------
#--------------install Savitar----------------------
#---------------------------------------------------
cd libSavitar
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$VIRTUAL_ENV -DCMAKE_PREFIX_PATH=$VIRTUAL_ENV -DBUILD_STATIC=ON -DBUILD_PYTHON=ON -DBUILD_EXAMPLES=OFF ..
make -j4
sudo make install

cd ../..
#---------------------------------------------------
#--------------install Charon-----------------------
#---------------------------------------------------
cd libCharon
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$VIRTUAL_ENV -DCMAKE_PREFIX_PATH=$VIRTUAL_ENV ..
make -j4
pip3 install ..

cd ../..
#---------------------------------------------------
#--------------install CuraEngine-------------------
#---------------------------------------------------
cd CuraEngine
mkdir build 
cd build
cmake ..
make -j4
sudo make install

cd ../..
#---------------------------------------------------
#--------------install Cura & Uranium---------------
#---------------------------------------------------
cd Cura
ln -s /usr/local/bin/CuraEngine .
mkdir resources/materials
cd ..
ln -s `pwd`/fdm_materials Cura/resources/materials/fdm_materials

echo "export PYTHONPATH=`pwd`/Cura:`pwd`/Uranium:${PYTHONPATH}" >> ~/.bashrc
echo "alias cura=\"cd `pwd`/Cura && python3 `pwd`/Cura/cura_app.py\"" >> ~/.bashrc