install_folder=$PWD/install
cur=$PWD

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur

