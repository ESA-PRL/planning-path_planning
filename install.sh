sudo apt-get install -y python-matplotlib cmake

install_folder=$PWD/install
cur=$PWD

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$install_folder -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -DCMAKE_CXX_FLAGS=-std=c++11 ..
make

cd $cur

rm env.sh

echo "export PATH=$PATH:$cur/build" >> env.sh
echo "DyMu project has been successfully installed"
echo "A Global Path Planning example can be executed by sourcing env.sh and typing runGPPtest"
