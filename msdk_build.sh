export MFX_HOME=`pwd`;
mkdir -p build
cd build
make -j8
sudo make -j8 install
