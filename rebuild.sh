#/bin/bash
sudo make uninstall
make clean
make distclean
architecture=`uname -p`
./configure --prefix=/usr/lib/$architecture-linux-gnu/gstreamer-1.0/ --libdir=/usr/lib/$architecture-linux-gnu/
make -j4
sudo make install

