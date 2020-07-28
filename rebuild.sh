#/bin/bash
sudo make uninstall
make clean
make distclean
./configure --prefix=/usr/lib/x86_64-linux-gnu/gstreamer-1.0/ --libdir=/usr/lib/x86_64-linux-gnu/
make -j4 
sudo make install
