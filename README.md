Plug in power and USB
```
cd PATH_TO_WS/src/novint_ros/extern/libnifalcon
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
make install
sudo cp ../linux/40-novint-falcon-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Unplug USB and plug device in again
```
sudo ./bin/findfalcons
```
You may have to run this twice. You should see the LED change and encoder feedback work.

If the arm won't calibrate, there may be something wrong with the wall power. Check the Falcon is plugged in correctly.

