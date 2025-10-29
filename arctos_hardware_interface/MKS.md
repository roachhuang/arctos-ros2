sudo apt-get install can-utils -y
ls /dev/serial/by-id

// to bridge canable device to a virtual socket can network interface.
sudo slcan -o -c -s6 /dev/ttyACM1 can0
