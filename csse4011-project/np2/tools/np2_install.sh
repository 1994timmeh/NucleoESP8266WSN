sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi	#Install ARM GCC EABI compiler
sudo apt-get install dfu-util		#Install DFU progammer
sudo apt-get install ckermit		#Install Serial Terminal
sudo apt-get install libglib2.0 autoconf libtool libsdl-console libsdl-console-dev zlib1g-dev flex bison #dependencies for QEMU
sudo apt-get install gitg		#Git management
sudo apt-get remove modemmanager brltty	#Remove serial terminal interference
sudo adduser $USER dialout		#Add user permissions for dialout
sudo echo  source ~/np2git/np2/tools/np2_env.sh >> ~/.bashrc	#setup environment variables
