This folder contains the following scripts:

kermusb - automatically starts kermit, set to 115200 Baudrate.
	- to use, add the scripts folder to your PATH variable.
	1) Ppen a terminal
	2) Run gedit ~/.bashrc
 	3) Add the following line at the bottom of the file
	4) PATH=/home/csse3010/svn/np2/tools:$PATH
	5) Save file
	6) Close and open a new terminal
	7) Type kermusb, each time you want to use kermit

np2_env.sh - contains environment variables for using the NP2 source and kermusb. Should be included as "source /home/csse3010/np2/tools/np2_env.sh" in .bashrc profile.

np2_install.sh - installation script for ubuntu 12.04.

Makefile - creates STM32F4XX Peripheral Library documentation using doxygen
