# MMDVMSdr

## Required libraries:
### LiquidSDR
https://liquidsdr.org/

`apt install libliquid-dev`

### SoapySDR
https://github.com/pothosware/SoapySDR/wiki

`apt install libsoapysdr-dev libsoapysdr0.6 soapysdr-module-all`

## Use MMDVMSdr with MMDVMHost
Start MMDVMSdr and note the created serial device file (`ptsname: /dev/pts/1`).

Use this device in the MMDVM.ini
~~~~
[Modem]
Port=/dev/pts/1
~~~~