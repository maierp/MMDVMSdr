# MMDVMSdr

## Required libraries:
### LiquidSDR
https://liquidsdr.org/

`apt install libliquid-dev`

### SoapySDR
https://github.com/pothosware/SoapySDR/wiki

`apt install libsoapysdr-dev libsoapysdr0.6 soapysdr-module-all`

## Modify MMDVMHost
The original MMDVMHost application dies not work with virtual serial ports.
So we have to patch it (only a single line).

In file Modem.cpp change the last parameter of line 183 from `true` to `false`.

old:

~~~~
m_serial = new CSerialController(m_port, SERIAL_115200, true);
~~~~

new:

~~~~
m_serial = new CSerialController(m_port, SERIAL_115200, false);
~~~~

And compile MMDVMHost.

## Use MMDVMSdr with MMDVMHost
Start MMDVMSdr and note the created serial device file (`ptsname: /dev/pts/1`).

Use this device in the MMDVM.ini
~~~~
[Modem]
Port=/dev/pts/1
~~~~
