# MMDVMSdr

## Required libraries:
### LiquidSDR
https://liquidsdr.org/

`apt install libliquid-dev`

### SoapySDR
https://github.com/pothosware/SoapySDR/wiki

`apt install libsoapysdr-dev libsoapysdr0.6 soapysdr-module-all`

## Modify MMDVMHost
The original MMDVMHost application does not work with virtual serial ports.
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
Upon start, MMDVMSdr creates a virtual serial port `/dev/pts/1` and a symbolic link to this `/dev/MMDVMSdr`.

Use this symbolic link `/dev/MMDVMSdr` in the MMDVM.ini
~~~~
[Modem]
Port=/dev/MMDVMSdr
~~~~
