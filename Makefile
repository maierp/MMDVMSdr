CC      = cc
CXX     = c++
CFLAGS  = -g -O3 -Wall -std=c++0x
LIBS    = -lSoapySDR -lliquid -lpthread -lncurses
LDFLAGS = -g

OBJECTS =	MMDVMSdr.o SerialPort.o IO.o DMRTX.o DMRSlotType.o SDR.o Debug.o

all:		MMDVMSdr

MMDVMSdr:	GitVersion.h $(OBJECTS) 
		$(CXX) $(OBJECTS) $(CFLAGS) $(LIBS) -o MMDVMSdr

%.o: %.cpp
		$(CXX) $(CFLAGS) -c -o $@ $<

clean:
		$(RM) MMDVMSdr *.o *.d *.bak *~ GitVersion.h

# Export the current git version if the index file exists, else 000...
GitVersion.h:
ifneq ("$(wildcard .git/index)","")
	echo "#define GITVERSION \"$(shell git rev-parse HEAD)\"" > $@
else
	echo "#define GITVERSION \"0000000000000000000000000000000000000000\"" > $@
endif

