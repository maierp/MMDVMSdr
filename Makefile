ifdef USENCURSES
LIBNCURSES = -lncurses
DEFS_NCURSES=-DUSENCURSES
endif
CC      = cc
CXX     = c++
CFLAGS  = -g -O3 -Wall -std=c++0x $(DEFS_NCURSES)
LIBS    = -lSoapySDR -lliquid -lpthread $(LIBNCURSES)
LDFLAGS = -g

OBJECTS =	MMDVMSdr.o SerialPort.o IO.o DMRTX.o DMRSlotType.o SDR.o Debug.o

all:		MMDVMSdr

sdr_test:
		$(CXX) test_mod_demod.cpp $(CFLAGS) $(LIBS) -o test_mod_demod
		$(CXX) test_sync.cpp $(CFLAGS) $(LIBS) -o test_sync

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

