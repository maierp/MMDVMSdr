// MMDVMSdr.cpp: Definiert den Einstiegspunkt für die Anwendung.
//

#include "Config.h"
#include "Globals.h"

#include "MMDVMSdr.h"

using namespace std;

// Global variables
MMDVM_STATE m_modemState = STATE_IDLE;

bool m_dstarEnable = true;
bool m_dmrEnable = true;
bool m_ysfEnable = true;
bool m_p25Enable = true;
bool m_nxdnEnable = true;
bool m_pocsagEnable = true;

bool m_duplex = true;

bool m_tx = false;
bool m_dcd = false;

//CDStarRX   dstarRX;
//CDStarTX   dstarTX;
//
//CDMRIdleRX dmrIdleRX;
//CDMRRX     dmrRX;
CDMRTX     dmrTX;

//CDMRDMORX  dmrDMORX;
//CDMRDMOTX  dmrDMOTX;
//
//CYSFRX     ysfRX;
//CYSFTX     ysfTX;
//
//CP25RX     p25RX;
//CP25TX     p25TX;
//
//CNXDNRX    nxdnRX;
//CNXDNTX    nxdnTX;
//
//CPOCSAGTX  pocsagTX;
//
//CCalDStarRX calDStarRX;
//CCalDStarTX calDStarTX;
//CCalDMR     calDMR;
//CCalP25     calP25;
//CCalNXDN    calNXDN;
//CCalPOCSAG  calPOCSAG;
//CCalRSSI    calRSSI;
//
//CCWIdTX cwIdTX;

CSerialPort serial;
CIO io;
CSDR sdr;

void loop()
{
	serial.process();

	io.process();

	// The following is for transmitting
	//if (m_dstarEnable && m_modemState == STATE_DSTAR)
	//	dstarTX.process();

	if (m_dmrEnable && m_modemState == STATE_DMR) {
		//std::cout << "dmrEnable:" << m_dmrEnable << " modemState:" << std::to_string(m_modemState) << std::endl;
		//if (m_duplex)
			dmrTX.process();
		//else
		//	dmrDMOTX.process();
	}

	//if (m_ysfEnable && m_modemState == STATE_YSF)
	//	ysfTX.process();

	//if (m_p25Enable && m_modemState == STATE_P25)
	//	p25TX.process();

	//if (m_nxdnEnable && m_modemState == STATE_NXDN)
	//	nxdnTX.process();

	//if (m_pocsagEnable && (m_modemState == STATE_POCSAG || pocsagTX.busy()))
	//	pocsagTX.process();

	//if (m_modemState == STATE_DSTARCAL)
	//	calDStarTX.process();

	//if (m_modemState == STATE_DMRCAL || m_modemState == STATE_LFCAL || m_modemState == STATE_DMRCAL1K || m_modemState == STATE_DMRDMO1K)
	//	calDMR.process();

	//if (m_modemState == STATE_P25CAL1K)
	//	calP25.process();

	//if (m_modemState == STATE_NXDNCAL1K)
	//	calNXDN.process();

	//if (m_modemState == STATE_POCSAGCAL)
	//	calPOCSAG.process();

	//if (m_modemState == STATE_IDLE)
	//	cwIdTX.process();
}

/***********************************************************************
 * main utility entry point
 **********************************************************************/
int main()
{
	cout << "MMDVM-SDR is starting" << endl;

	for (;;)
		loop();

	return 0;
}
