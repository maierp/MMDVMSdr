/*
 *   Copyright (C) 2019 by Patrick Maier DK5MP
 *
 *   This program is free software : you can redistribute itand /or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.If not, see < http://www.gnu.org/licenses/>.
 *
 */

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

	//if (m_dmrEnable && m_modemState == STATE_DMR) {
	//	//std::cout << "dmrEnable:" << m_dmrEnable << " modemState:" << std::to_string(m_modemState) << std::endl;
	//	//if (m_duplex)
	//		dmrTX.process();
	//	//else
	//	//	dmrDMOTX.process();
	//}

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

void dmrThreadTXProcess()
{
	while (true)
	{
		if (m_dmrEnable && m_modemState == STATE_DMR) {
			dmrTX.process();
		}
	}
}

void dmrThreadRXProcess()
{
	//while (true)
	//{
	//	dmrRX.process();
	//}
}

/***********************************************************************
 * main utility entry point
 **********************************************************************/
int main()
{
	cout << "MMDVM-SDR is starting" << endl;

	std::thread dmrThread(dmrThreadTXProcess);
	for (;;)
		loop();

	return 0;
}
