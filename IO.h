/*
 *   Copyright (C) 2019 by Patrick Maier DK5MP
 *   Copyright (C) 2015,2016,2017,2018 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(IO_H)
#define  IO_H

#include <queue>
#include <utility>
#include <cstdlib>
#include <chrono>

#include "Globals.h"

//#include "SampleRB.h"
//#include "RSSIRB.h"

class CIO {
public:
    CIO();

    void process();

    void write(MMDVM_STATE mode, std::vector<int16_t> &samples, uint16_t length, const uint8_t* control = NULL);
    void read();

    uint16_t getSpace() const;

    void setDecode(bool dcd);
    void setADCDetection(bool detect);

    void interrupt();

    void setParameters(bool rxInvert, bool txInvert, bool pttInvert, uint8_t rxLevel, uint8_t cwIdTXLevel, uint8_t dstarTXLevel, uint8_t dmrTXLevel, uint8_t ysfTXLevel, uint8_t p25TXLevel, uint8_t nxdnTXLevel, uint8_t pocsagTXLevel, int16_t txDCOffset, int16_t rxDCOffset);

    void getOverflow(bool& adcOverflow, bool& dacOverflow);

    bool hasTXOverflow();
    bool hasRXOverflow();

    bool hasLockout() const;

    void resetWatchdog();
    std::chrono::steady_clock::duration getWatchdog();

    void selfTest();

private:
    std::queue<std::pair<uint16_t, uint8_t>> m_rxBuffer;
    std::queue<std::pair<uint16_t, uint8_t>> m_txBuffer;
    std::queue<uint16_t> m_rssiBuffer;


    //arm_biquad_casd_df1_inst_q31 m_dcFilter;
    //q31_t                        m_dcState[4];

    //arm_fir_instance_q15 m_rrcFilter;
    //arm_fir_instance_q15 m_gaussianFilter;
    //arm_fir_instance_q15 m_boxcarFilter;
    //arm_fir_instance_q15 m_nxdnFilter;
    //arm_fir_instance_q15 m_nxdnISincFilter;
    //int16_t                m_rrcState[70U];           // NoTaps + BlockSize - 1, 42 + 20 - 1 plus some spare
    //int16_t                m_gaussianState[40U];      // NoTaps + BlockSize - 1, 12 + 20 - 1 plus some spare
    //int16_t                m_boxcarState[30U];        // NoTaps + BlockSize - 1, 6 + 20 - 1 plus some spare
    //int16_t                m_nxdnState[110U];         // NoTaps + BlockSize - 1, 82 + 20 - 1 plus some spare
    //int16_t                m_nxdnISincState[60U];     // NoTaps + BlockSize - 1, 32 + 20 - 1 plus some spare

    bool                 m_pttInvert;
    int16_t                m_rxLevel;
    int16_t                m_cwIdTXLevel;
    int16_t                m_dstarTXLevel;
    int16_t                m_dmrTXLevel;
    int16_t                m_ysfTXLevel;
    int16_t                m_p25TXLevel;
    int16_t                m_nxdnTXLevel;
    int16_t                m_pocsagTXLevel;

    uint16_t             m_rxDCOffset;
    uint16_t             m_txDCOffset;

    uint32_t             m_ledCount;
    bool                 m_ledValue;

    bool                 m_detect;

    uint16_t             m_adcOverflow;
    uint16_t             m_dacOverflow;

    volatile uint32_t    m_watchdog;
    std::chrono::steady_clock::time_point m_timeout;

    bool                 m_lockout;

    // Hardware specific routines
    //void initInt();
    //void startInt();

    //bool getCOSInt();

    //void setLEDInt(bool on);
    //void setPTTInt(bool on);
    //void setCOSInt(bool on);

    //void setDStarInt(bool on);
    //void setDMRInt(bool on);
    //void setYSFInt(bool on);
    //void setP25Int(bool on);
    //void setNXDNInt(bool on);
    //void setPOCSAGInt(bool on);

    //void delayInt(unsigned int dly);
};

#endif
