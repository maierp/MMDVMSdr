/*
 *   Copyright (C) 2019 by Patrick Maier DK5MP
 *   Copyright (C) 2015,2016,2017 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Colin Durbridge G4EML
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

#if !defined(DMRTX_H)
#define  DMRTX_H

#include <liquid/liquid.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "Config.h"
#include "DMRDefines.h"


enum DMRTXSTATE {
    DMRTXSTATE_IDLE,
    DMRTXSTATE_SLOT1,
    DMRTXSTATE_CACH1,
    DMRTXSTATE_SLOT2,
    DMRTXSTATE_CACH2,
    DMRTXSTATE_CAL
};

class CDMRTX {
public:
    CDMRTX();

    uint8_t writeData1(const uint8_t* data, uint8_t length);
    uint8_t writeData2(const uint8_t* data, uint8_t length);

    uint8_t writeShortLC(const uint8_t* data, uint8_t length);
    uint8_t writeAbort(const uint8_t* data, uint8_t length);

    void setStart(bool start);
    void setCal(bool start);

    void process();

    void resetFifo1();
    void resetFifo2();
    uint32_t getFrameCount();

    uint8_t getSpace1() const;
    uint8_t getSpace2() const;

    void setColorCode(uint8_t colorCode);

private:
    freqmod                          m_fmod;
    std::vector<int16_t>             m_sampleBuffer;
    std::queue<uint8_t>              m_fifo[2];
    std::vector<std::mutex>          m_fifoMutex;
    firinterp_rrrf                   m_rrc_interp_filter_obj;
    DMRTXSTATE                       m_state;
    uint8_t                          m_idle[DMR_FRAME_LENGTH_BYTES];
    uint8_t                          m_cachPtr;
    uint8_t                          m_shortLC[12U];
    uint8_t                          m_newShortLC[12U];
    uint8_t                          m_markBuffer[40U];
    uint8_t                          m_poBuffer[40U];
    uint16_t                         m_poLen;
    uint16_t                         m_poPtr;
    uint32_t                         m_frameCount;
    uint32_t                         m_abortCount[2U];
    bool                             m_abort[2U];
    std::condition_variable          m_DataAvailableConditionVariable;
    std::mutex                       m_DataAvailableMutex;

    void createData(uint8_t slotIndex);
    void createCACH(uint8_t txSlotIndex, uint8_t rxSlotIndex);
    void createCal();
    void writeByte(uint8_t c, uint8_t control);
    void readByte();
};

#endif
