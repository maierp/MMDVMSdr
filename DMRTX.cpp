/*
 *   Copyright (C) 2019 by Patrick Maier DK5MP
 *   Copyright (C) 2009-2017 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2017 by Andy Uribe CA6JAU
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

#include <algorithm>
#include <iterator>
#include <iostream>
#include <thread>
#include <chrono>

#include "DMRTX.h"
#include "Globals.h"
#include "DMRSlotType.h"

const float DMR_MAX_FREQ_DEV = 1944; // 1.944 kHz | 648 Hz | -648 Hz | -1.944 kHz
const double SAMPLERATE = 255 * 4800; //4800 symbols/s with 255 samples/symbol

const float DMR_SYMBOL_A = 1.0f;
const float DMR_SYMBOL_B = 1.0f / 3.0f;
const float DMR_SYMBOL_C = -1.0f / 3.0f;
const float DMR_SYMBOL_D = -1.0f;

#define DATA_TIMEOUT 150

// The PR FILL and BS Data Sync pattern.
const uint8_t IDLE_DATA[] =
{ 0x53U, 0xC2U, 0x5EU, 0xABU, 0xA8U, 0x67U, 0x1DU, 0xC7U, 0x38U, 0x3BU, 0xD9U,
 0x36U, 0x00U, 0x0DU, 0xFFU, 0x57U, 0xD7U, 0x5DU, 0xF5U, 0xD0U, 0x03U, 0xF6U,
 0xE4U, 0x65U, 0x17U, 0x1BU, 0x48U, 0xCAU, 0x6DU, 0x4FU, 0xC6U, 0x10U, 0xB4U };

const uint8_t CACH_INTERLEAVE[] =
{ 1U,  2U,  3U,  5U,  6U,  7U,  9U, 10U, 11U, 13U, 15U, 16U, 17U, 19U, 20U, 21U, 23U,
 25U, 26U, 27U, 29U, 30U, 31U, 33U, 34U, 35U, 37U, 39U, 40U, 41U, 43U, 44U, 45U, 47U,
 49U, 50U, 51U, 53U, 54U, 55U, 57U, 58U, 59U, 61U, 63U, 64U, 65U, 67U, 68U, 69U, 71U,
 73U, 74U, 75U, 77U, 78U, 79U, 81U, 82U, 83U, 85U, 87U, 88U, 89U, 91U, 92U, 93U, 95U };

const uint8_t EMPTY_SHORT_LC[] =
{ 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U };

const uint8_t BIT_MASK_TABLE[] = { 0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U };

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
#define READ_BIT1(p,i)    (p[(i)>>3] & BIT_MASK_TABLE[(i)&7])

const uint32_t STARTUP_COUNT = 10U; //20U
const uint32_t ABORT_COUNT = 3U;// 6U;

CDMRTX::CDMRTX() :
    m_fmod(freqmod_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */)),
    m_sampleBuffer(2 * 1020),
    m_fifo(),
    m_fifoMutex(2),
    m_state(DMRTXSTATE_IDLE),
    m_idle(),
    m_cachPtr(0U),
    m_shortLC(),
    m_newShortLC(),
    m_markBuffer(),
    m_poBuffer(),
    m_poLen(0U),
    m_poPtr(0U),
    m_frameCount(0U),
    m_abortCount(),
    m_abort()
{
    m_rrc_interp_filter_obj = firinterp_rrrf_create_prototype(LIQUID_FIRFILT_RRC, 5, 4, 0.2, 0); // 4 Symbols with 5 interpolation samples each

    std::copy(std::begin(EMPTY_SHORT_LC), std::end(EMPTY_SHORT_LC), std::begin(m_newShortLC));
    std::copy(std::begin(EMPTY_SHORT_LC), std::end(EMPTY_SHORT_LC), std::begin(m_shortLC));

    m_lastDataSeen[0] = m_lastDataSeen[1] = std::chrono::steady_clock::now();

    m_abort[0U] = false;
    m_abort[1U] = false;

    m_abortCount[0U] = 0U;
    m_abortCount[1U] = 0U;
}

void CDMRTX::process()
{
    if (m_state == DMRTXSTATE_IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
    }

    if (m_poLen == 0U) {
        //Wait for enough Data to process
        std::chrono::steady_clock::time_point now;
        std::unique_lock<std::mutex> lock(m_DataAvailableMutex);
        bool waitForData = true;
        while (waitForData)
        {
            bool enoughData1, enoughData2;
            {
                std::unique_lock<std::mutex> fifoGuard1(m_fifoMutex[0], std::defer_lock);
                std::unique_lock<std::mutex> fifoGuard2(m_fifoMutex[1], std::defer_lock);
                std::lock(fifoGuard1, fifoGuard2);
                enoughData1 = m_fifo[0U].size() >= DMR_FRAME_LENGTH_BYTES;
                enoughData2 = m_fifo[1U].size() >= DMR_FRAME_LENGTH_BYTES;
            }
            now = std::chrono::steady_clock::now();
            if ((enoughData1 || std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastDataSeen[0]).count() >= DATA_TIMEOUT)
             && (enoughData2 || std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastDataSeen[1]).count() >= DATA_TIMEOUT))
            {
                waitForData = false;
            }
            else
            {
                m_DataAvailableConditionVariable.wait(lock);
            }
        }
        
        sdr.setStreamState(true); // Turn on SDR if not already
        switch (m_state) {
        case DMRTXSTATE_SLOT1:
            createData(0U);
            m_state = DMRTXSTATE_CACH2;
            break;

        case DMRTXSTATE_CACH2:
            createCACH(1U, 0U);
            m_state = DMRTXSTATE_SLOT2;
            break;

        case DMRTXSTATE_SLOT2:
            createData(1U);
            m_state = DMRTXSTATE_CACH1;
            break;

        case DMRTXSTATE_CAL:
            createCal();
            break;

        default:
            createCACH(0U, 1U);
            m_state = DMRTXSTATE_SLOT1;
            break;
        }
    }

    if (m_poLen > 0U) {
        while (m_poPtr < m_poLen) {
            uint8_t c = m_poBuffer[m_poPtr];
            uint8_t m = m_markBuffer[m_poPtr];
            m_poPtr++;
            writeByte(c, m);
            //readByte();
        }
        m_poPtr = 0U;
        m_poLen = 0U;
    }
}

uint8_t CDMRTX::writeData1(const uint8_t* data, uint8_t length)
{
    if (length != (DMR_FRAME_LENGTH_BYTES + 1U))
        return 4U;

    if (m_abort[0U]) {
        std::lock_guard<std::mutex> lock(m_fifoMutex[0]);
        std::queue<uint8_t>().swap(m_fifo[0U]); //clear the buffer
        m_abort[0U] = false;
    }

    {
        std::lock_guard<std::mutex> lock(m_fifoMutex[0]);
        for (uint8_t i = 0U; i < DMR_FRAME_LENGTH_BYTES; i++)
            m_fifo[0U].push(data[i + 1U]);
        m_lastDataSeen[0] = std::chrono::steady_clock::now();
    }

    if (m_state == DMRTXSTATE_IDLE)
    {
        m_state = DMRTXSTATE_SLOT1;
    }

    m_DataAvailableConditionVariable.notify_one();
    return 0U;
}

uint8_t CDMRTX::writeData2(const uint8_t* data, uint8_t length)
{
    if (length != (DMR_FRAME_LENGTH_BYTES + 1U))
        return 4U;

    if (m_abort[1U]) {
        std::lock_guard<std::mutex> lock(m_fifoMutex[1]);
        std::queue<uint8_t>().swap(m_fifo[1U]); //clear the buffer
        m_abort[1U] = false;
    }

    {
        std::lock_guard<std::mutex> lock(m_fifoMutex[1]);
        for (uint8_t i = 0U; i < DMR_FRAME_LENGTH_BYTES; i++)
            m_fifo[1U].push(data[i + 1U]);
        m_lastDataSeen[1] = std::chrono::steady_clock::now();
    }

    if (m_state == DMRTXSTATE_IDLE)
    {
        m_state = DMRTXSTATE_SLOT1;
    }

    m_DataAvailableConditionVariable.notify_one();
    return 0U;
}

uint8_t CDMRTX::writeShortLC(const uint8_t* data, uint8_t length)
{
    if (length != 9U)
        return 4U;

    std::fill_n(m_newShortLC, 12U, 0x00U);

    for (uint8_t i = 0U; i < 68U; i++) {
        bool b = READ_BIT1(data, i);
        uint8_t n = CACH_INTERLEAVE[i];
        WRITE_BIT1(m_newShortLC, n, b);
    }

    return 0U;
}

uint8_t CDMRTX::writeAbort(const uint8_t* data, uint8_t length)
{
    if (length != 1U)
        return 4U;

    switch (data[0U]) {
    case 1U:
        m_abortCount[0U] = 0U;
        m_abort[0U] = true;
        return 0U;

    case 2U:
        m_abortCount[1U] = 0U;
        m_abort[1U] = true;
        return 0U;

    default:
        return 4U;
    }
}

void CDMRTX::setStart(bool start)
{
    if (start && m_state != DMRTXSTATE_IDLE)
        return;

    m_state = start ? DMRTXSTATE_SLOT1 : DMRTXSTATE_IDLE;
    LOGCONSOLE("CDMRTX::setStart() m_state %d", m_state);

    m_frameCount = 0U;
    m_abortCount[0U] = 0U;
    m_abortCount[1U] = 0U;

    m_abort[0U] = false;
    m_abort[1U] = false;
}

void CDMRTX::setCal(bool start)
{
    m_state = start ? DMRTXSTATE_CAL : DMRTXSTATE_IDLE;
    std::cout << "CDMRTX::setCal() m_state" << std::to_string(m_state) << std::endl;
}

void CDMRTX::writeByte(uint8_t c, uint8_t control)
{
    double TXFullScale = sdr.getTXFullScale();
    float inBuffer[4U];
    float outBuffer[DMR_RADIO_SYMBOL_LENGTH * 4U];

    // Extract 4 symbols from each byte
    const uint8_t MASK = 0xC0U;
    for (uint8_t i = 0U; i < 4U; i++, c <<= 2) {
        switch (c & MASK) {
        case 0xC0U:
            inBuffer[i] = DMR_SYMBOL_A;
            break;
        case 0x80U:
            inBuffer[i] = DMR_SYMBOL_B;
            break;
        case 0x00U:
            inBuffer[i] = DMR_SYMBOL_C;
            break;
        default:
            inBuffer[i] = DMR_SYMBOL_D;
            break;
        }
    }

    uint8_t controlBuffer[DMR_RADIO_SYMBOL_LENGTH * 4U];
    std::fill_n(controlBuffer, DMR_RADIO_SYMBOL_LENGTH * 4U, MARK_NONE);
    controlBuffer[DMR_RADIO_SYMBOL_LENGTH * 2U] = control;

    // Interpolate each symbol to 5 samples each using square root raised cosine filter
    firinterp_rrrf_execute_block(m_rrc_interp_filter_obj, inBuffer, 4U, outBuffer);

    // Do the FM modulation and store the samples in m_sampleBuffer
    int index = 0;
    liquid_float_complex s;
    for (unsigned int j = 0; j < 4 * DMR_RADIO_SYMBOL_LENGTH; j++) //4 Symbols with 5 Samples each
    {
        for (int i = 0; i < 51; i++)
        {
            freqmod_modulate(m_fmod, outBuffer[j], &s);
            m_sampleBuffer[index++] = TXFullScale * s.imag;
            m_sampleBuffer[index++] = TXFullScale * s.real;
        }
    }

    io.write(STATE_DMR, m_sampleBuffer, 2040U, controlBuffer);
}

void CDMRTX::readByte()
{
    io.read();
}

uint8_t CDMRTX::getSpace1() const
{
    return 10; // Like empty ringbuffer //m_fifo[0U].getSpace() / (DMR_FRAME_LENGTH_BYTES + 2U);
}

uint8_t CDMRTX::getSpace2() const
{
    return 10; // Like empty ringbuffer // m_fifo[1U].getSpace() / (DMR_FRAME_LENGTH_BYTES + 2U);
}

void CDMRTX::createData(uint8_t slotIndex)
{
    std::lock_guard<std::mutex> lock(m_fifoMutex[slotIndex]);
    if (m_fifo[slotIndex].size() >= DMR_FRAME_LENGTH_BYTES && m_frameCount >= STARTUP_COUNT && m_abortCount[slotIndex] >= ABORT_COUNT) {
        for (unsigned int i = 0U; i < DMR_FRAME_LENGTH_BYTES; i++) {
            m_poBuffer[i] = m_fifo[slotIndex].front();
            m_fifo[slotIndex].pop();
            m_markBuffer[i] = MARK_NONE;
        }
    }
    else {
        m_abort[slotIndex] = false;
        // Transmit an idle message
        for (unsigned int i = 0U; i < DMR_FRAME_LENGTH_BYTES; i++) {
            m_poBuffer[i] = m_idle[i];
            m_markBuffer[i] = MARK_NONE;
        }
    }

    m_poLen = DMR_FRAME_LENGTH_BYTES;
    m_poPtr = 0U;
}

void CDMRTX::createCal()
{
    // 1.2 kHz sine wave generation
    if (m_modemState == STATE_DMRCAL) {
        for (unsigned int i = 0U; i < DMR_FRAME_LENGTH_BYTES; i++) {
            m_poBuffer[i] = 0x5FU;              // +3, +3, -3, -3 pattern for deviation cal.
            m_markBuffer[i] = MARK_NONE;
        }

        m_poLen = DMR_FRAME_LENGTH_BYTES;
    }

    // 80 Hz square wave generation
    if (m_modemState == STATE_LFCAL) {
        for (unsigned int i = 0U; i < 7U; i++) {
            m_poBuffer[i] = 0x55U;              // +3, +3, ... pattern
            m_markBuffer[i] = MARK_NONE;
        }

        m_poBuffer[7U] = 0x5FU;               // +3, +3, -3, -3 pattern

        for (unsigned int i = 8U; i < 15U; i++) {
            m_poBuffer[i] = 0xFFU;              // -3, -3, ... pattern
            m_markBuffer[i] = MARK_NONE;
        }

        m_poLen = 15U;
    }

    m_poPtr = 0U;
}

void CDMRTX::createCACH(uint8_t txSlotIndex, uint8_t rxSlotIndex)
{
    std::unique_lock<std::mutex> fifoGuard1(m_fifoMutex[0], std::defer_lock);
    std::unique_lock<std::mutex> fifoGuard2(m_fifoMutex[1], std::defer_lock);
    std::lock(fifoGuard1, fifoGuard2);
    m_frameCount++;
    m_abortCount[0U]++;
    m_abortCount[1U]++;

    if (m_cachPtr >= 12U)
        m_cachPtr = 0U;

    if (m_cachPtr == 0U) {
        if (m_fifo[0U].size() == 0U && m_fifo[1U].size() == 0U) {
            std::copy(std::begin(EMPTY_SHORT_LC), std::end(EMPTY_SHORT_LC), std::begin(m_shortLC));
        }
        else {
            std::copy(std::begin(m_newShortLC), std::end(m_newShortLC), std::begin(m_shortLC));
        }
    }

    std::copy(m_shortLC + m_cachPtr, m_shortLC + m_cachPtr + 3U, m_poBuffer);
    m_markBuffer[0U] = MARK_NONE;
    m_markBuffer[1U] = MARK_NONE;
    m_markBuffer[2U] = rxSlotIndex == 1U ? MARK_SLOT1 : MARK_SLOT2;

    bool at = false;
    if (m_frameCount >= STARTUP_COUNT)
        at = m_fifo[rxSlotIndex].size() > 0U;
    bool tc = txSlotIndex == 1U;
    bool ls0 = true;            // For 1 and 2
    bool ls1 = true;

    if (m_cachPtr == 0U)          // For 0
        ls1 = false;
    else if (m_cachPtr == 9U)     // For 3
        ls0 = false;

    bool h0 = at ^ tc ^ ls1;
    bool h1 = tc ^ ls1 ^ ls0;
    bool h2 = at ^ tc ^ ls0;

    m_poBuffer[0U] |= at ? 0x80U : 0x00U;
    m_poBuffer[0U] |= tc ? 0x08U : 0x00U;
    m_poBuffer[1U] |= ls1 ? 0x80U : 0x00U;
    m_poBuffer[1U] |= ls0 ? 0x08U : 0x00U;
    m_poBuffer[1U] |= h0 ? 0x02U : 0x00U;
    m_poBuffer[2U] |= h1 ? 0x20U : 0x00U;
    m_poBuffer[2U] |= h2 ? 0x02U : 0x00U;

    m_poLen = DMR_CACH_LENGTH_BYTES;
    m_poPtr = 0U;

    m_cachPtr += 3U;
    //std::cout << "CDMRTX::createCACH() " << std::to_string(txSlotIndex) << " " << std::to_string(rxSlotIndex) << std::endl;

}

void CDMRTX::setColorCode(uint8_t colorCode)
{
    std::copy(IDLE_DATA, IDLE_DATA + DMR_FRAME_LENGTH_BYTES, m_idle);

    CDMRSlotType slotType;
    slotType.encode(colorCode, DT_IDLE, m_idle);
}

void CDMRTX::resetFifo1()
{
    std::lock_guard<std::mutex> lock(m_fifoMutex[0]);
    std::queue<uint8_t>().swap(m_fifo[0U]); //clear the buffer
}

void CDMRTX::resetFifo2()
{
    std::lock_guard<std::mutex> lock(m_fifoMutex[0]);
    std::queue<uint8_t>().swap(m_fifo[1U]); //clear the buffer
}

uint32_t CDMRTX::getFrameCount()
{
    return m_frameCount;
}
