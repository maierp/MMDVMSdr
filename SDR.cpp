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

#include <iostream>
#include <cstddef> //size_t
#include <SoapySDR/Logger.hpp>
#include <string>

#include "Debug.h"
#include "SDR.h"

void CSDR::setStreamState(bool isEnabled)
{
    std::lock_guard<std::mutex> lock(m_streamStateMutex);
    if (m_streamState != isEnabled)
    {
        if (isEnabled)
        {
            LOGCONSOLE("SDR: Enable Modem");
            m_TXstream = m_device->setupStream(SOAPY_SDR_TX, m_TXformat);
            m_device->activateStream(m_TXstream);
            m_numElems = m_device->getStreamMTU(m_TXstream); // Number of IQ pairs
            LOGCONSOLE("SDR: NumElements: %d", m_numElems);
        }
        else
        {
            LOGCONSOLE("SDR: Disable Modem");
            m_device->deactivateStream(m_TXstream);
            m_device->closeStream(m_TXstream);
            //SoapySDR::Device::unmake(m_device);
        }
        m_streamState = isEnabled;
    }
}

int CSDR::readStreamStatus(int& flags)
{
    std::lock_guard<std::mutex> lock(m_streamStateMutex);
    std::size_t chanMask(0);
    long long timeNs(0);
    int result = 0;
    if (m_streamState) {
        result = m_device->readStreamStatus(m_TXstream, chanMask, flags, timeNs);
    }
    return result;
}

double CSDR::getTXFullScale()
{
    return m_TXfullScale;
}

void CSDR::write(std::vector<int16_t> &samples, uint16_t length)
{
    m_TXBuffs[0] = samples.data();
    m_TXBuffs[1] = m_TXBuffMem[1].data();

    int flags(0);
    long long timeNs(0);

    // write only to stream if active
    std::lock_guard<std::mutex> lock(m_streamStateMutex);
    if (m_streamState)
    {
        m_device->writeStream(m_TXstream, m_TXBuffs.data(), /*numElems*/1020, flags, timeNs);
    }
}

void CSDR::read(float* symbols, uint16_t length)
{
    for (int i = 0; i < m_numChans; i++) m_RXBuffs[i] = m_RXBuffMem[i].data();
    int flags(0);
    long long timeNs(0);
    m_device->readStream(m_RXstream, m_RXBuffs.data(), 1020, flags, timeNs);
}

static void SoapyPocoLogHandler(const SoapySDR::LogLevel logLevel, const char* message)
{
    LOGCONSOLE(message);
}

CSDR::CSDR() :
    m_streamState(false),
    m_device(nullptr),
    m_numChans(1),
    m_samplerate(255 * 4800), //4800 symbols/s with 255 samples/symbol
    m_TXBuffMem(m_numChans, std::vector<int16_t>(2 * 1020)),
    m_TXBuffs(m_numChans),
    m_RXBuffMem(m_numChans, std::vector<int16_t>(2 * 1020)),
    m_RXBuffs(m_numChans)
{
    try
    {
        SoapySDR::registerLogHandler(&SoapyPocoLogHandler);
        m_device = SoapySDR::Device::make("driver=lime");
        m_device->setSampleRate(SOAPY_SDR_TX, 0, m_samplerate);
        m_device->setSampleRate(SOAPY_SDR_RX, 0, m_samplerate);
        m_device->setFrequency(SOAPY_SDR_TX, 0, 430262500);
        m_device->setFrequency(SOAPY_SDR_RX, 0, 430262500);
        m_device->setGain(SOAPY_SDR_TX, 0, 64);
        m_device->setGain(SOAPY_SDR_RX, 0, 64);
        LOGCONSOLE("SDR: TXGain: %d", m_device->getGain(SOAPY_SDR_TX, 0));
        LOGCONSOLE("SDR: RXGain: %d", m_device->getGain(SOAPY_SDR_RX, 0));
        LOGCONSOLE("SDR: List TX antennas:");
        const auto antennasTX = m_device->listAntennas(SOAPY_SDR_TX, 0);
        for (const auto& antenna : antennasTX)
        {
            LOGCONSOLE("SDR:    %s", antenna.c_str());
        }
        LOGCONSOLE("SDR: Selected TX antenna: %s", m_device->getAntenna(SOAPY_SDR_TX, 0).c_str());

        LOGCONSOLE("SDR: List RX antennas:");
        const auto antennasRX = m_device->listAntennas(SOAPY_SDR_RX, 0);
        for (const auto& antenna : antennasRX)
        {
            LOGCONSOLE("SDR:    %s", antenna.c_str());
        }
        LOGCONSOLE("SDR: Selected RX antenna: %s", m_device->getAntenna(SOAPY_SDR_RX, 0).c_str());

        m_TXformat = m_device->getNativeStreamFormat(SOAPY_SDR_TX, 0, m_TXfullScale);
        m_RXformat = m_device->getNativeStreamFormat(SOAPY_SDR_RX, 0, m_RXfullScale);
        //m_TXstream = m_device->setupStream(SOAPY_SDR_TX, m_TXformat);
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        LOGCONSOLE("SDR: TX Format: %s FullScale: %f", m_TXformat.c_str(), m_TXfullScale);
        LOGCONSOLE("SDR: RX Format: %s FullScale: %f", m_RXformat.c_str(), m_RXfullScale);
        setStreamState(true);
        setStreamState(false);
    }
    catch (const std::exception& ex)
    {
        //std::cerr << "Error in rate test: " << ex.what() << std::endl;
        SoapySDR::Device::unmake(m_device);
    }
}

uint8_t CSDR::setFrequency(const uint8_t* data, uint8_t length)
{
    if (length < 8U)
        return 4U;

    m_rxFrequency = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
    m_txFrequency = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
    LOGCONSOLE("SDR: Set RX frequency: %d", m_rxFrequency);
    LOGCONSOLE("SDR: Set TX frequency: %d", m_txFrequency);
    m_device->setFrequency(SOAPY_SDR_TX, 0, m_txFrequency);
    Kwargs args;
    args["OFFSET"] = "0.5e6";
    m_device->setFrequency(SOAPY_SDR_RX, 0, m_rxFrequency, args);
    return 0U;
}
