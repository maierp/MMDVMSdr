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

#include "SDR.h"

void CSDR::setStreamState(bool isEnabled)
{
    if (m_streamState != isEnabled)
    {
        if (isEnabled)
        {
            std::cout << "SDR: Enable Modem" << std::endl;

            m_device->activateStream(m_TXstream);
            m_numElems = m_device->getStreamMTU(m_TXstream); // Number of IQ pairs
            std::cout << "SDR: NumElements: " << m_numElems << std::endl;
        }
        else
        {
            std::cout << "SDR: Disable Modem" << std::endl;
            m_device->deactivateStream(m_TXstream);
            //m_device->closeStream(m_stream);
            //SoapySDR::Device::unmake(m_device);
        }
        m_streamState = isEnabled;
    }
}

int CSDR::readStreamStatus(int& flags)
{
    size_t chanMask(0);
    long long timeNs(0);
    return m_device->readStreamStatus(m_TXstream, chanMask, flags, timeNs);
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

    m_device->writeStream(m_TXstream, m_TXBuffs.data(), /*numElems*/1020, flags, timeNs);
}

void CSDR::read(float* symbols, uint16_t length)
{
    for (int i = 0; i < m_numChans; i++) m_RXBuffs[i] = m_RXBuffMem[i].data();
    int flags(0);
    long long timeNs(0);
    m_device->readStream(m_RXstream, m_RXBuffs.data(), 1020, flags, timeNs);
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
        m_device = SoapySDR::Device::make("driver=lime");
        m_device->setSampleRate(SOAPY_SDR_TX, 0, m_samplerate);
        m_device->setSampleRate(SOAPY_SDR_RX, 0, m_samplerate);
        m_device->setFrequency(SOAPY_SDR_TX, 0, 430262500);
        m_device->setFrequency(SOAPY_SDR_RX, 0, 430262500);
        m_device->setGain(SOAPY_SDR_TX, 0, 64);
        m_device->setGain(SOAPY_SDR_RX, 0, 64);
        std::cout << "SDR: TXGain: " << m_device->getGain(SOAPY_SDR_TX, 0) << std::endl;
        std::cout << "SDR: RXGain: " << m_device->getGain(SOAPY_SDR_RX, 0) << std::endl;

        std::cout << "SDR: List TX antennas:" << std::endl;
        const auto antennasTX = m_device->listAntennas(SOAPY_SDR_TX, 0);
        for (const auto& antenna : antennasTX)
        {
            std::cout << "SDR:    " << antenna << std::endl;
        }
        std::cout << "SDR: Selected TX antenna: " << m_device->getAntenna(SOAPY_SDR_TX, 0) << std::endl;

        std::cout << "SDR: List RX antennas:" << std::endl;
        const auto antennasRX = m_device->listAntennas(SOAPY_SDR_RX, 0);
        for (const auto& antenna : antennasRX)
        {
            std::cout << "SDR:    " << antenna << std::endl;
        }
        std::cout << "SDR: Selected RX antenna: " << m_device->getAntenna(SOAPY_SDR_RX, 0) << std::endl;

        m_TXformat = m_device->getNativeStreamFormat(SOAPY_SDR_TX, 0, m_TXfullScale);
        m_RXformat = m_device->getNativeStreamFormat(SOAPY_SDR_RX, 0, m_RXfullScale);
        m_TXstream = m_device->setupStream(SOAPY_SDR_TX, m_TXformat);
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        std::cout << "SDR: TX Format:" << m_TXformat << " FullScale:" << m_TXfullScale << std::endl;
        std::cout << "SDR: RX Format:" << m_RXformat << " FullScale:" << m_RXfullScale << std::endl;
        setStreamState(true);
        setStreamState(false);
        m_device->activateStream(m_RXstream);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error in rate test: " << ex.what() << std::endl;
        SoapySDR::Device::unmake(m_device);
    }
}

uint8_t CSDR::setFrequency(const uint8_t* data, uint8_t length)
{
    if (length < 8U)
        return 4U;

    m_rxFrequency = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
    m_txFrequency = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4];
    std::cout << "SDR: Set RX frequency: " << m_rxFrequency << std::endl;
    std::cout << "SDR: Set TX frequency: " << m_txFrequency << std::endl;
    m_device->setFrequency(SOAPY_SDR_TX, 0, m_txFrequency);
    m_device->setFrequency(SOAPY_SDR_RX, 0, m_rxFrequency);
    return 0U;
}
