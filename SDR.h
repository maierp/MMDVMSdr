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

#if !defined(SDR_H)
#define  SDR_H

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <cstdint>

class CSDR {
public:
	CSDR();
	uint8_t setFrequency(const uint8_t* data, uint8_t length);
	void setStreamState(bool isEnabled);
	int readStreamStatus(int& flags);
	void write(float* symbols, uint16_t length);

private:
	SoapySDR::Device* m_device;
	SoapySDR::Stream* m_stream;
	int               m_numChans;
	size_t            m_numElems;
	std::string       m_format;
	double            m_fullScale;
	uint32_t          m_txFrequency;
	uint32_t          m_rxFrequency;
	double            m_samplerate;
	double            m_sin_phase;
	double            m_phase_delta;
	std::vector<std::vector<int16_t>> m_buffMem;
	std::vector<void*> m_buffs;
};

#endif
