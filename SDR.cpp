
#include <iostream>

#include "SDR.h"

void CSDR::setStreamState(bool isEnabled)
{
	if (isEnabled )
	{
		std::cout << "SDR: Enable Modem" << std::endl;

		m_device->activateStream(m_stream);
	}
	else
	{
		std::cout << "SDR: Disable Modem" << std::endl;
		m_device->deactivateStream(m_stream);
		//m_device->closeStream(m_stream);
		//SoapySDR::Device::unmake(m_device);
	}
}

void CSDR::write(int16_t* symbols, uint16_t length)
{
	int numChans(1);
	const size_t numElems = m_device->getStreamMTU(m_stream); // Number of IQ pairs
	std::vector<std::vector<int16_t>> buffMem(numChans, std::vector<int16_t>(2 * numElems)); // (2*numElements)/2  two characters fit in full buffer
	std::vector<void*> buffs(numChans);

	//std::fill(buffMem[0].begin(), buffMem[0].end(), m_fullScale);

	for (size_t j = 0; j < 4; j++) //Todo: hier erst nur 4 symbole
	{
		const char symbol = symbols[j];
		for (size_t i = 0; i < 255; i++)
		{
			switch (symbol)
			{
			case 0:
				m_sin_phase += m_phase_rate_A;// 0.003;
				break;
			case 1:
				m_sin_phase += m_phase_rate_B;// 0.001;
				break;
			case 2:
				m_sin_phase += m_phase_rate_C;// 0.001;
				break;
			case 3:
				m_sin_phase += m_phase_rate_D;// 0.003;
				break;
			default:
				std::cout << ".";
				break;
			}
			if (m_sin_phase >= 2 * M_PI)
			{
				m_sin_phase -= 2 * M_PI;
			}
			if (m_sin_phase <= -2 * M_PI)
			{
				m_sin_phase += 2 * M_PI;
			}
			buffMem[0][(j * 255 * 2) + (2 * i)] = m_fullScale * std::sin(m_sin_phase);
			buffMem[0][(j * 255 * 2) + (2 * i) + 1] = m_fullScale * std::cos(m_sin_phase);
		}
	}

	for (int i = 0; i < numChans; i++) buffs[i] = buffMem[i].data();

	//int ret(0);
	int flags(0);
	long long timeNs(0);
	m_device->writeStream(m_stream, buffs.data(), /*numElems*/1020, flags, timeNs);

}

CSDR::CSDR() :
	m_device(nullptr),
	m_samplerate(255 * 4800), //4800 symbols/s with 255 samples/symbol
	m_sin_phase(0),
	m_phase_delta(2.0 * M_PI / m_samplerate),
	m_phase_rate_A(m_phase_delta * 1944), //+1.944Hz
	m_phase_rate_B(m_phase_delta * 648),  //  +648Hz
	m_phase_rate_C(m_phase_delta * -648), //  -648Hz
	m_phase_rate_D(m_phase_delta * -1944) //-1.944Hz
{
	try
	{
		m_device = SoapySDR::Device::make("driver=lime");
		m_device->setSampleRate(SOAPY_SDR_TX, 0, m_samplerate);
		m_device->setFrequency(SOAPY_SDR_TX, 0, 430262500);
		m_device->setGain(SOAPY_SDR_TX, 0, 64);
		std::cout << "SDR: Gain: " << m_device->getGain(SOAPY_SDR_TX, 0) << std::endl;

		std::cout << "SDR: List antennas:" << std::endl;
		const auto antennas = m_device->listAntennas(SOAPY_SDR_TX, 0);
		for (const auto& antenna : antennas)
		{
			std::cout << "SDR:    " << antenna << std::endl;
		}
		std::cout << "SDR: Selected antenna: " << m_device->getAntenna(SOAPY_SDR_TX, 0) << std::endl;
		m_format = m_device->getNativeStreamFormat(SOAPY_SDR_TX, 0, m_fullScale);
		m_stream = m_device->setupStream(SOAPY_SDR_TX, m_format);
		std::cout << "SDR: Format:" << m_format << " FullScale:" << m_fullScale << std::endl;
		setStreamState(true);
		setStreamState(false);
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
