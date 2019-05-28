
#if !defined(SDR_H)
#define  SDR_H

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <cstdint>
#include <liquid/liquid.h>

class CSDR {
public:
	CSDR();
	uint8_t setFrequency(const uint8_t* data, uint8_t length);
	void setStreamState(bool isEnabled);
	int readStreamStatus(int& flags);
	void write(int16_t* symbols, uint16_t length);

private:
	SoapySDR::Device* m_device;
	SoapySDR::Stream* m_stream;
	std::string       m_format;
	double            m_fullScale;
	uint32_t          m_txFrequency;
	uint32_t          m_rxFrequency;
	double            m_samplerate;
	double            m_sin_phase;
	double            m_phase_delta;
	const double            m_phase_rate_A;
	const double            m_phase_rate_B;
	const double            m_phase_rate_C;
	const double            m_phase_rate_D;
	firfilt_crcf            m_rrc_filter_obj;
	liquid_float_complex m_x;
	liquid_float_complex m_y;
};

#endif
