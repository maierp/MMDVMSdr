#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
#include <cstdarg>
#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <queue>
#include <vector>
#include <liquid/liquid.h>
#include <complex.h>

const float DMR_MAX_FREQ_DEV = 1944; // 1.944 kHz | 648 Hz | -648 Hz | -1.944 kHz
const double SAMPLERATE = 255 * 4800; //4800 symbols/s with 5 samples/symbol

const unsigned int DMR_RADIO_SYMBOL_LENGTH = 5U;

const float DMR_SYMBOL_A = 1.0f;
const float DMR_SYMBOL_B = 1.0f / 3.0f;
const float DMR_SYMBOL_C = -1.0f / 3.0f;
const float DMR_SYMBOL_D = -1.0f;

std::vector<int16_t> m_sampleBuffer(2 * 20*51);

freqmod m_fmod;
freqdem m_fdem;
firinterp_rrrf m_rrc_interp_filter_obj;

SoapySDR::Device* m_device;
SoapySDR::Stream* m_RXstream;
int               m_numChans(2);
int               m_numElems;
std::string       m_RXformat;
double            m_RXfullScale;
uint32_t          m_rxFrequency;
std::vector<std::vector<int16_t>> m_RXBuffMem(m_numChans, std::vector<int16_t>(2 * 20*51));
std::vector<void*> m_RXBuffs(m_numChans);
std::ofstream myfile;

void writeByte(uint8_t c);
void readByte(uint8_t* c);
void sdrInit();
void read(); // 4 symbols with 5 samples each = 20 samples
void LOGCONSOLE(const char* msg, ...);
void enableDisableStream(bool state);

int main() {
    sdrInit();
    m_fdem = freqdem_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */);
    myfile.open("dmrrecording.dat");
    std::cout << "######### START #########" << std::endl;
    for (int i = 0; i <= 5*1200; i++) {
        read();
    }
    std::cout << "######### END #########" << std::endl;
    myfile.close();
    enableDisableStream(false);
    SoapySDR::Device::unmake(m_device);
    return 0;
}

void LOGCONSOLE(const char* msg, ...) {
    va_list args;
    va_start(args, msg);
    struct timeval now;
    gettimeofday(&now, NULL);
    struct tm* tm = gmtime(&now.tv_sec);
    printf("%04d-%02d-%02d %02d:%02d:%02d.%03lu ", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, now.tv_usec / 1000U);
    vprintf(msg, args);
    printf("\n");
}

static void SoapyPocoLogHandler(const SoapySDR::LogLevel logLevel, const char* message)
{
    LOGCONSOLE(message);
}

void sdrInit() {
    try
    {
        SoapySDR::registerLogHandler(&SoapyPocoLogHandler);
        m_device = SoapySDR::Device::make("driver=lime");
        m_device->setSampleRate(SOAPY_SDR_RX, 0, SAMPLERATE);
        m_device->setFrequency(SOAPY_SDR_RX, 0, 431137500);
        m_device->setGainMode(SOAPY_SDR_RX, 0, false);
        m_device->setGain(SOAPY_SDR_RX, 0, 32);
        LOGCONSOLE("SDR: RXGain: %d", m_device->getGain(SOAPY_SDR_RX, 0));

        LOGCONSOLE("SDR: List RX antennas:");
        const auto antennasRX = m_device->listAntennas(SOAPY_SDR_RX, 0);
        for (const auto& antenna : antennasRX)
        {
            LOGCONSOLE("SDR:    %s", antenna.c_str());
        }
        LOGCONSOLE("SDR: Selected RX antenna: %s", m_device->getAntenna(SOAPY_SDR_RX, 0).c_str());

        m_RXformat = m_device->getNativeStreamFormat(SOAPY_SDR_RX, 0, m_RXfullScale);
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        LOGCONSOLE("SDR: RX Format: %s FullScale: %f", m_RXformat.c_str(), m_RXfullScale);

        enableDisableStream(false);
        enableDisableStream(true);

        m_numElems = m_device->getStreamMTU(m_RXstream); // Number of IQ pairs
        LOGCONSOLE("SDR: NumElements: %d", m_numElems);

    }
    catch (const std::exception& ex)
    {
        SoapySDR::Device::unmake(m_device);
    }

}

void enableDisableStream(bool state) {
    if (state) {
        LOGCONSOLE("SDR: Enable Modem");
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        m_device->activateStream(m_RXstream);
    } else {
        LOGCONSOLE("SDR: Disable Modem");
        m_device->deactivateStream(m_RXstream);
        m_device->closeStream(m_RXstream);
    }
}

void read()
{
    for (int i = 0; i < m_numChans; i++) m_RXBuffs[i] = m_RXBuffMem[i].data();
    int flags(0);
    long long timeNs(0);
    liquid_float_complex s;
    int index = 0;
    float outBuffer[20*51];
    m_device->readStream(m_RXstream, m_RXBuffs.data(), 20*51, flags, timeNs);
    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < 51; i++) {
            s.imag(m_RXBuffMem[0][index++] / m_RXfullScale);
            s.real(m_RXBuffMem[0][index++] / m_RXfullScale);
            freqdem_demodulate(m_fdem, s, &outBuffer[j]);
        }
        myfile << outBuffer[j] << std::endl;
    }
}
