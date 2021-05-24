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
#include <unistd.h>
#include <liquid/liquid.h>
#include <complex.h>

const float DMR_MAX_FREQ_DEV = 1944; // 1.944 kHz | 648 Hz | -648 Hz | -1.944 kHz
const double SAMPLERATE = 255 * 4800; //4800 symbols/s with 5 samples/symbol

const unsigned int DMR_RADIO_SYMBOL_LENGTH = 5U;

const float DMR_SYMBOL_A = 1.0f;
const float DMR_SYMBOL_B = 1.0f / 3.0f;
const float DMR_SYMBOL_C = -1.0f / 3.0f;
const float DMR_SYMBOL_D = -1.0f;

std::vector<float> m_sampleBuffer(2 * 20*51 * 1200+1);
int bufIndex = 0;

freqmod m_fmod;
freqdem m_fdem;
firinterp_rrrf m_rrc_interp_filter_obj;

// Low-Pass Filter
unsigned int h_len;
float* h;
float As = 60.0f;         // stop-band attenuation [dB]
firfilt_rrrf m_low_pass_filter_obj;
firfilt_rrrf m_rrc_filt_filter_obj;


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
bool read(bool); // 4 symbols with 5 samples each = 20 samples
void LOGCONSOLE(const char* msg, ...);
void enableDisableStream(bool state);

int main() {
    sdrInit();

    m_fdem = freqdem_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */);

    // Low-Pass Filter
    h_len = estimate_req_filter_len(15000.0*2.0 / SAMPLERATE, As);
    h = new float[h_len];
    liquid_firdes_kaiser(h_len, 15000.0*2.0 / SAMPLERATE, As, 0 /*mu*/, h);
    m_low_pass_filter_obj = firfilt_rrrf_create(h, h_len);

    // RRC Filter
    m_rrc_filt_filter_obj = firfilt_rrrf_create_rnyquist(LIQUID_FIRFILT_RRC, 1, 4, 0.2, 0); // 4 Symbols with 5 interpolation samples each


    myfile.open("dmrrecording.dat", std::ios::binary);
    std::cout << "######### START #########" << std::endl;
    for (int i = 0; i < 6000; i++) {
        read(false);
        bufIndex = 0;
	if (i%1000 == 0) {
	    std::cout << (6000-i)/1000 << std::endl;
        }
    }
    bufIndex = 0;
    std::cout << "######### SIGNAL #########" << std::endl;
    for (int i = 0; i < 1200-1; i++) {
        read(true);
    }
    std::cout << "######### END #########" << std::endl;
    myfile.close();
    enableDisableStream(false);
    SoapySDR::Device::unmake(m_device);
    //std::ofstream mySampleFile;
    //mySampleFile.open("dmrcomplexsamples.dat");
    std::ofstream mySampleBinFile;
    mySampleBinFile.open("dmrbinsamples.dat", std::ios::binary);
    for (int i = 0; i < 1200 * 20*51; i++) {
    //    mySampleFile << m_sampleBuffer[i] << std::endl;
        mySampleBinFile.write(reinterpret_cast<char *>(&m_sampleBuffer[i]), sizeof(float));
    }
    //mySampleFile.close();
    mySampleBinFile.close();
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
        m_device->setFrequency(SOAPY_SDR_RX, 0, 431137300);
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

bool read(bool record)
{
    for (int i = 0; i < m_numChans; i++) m_RXBuffs[i] = m_RXBuffMem[i].data();
    int flags(0);
    long long timeNs(0);
    liquid_float_complex s;
    int index = 0;
    float outBuffer[20*51];
    float outBufferFiltered[20];
    float outBufferRRC[20];
    bool signalDetected = false;
    m_device->readStream(m_RXstream, m_RXBuffs.data(), 20*51, flags, timeNs);
    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < 51; i++) {
            m_sampleBuffer[bufIndex++] = m_RXBuffMem[0][index] / m_RXfullScale;
            signalDetected |= m_sampleBuffer[bufIndex] >= 0.015;
            s.real(m_RXBuffMem[0][index++] / m_RXfullScale);

            m_sampleBuffer[bufIndex++] = m_RXBuffMem[0][index] / m_RXfullScale;
            signalDetected |= m_sampleBuffer[bufIndex] >= 0.015;
            s.imag(m_RXBuffMem[0][index++] / m_RXfullScale);

            freqdem_demodulate(m_fdem, s, &outBuffer[j*51+i]);
            firfilt_rrrf_push(m_low_pass_filter_obj, outBuffer[j * 51 + i]);    // push input sample
            firfilt_rrrf_execute(m_low_pass_filter_obj, &outBufferFiltered[j]); // compute output
            if (signalDetected || record) {
           //     myfile.write(reinterpret_cast<char *>(&outBufferFiltered[j * 51 + i]), sizeof(float));
            }
        }
        if (signalDetected || record) {
            myfile.write(reinterpret_cast<char *>(&outBufferFiltered[j]), sizeof(float));
        }
    }

    firfilt_rrrf_execute_block(m_rrc_filt_filter_obj, outBufferFiltered, 20, outBufferRRC);

    if (signalDetected || record) {
        //myfile.write(reinterpret_cast<char *>(&outBufferFiltered[j * 51]), sizeof(float));
    }
    return signalDetected;
}
