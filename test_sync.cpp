#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
#include <cstdarg>

#include <liquid/liquid.h>
#include <queue>

const float DMR_MAX_FREQ_DEV = 1944; // 1.944 kHz | 648 Hz | -648 Hz | -1.944 kHz
const double SAMPLERATE = 5 * 4800; //4800 symbols/s with 5 samples/symbol

const unsigned int DMR_RADIO_SYMBOL_LENGTH = 5U;

const float DMR_SYMBOL_A = 1.0f;
const float DMR_SYMBOL_B = 1.0f / 3.0f;
const float DMR_SYMBOL_C = -1.0f / 3.0f;
const float DMR_SYMBOL_D = -1.0f;

std::vector<int16_t> m_sampleBuffer(2 * 1020);

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
std::vector<std::vector<int16_t>> m_RXBuffMem;
std::vector<void*> m_RXBuffs(m_numChans, std::vector<int16_t>(2 * 1020));


void writeByte(uint8_t c);
void readByte(uint8_t* c);
void sdrInit();
void read(); // 4 symbols with 5 samples each = 20 samples
void LOGCONSOLE(const char* msg, ...);

int main() {
    sdrInit();
    m_fdem = freqdem_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */);
    read();

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

void writeByte(uint8_t c)
{
    double TXFullScale = 255;
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

    // Interpolate each symbol to 5 samples each using square root raised cosine filter
    firinterp_rrrf_execute_block(m_rrc_interp_filter_obj, inBuffer, 4U, outBuffer);

    std::ofstream myfile;
    myfile.open("samples.dat");

    // Do the FM modulation and store the samples in m_sampleBuffer
    int index = 0;
    liquid_float_complex s;
    for (unsigned int j = 0; j < 4 * DMR_RADIO_SYMBOL_LENGTH; j++) //4 Symbols with 5 Samples each
    {
        for (int i = 0; i < 51; i++)
        {
            freqmod_modulate(m_fmod, outBuffer[j], &s);
            m_sampleBuffer[index++] = TXFullScale * s.imag;
            myfile << (TXFullScale * s.imag) << std::endl;
            m_sampleBuffer[index++] = TXFullScale * s.real;
            myfile << (TXFullScale * s.real) << std::endl;
        }
    }
    myfile.close();
}

void readByte(uint8_t* c) {
    int index = 0;
    float complex s;
    float outBuffer[51*4* DMR_RADIO_SYMBOL_LENGTH];
    std::ofstream myfile;
    myfile.open("demod.dat");
    for (unsigned int j = 0; j < 4 * DMR_RADIO_SYMBOL_LENGTH; j++) //4 Symbols with 5 Samples each
    {
        for (int i = 0; i < 51; i++)
        {
            s.imag = m_sampleBuffer[index++] / TXFullScale;
            s.real = m_sampleBuffer[index++] / TXFullScale;
            freqdem_demodulate(m_dem, s, &outBuffer[j*51 + i]);
            myfile << outBuffer[j * 51 + i] << std::endl;
        }
    }
    myfile.close();
}

void sdrInit() {
    try
    {
        SoapySDR::registerLogHandler(&SoapyPocoLogHandler);
        m_device = SoapySDR::Device::make("driver=lime");
        //m_device->setSampleRate(SOAPY_SDR_TX, 0, m_samplerate);
        m_device->setSampleRate(SOAPY_SDR_RX, 0, SAMPLERATE);
        //m_device->setFrequency(SOAPY_SDR_TX, 0, 430262500);
        m_device->setFrequency(SOAPY_SDR_RX, 0, 431137500);
        //m_device->setGain(SOAPY_SDR_TX, 0, 64);
        m_device->setGain(SOAPY_SDR_RX, 0, 0);
        //LOGCONSOLE("SDR: TXGain: %d", m_device->getGain(SOAPY_SDR_TX, 0));
        LOGCONSOLE("SDR: RXGain: %d", m_device->getGain(SOAPY_SDR_RX, 0));
        //LOGCONSOLE("SDR: List TX antennas:");
        //const auto antennasTX = m_device->listAntennas(SOAPY_SDR_TX, 0);
        //for (const auto& antenna : antennasTX)
        //{
        //    LOGCONSOLE("SDR:    %s", antenna.c_str());
        //}
        //LOGCONSOLE("SDR: Selected TX antenna: %s", m_device->getAntenna(SOAPY_SDR_TX, 0).c_str());

        LOGCONSOLE("SDR: List RX antennas:");
        const auto antennasRX = m_device->listAntennas(SOAPY_SDR_RX, 0);
        for (const auto& antenna : antennasRX)
        {
            LOGCONSOLE("SDR:    %s", antenna.c_str());
        }
        LOGCONSOLE("SDR: Selected RX antenna: %s", m_device->getAntenna(SOAPY_SDR_RX, 0).c_str());

        //m_TXformat = m_device->getNativeStreamFormat(SOAPY_SDR_TX, 0, m_TXfullScale);
        m_RXformat = m_device->getNativeStreamFormat(SOAPY_SDR_RX, 0, m_RXfullScale);
        //m_TXstream = m_device->setupStream(SOAPY_SDR_TX, m_TXformat);
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        //LOGCONSOLE("SDR: TX Format: %s FullScale: %f", m_TXformat.c_str(), m_TXfullScale);
        LOGCONSOLE("SDR: RX Format: %s FullScale: %f", m_RXformat.c_str(), m_RXfullScale);

        LOGCONSOLE("SDR: Enable Modem");
        m_RXstream = m_device->setupStream(SOAPY_SDR_RX, m_RXformat);
        m_device->activateStream(m_RXstream);
        m_numElems = m_device->getStreamMTU(m_RXstream); // Number of IQ pairs
        LOGCONSOLE("SDR: NumElements: %d", m_numElems);

    }
    catch (const std::exception& ex)
    {
        //std::cerr << "Error in rate test: " << ex.what() << std::endl;
        SoapySDR::Device::unmake(m_device);
    }

}void read()
{
    int flags(0);
    long long timeNs(0);
    m_device->readStream(m_RXstream, m_RXBuffs.data(), 20, flags, timeNs);
    std::cout << "Data: ";
    for (int i = 0; i < 20; i++) {
        std::cout << m_RXBuffs[0][i] << " ";
    }
    std::cout << std::endl;
}