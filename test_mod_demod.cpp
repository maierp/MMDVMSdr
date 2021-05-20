#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <chrono>

#include <liquid/liquid.h>
#include <queue>

const float DMR_MAX_FREQ_DEV = 1944; // 1.944 kHz | 648 Hz | -648 Hz | -1.944 kHz
const double SAMPLERATE = 255 * 4800; //4800 symbols/s with 255 samples/symbol

const unsigned int DMR_RADIO_SYMBOL_LENGTH = 5U;

const float DMR_SYMBOL_A = 1.0f;
const float DMR_SYMBOL_B = 1.0f / 3.0f;
const float DMR_SYMBOL_C = -1.0f / 3.0f;
const float DMR_SYMBOL_D = -1.0f;

std::vector<int16_t> m_sampleBuffer(2 * 1020 * 20);

freqmod m_fmod;
freqdem m_fdem;
firinterp_rrrf m_rrc_interp_filter_obj;
firdecim_rrrf m_rrc_decim_filter_obj;

void writeByte(uint8_t c);
void readByte(uint8_t* c);

int index = 0;
std::ofstream samples_file;
std::ofstream demod_file;

int main() {
    m_fmod = freqmod_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */);
    m_fdem = freqdem_create(DMR_MAX_FREQ_DEV / SAMPLERATE /* modulation index */);
    m_rrc_interp_filter_obj = firinterp_rrrf_create_prototype(LIQUID_FIRFILT_RRC, 5, 4, 0.2, 0); // 4 Symbols with 5 interpolation samples each
    m_rrc_decim_filter_obj = firdecim_rrrf_create_prototype(LIQUID_FIRFILT_RRC, 5, 4, 0.2, 0); // 4 Symbols with 5 interpolation samples each
    samples_file.open("samples.dat");
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    writeByte(0xCD);
    samples_file.close();
    demod_file.open("demod.dat");
    uint8_t c;
    index = 0;
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);
    readByte(&c);

    printf("%X", c);
    demod_file.close();

    return 0;
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

    // Do the FM modulation and store the samples in m_sampleBuffer
    liquid_float_complex s;
    for (unsigned int j = 0; j < 4 * DMR_RADIO_SYMBOL_LENGTH; j++) //4 Symbols with 5 Samples each
    {
        for (int i = 0; i < 51; i++)
        {
            freqmod_modulate(m_fmod, outBuffer[j], &s);
            m_sampleBuffer[index++] = s.imag * TXFullScale;
            samples_file << (s.imag * TXFullScale) << std::endl;
            m_sampleBuffer[index++] = s.real * TXFullScale;
            samples_file << (s.real * TXFullScale) << std::endl;
        }
    }
}

void readByte(uint8_t* c) {
    double TXFullScale = 255;
    liquid_float_complex s;
    float outBuffer[51*4* DMR_RADIO_SYMBOL_LENGTH];
    float inBuffer[4U];
    for (unsigned int j = 0; j < 4 * DMR_RADIO_SYMBOL_LENGTH; j++) //4 Symbols with 5 Samples each
    {
        for (int i = 0; i < 51; i++)
        {
            s.imag = m_sampleBuffer[index++] / TXFullScale;
            s.real = m_sampleBuffer[index++] / TXFullScale;
            freqdem_demodulate(m_fdem, s, &outBuffer[j]);
        }
        //freqdem_demodulate(m_fdem, s, &outBuffer[j]);
        demod_file << outBuffer[j] << std::endl;

    }
    firdecim_rrrf_execute_block(m_rrc_decim_filter_obj, outBuffer, 4U, inBuffer);
    for (int i = 0; i < 4; i++) {
        printf("%f\n", inBuffer[i]);
    }
}
