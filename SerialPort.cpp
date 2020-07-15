/*
 *   Copyright (C) 2019 by Patrick Maier DK5MP
 *   Copyright (C) 2013,2015-2019 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Colin Durbridge G4EML
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
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <sys/stat.h>
#include <ncurses.h>

#include "Config.h"
#include "Globals.h"
#include "GitVersion.h"

#include "SerialPort.h"

const uint8_t MMDVM_FRAME_START = 0xE0U;

const uint8_t MMDVM_GET_VERSION = 0x00U;
const uint8_t MMDVM_GET_STATUS = 0x01U;
const uint8_t MMDVM_SET_CONFIG = 0x02U;
const uint8_t MMDVM_SET_MODE = 0x03U;
const uint8_t MMDVM_SET_FREQ = 0x04U;

const uint8_t MMDVM_CAL_DATA = 0x08U;
const uint8_t MMDVM_RSSI_DATA = 0x09U;

const uint8_t MMDVM_SEND_CWID = 0x0AU;

const uint8_t MMDVM_DSTAR_HEADER = 0x10U;
const uint8_t MMDVM_DSTAR_DATA = 0x11U;
const uint8_t MMDVM_DSTAR_LOST = 0x12U;
const uint8_t MMDVM_DSTAR_EOT = 0x13U;

const uint8_t MMDVM_DMR_DATA1 = 0x18U;
const uint8_t MMDVM_DMR_LOST1 = 0x19U;
const uint8_t MMDVM_DMR_DATA2 = 0x1AU;
const uint8_t MMDVM_DMR_LOST2 = 0x1BU;
const uint8_t MMDVM_DMR_SHORTLC = 0x1CU;
const uint8_t MMDVM_DMR_START = 0x1DU;
const uint8_t MMDVM_DMR_ABORT = 0x1EU;

const uint8_t MMDVM_YSF_DATA = 0x20U;
const uint8_t MMDVM_YSF_LOST = 0x21U;

const uint8_t MMDVM_P25_HDR = 0x30U;
const uint8_t MMDVM_P25_LDU = 0x31U;
const uint8_t MMDVM_P25_LOST = 0x32U;

const uint8_t MMDVM_NXDN_DATA = 0x40U;
const uint8_t MMDVM_NXDN_LOST = 0x41U;

const uint8_t MMDVM_POCSAG_DATA = 0x50U;

const uint8_t MMDVM_ACK = 0x70U;
const uint8_t MMDVM_NAK = 0x7FU;

const uint8_t MMDVM_SERIAL = 0x80U;

const uint8_t MMDVM_TRANSPARENT = 0x90U;
const uint8_t MMDVM_QSO_INFO = 0x91U;

const uint8_t MMDVM_DEBUG1 = 0xF1U;
const uint8_t MMDVM_DEBUG2 = 0xF2U;
const uint8_t MMDVM_DEBUG3 = 0xF3U;
const uint8_t MMDVM_DEBUG4 = 0xF4U;
const uint8_t MMDVM_DEBUG5 = 0xF5U;

#define BAUDRATE B115200

#define HW_TYPE "MMDVM SDR"
#define DESCRIPTION "20190523 (D-Star/DMR/System Fusion/P25/NXDN/POCSAG)"
#define SERIAL_DEVICE_FILE "/dev/MMDVMSdr"

#if defined(GITVERSION)
#define concat(h, a, c) h " " a " GitID #" c ""
const char HARDWARE[] = concat(HW_TYPE, DESCRIPTION, GITVERSION);
#else
#define concat(h, a, b, c) h " " a " (Build: " b " " c ")"
const char HARDWARE[] = concat(HW_TYPE, DESCRIPTION, __TIME__, __DATE__);
#endif

const uint8_t PROTOCOL_VERSION = 1U;


CSerialPort::CSerialPort() :
    m_buffer(),
    m_ptr(0U),
    m_len(0U),
    m_debug(false)
{
    // Create virtual serial Port
    m_serial_fd = open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK);
    grantpt(m_serial_fd);
    unlockpt(m_serial_fd);
    char* pts_name = ptsname(m_serial_fd);
    //std::cout << "ptsname: " << pts_name << std::endl;

    // Try to the virtual serial port if it already exists
    remove(SERIAL_DEVICE_FILE);
    // Create symlink to virtual serial port
    symlink(pts_name, SERIAL_DEVICE_FILE);

    /* serial port parameters */
    struct termios newtio = {};
    struct termios oldtio = {};
    tcgetattr(m_serial_fd, &oldtio);

    newtio = oldtio;
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    tcflush(m_serial_fd, TCIFLUSH);

    cfsetispeed(&newtio, BAUDRATE);
    cfsetospeed(&newtio, BAUDRATE);
    tcsetattr(m_serial_fd, TCSANOW, &newtio);
}

void CSerialPort::sendACK()
{
    uint8_t reply[4U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 4U;
    reply[2U] = MMDVM_ACK;
    reply[3U] = m_buffer[2U];

    write(m_serial_fd, reply, 4);
}

void CSerialPort::sendNAK(uint8_t err)
{
    uint8_t reply[5U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 5U;
    reply[2U] = MMDVM_NAK;
    reply[3U] = m_buffer[2U];
    reply[4U] = err;

    write(m_serial_fd, reply, 5);
}

void CSerialPort::getStatus()
{
    int flags(0);
    sdr.readStreamStatus(flags);
    //std::cout << "Flags: " << std::to_string(flags) << std::endl;
    if (flags & 4) // "4" seems to be set when there is no data left in the tx buffer of the sdr
    {
        setMode(STATE_IDLE);
    }
    io.resetWatchdog();

    uint8_t reply[20U];

    // Send all sorts of interesting internal values
    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 13U;
    reply[2U] = MMDVM_GET_STATUS;

    reply[3U] = 0x00U;
    if (m_dstarEnable)
        reply[3U] |= 0x01U;
    if (m_dmrEnable)
        reply[3U] |= 0x02U;
    if (m_ysfEnable)
        reply[3U] |= 0x04U;
    if (m_p25Enable)
        reply[3U] |= 0x08U;
    if (m_nxdnEnable)
        reply[3U] |= 0x10U;
    if (m_pocsagEnable)
        reply[3U] |= 0x20U;

    reply[4U] = uint8_t(m_modemState);

    reply[5U] = m_modemState == STATE_IDLE ? 0x00U : 0x01U;

    bool adcOverflow(false);
    bool dacOverflow(false);
    //io.getOverflow(adcOverflow, dacOverflow);

    if (adcOverflow)
        reply[5U] |= 0x02U;

    if (io.hasRXOverflow())
        reply[5U] |= 0x04U;

    if (io.hasTXOverflow())
        reply[5U] |= 0x08U;

    if (io.hasLockout())
        reply[5U] |= 0x10U;

    if (dacOverflow)
        reply[5U] |= 0x20U;

    reply[5U] |= m_dcd ? 0x40U : 0x00U;
    //std::cout << std::to_string(reply[5U]) << std::endl;

    //if (m_dstarEnable)
    //    reply[6U] = dstarTX.getSpace();
    //else
        reply[6U] = 0U;

    if (m_dmrEnable) {
        if (m_duplex) {
            reply[7U] = dmrTX.getSpace1();
            reply[8U] = dmrTX.getSpace2();
        }
        else {
            reply[7U] = 10U;
            //reply[8U] = dmrDMOTX.getSpace();
        }
    }
    else {
        reply[7U] = 0U;
        reply[8U] = 0U;
    }

    //if (m_ysfEnable)
    //    reply[9U] = ysfTX.getSpace();
    //else
        reply[9U] = 0U;

    //if (m_p25Enable)
    //    reply[10U] = p25TX.getSpace();
    //else
        reply[10U] = 0U;

    //if (m_nxdnEnable)
    //    reply[11U] = nxdnTX.getSpace();
    //else
        reply[11U] = 0U;

    //if (m_pocsagEnable)
    //    reply[12U] = pocsagTX.getSpace();
    //else
        reply[12U] = 0U;

    write(m_serial_fd, reply, 13);
}

void CSerialPort::getVersion()
{
    uint8_t reply[150U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_GET_VERSION;

    reply[3U] = PROTOCOL_VERSION;

    uint8_t count = 4U;
    for (uint8_t i = 0U; HARDWARE[i] != 0x00U; i++, count++)
        reply[count] = HARDWARE[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

uint8_t CSerialPort::setConfig(const uint8_t* data, uint8_t length)
{
    if (length < 18U)
        return 4U;

    bool rxInvert = (data[0U] & 0x01U) == 0x01U;
    bool txInvert = (data[0U] & 0x02U) == 0x02U;
    bool pttInvert = (data[0U] & 0x04U) == 0x04U;
    //bool ysfLoDev = (data[0U] & 0x08U) == 0x08U;
    bool simplex = (data[0U] & 0x80U) == 0x80U;

    m_debug = (data[0U] & 0x10U) == 0x10U;

    bool dstarEnable = (data[1U] & 0x01U) == 0x01U;
    bool dmrEnable = (data[1U] & 0x02U) == 0x02U;
    bool ysfEnable = (data[1U] & 0x04U) == 0x04U;
    bool p25Enable = (data[1U] & 0x08U) == 0x08U;
    bool nxdnEnable = (data[1U] & 0x10U) == 0x10U;
    bool pocsagEnable = (data[1U] & 0x20U) == 0x20U;

    uint8_t txDelay = data[2U];
    if (txDelay > 50U)
        return 4U;

    MMDVM_STATE modemState = MMDVM_STATE(data[3U]);

    if (modemState != STATE_IDLE && modemState != STATE_DSTAR && modemState != STATE_DMR && modemState != STATE_YSF && modemState != STATE_P25 && modemState != STATE_NXDN && modemState != STATE_POCSAG && modemState != STATE_DSTARCAL && modemState != STATE_DMRCAL && modemState != STATE_RSSICAL && modemState != STATE_LFCAL && modemState != STATE_DMRCAL1K && modemState != STATE_P25CAL1K && modemState != STATE_DMRDMO1K && modemState != STATE_NXDNCAL1K && modemState != STATE_POCSAGCAL)
        return 4U;
    if (modemState == STATE_DSTAR && !dstarEnable)
        return 4U;
    if (modemState == STATE_DMR && !dmrEnable)
        return 4U;
    if (modemState == STATE_YSF && !ysfEnable)
        return 4U;
    if (modemState == STATE_P25 && !p25Enable)
        return 4U;
    if (modemState == STATE_NXDN && !nxdnEnable)
        return 4U;
    if (modemState == STATE_POCSAG && !pocsagEnable)
        return 4U;

    uint8_t rxLevel = data[4U];

    uint8_t colorCode = data[6U];
    if (colorCode > 15U)
        return 4U;

    //uint8_t dmrDelay = data[7U];

    uint8_t cwIdTXLevel = data[5U];
    uint8_t dstarTXLevel = data[9U];
    uint8_t dmrTXLevel = data[10U];
    uint8_t ysfTXLevel = data[11U];
    uint8_t p25TXLevel = data[12U];

    int16_t txDCOffset = int16_t(data[13U]) - 128;
    int16_t rxDCOffset = int16_t(data[14U]) - 128;

    uint8_t nxdnTXLevel = data[15U];

    //uint8_t ysfTXHang = data[16U];

    uint8_t pocsagTXLevel = data[17U];

    m_modemState = modemState;
    //std::cout << "SerialPort::setConfig() Set modemState to " << std::to_string(modemState) << std::endl;
    mvprintw(2, 0, "SerialPort::setConfig() Set modemState to %d", modemState);
    insertln();
    refresh();

    m_dstarEnable = dstarEnable;
    m_dmrEnable = dmrEnable;
    m_ysfEnable = ysfEnable;
    m_p25Enable = p25Enable;
    m_nxdnEnable = nxdnEnable;
    m_pocsagEnable = pocsagEnable;
    m_duplex = !simplex;

    //dstarTX.setTXDelay(txDelay);
    //ysfTX.setTXDelay(txDelay);
    //p25TX.setTXDelay(txDelay);
    //dmrDMOTX.setTXDelay(txDelay);
    //nxdnTX.setTXDelay(txDelay);
    //pocsagTX.setTXDelay(txDelay);

    dmrTX.setColorCode(colorCode);
    //dmrRX.setColorCode(colorCode);
    //dmrRX.setDelay(dmrDelay);
    //dmrDMORX.setColorCode(colorCode);
    //dmrIdleRX.setColorCode(colorCode);

    //ysfTX.setParams(ysfLoDev, ysfTXHang);

    io.setParameters(rxInvert, txInvert, pttInvert, rxLevel, cwIdTXLevel, dstarTXLevel, dmrTXLevel, ysfTXLevel, p25TXLevel, nxdnTXLevel, pocsagTXLevel, txDCOffset, rxDCOffset);

    return 0U;
}

uint8_t CSerialPort::setMode(const uint8_t* data, uint8_t length)
{
    if (length < 1U)
        return 4U;

    MMDVM_STATE modemState = MMDVM_STATE(data[0U]);

    if (modemState == m_modemState)
        return 0U;

    if (modemState != STATE_IDLE && modemState != STATE_DSTAR && modemState != STATE_DMR && modemState != STATE_YSF && modemState != STATE_P25 && modemState != STATE_NXDN && modemState != STATE_POCSAG && modemState != STATE_DSTARCAL && modemState != STATE_DMRCAL && modemState != STATE_RSSICAL && modemState != STATE_LFCAL && modemState != STATE_DMRCAL1K && modemState != STATE_P25CAL1K && modemState != STATE_DMRDMO1K && modemState != STATE_NXDNCAL1K && modemState != STATE_POCSAGCAL)
        return 4U;
    if (modemState == STATE_DSTAR && !m_dstarEnable)
        return 4U;
    if (modemState == STATE_DMR && !m_dmrEnable)
        return 4U;
    if (modemState == STATE_YSF && !m_ysfEnable)
        return 4U;
    if (modemState == STATE_P25 && !m_p25Enable)
        return 4U;
    if (modemState == STATE_NXDN && !m_nxdnEnable)
        return 4U;
    if (modemState == STATE_POCSAG && !m_pocsagEnable)
        return 4U;

    setMode(modemState);

    return 0U;
}

void CSerialPort::setMode(MMDVM_STATE modemState)
{
    switch (modemState) {
    case STATE_DMR:
        DEBUG1("Mode set to DMR");
        break;
    case STATE_DSTAR:
        DEBUG1("Mode set to D-Star");
        break;
    case STATE_YSF:
        DEBUG1("Mode set to System Fusion");
        break;
    case STATE_P25:
        DEBUG1("Mode set to P25");
        break;
    case STATE_NXDN:
        DEBUG1("Mode set to NXDN");
        break;
    case STATE_POCSAG:
        DEBUG1("Mode set to POCSAG");
        break;
    case STATE_DSTARCAL:
        DEBUG1("Mode set to D-Star Calibrate");
        break;
    case STATE_DMRCAL:
        DEBUG1("Mode set to DMR Calibrate");
        break;
    case STATE_RSSICAL:
        DEBUG1("Mode set to RSSI Calibrate");
        break;
    case STATE_LFCAL:
        DEBUG1("Mode set to 80 Hz Calibrate");
        break;
    case STATE_DMRCAL1K:
        DEBUG1("Mode set to DMR BS 1031 Hz Calibrate");
        break;
    case STATE_P25CAL1K:
        DEBUG1("Mode set to P25 1011 Hz Calibrate");
        break;
    case STATE_DMRDMO1K:
        DEBUG1("Mode set to DMR MS 1031 Hz Calibrate");
        break;
    case STATE_NXDNCAL1K:
        DEBUG1("Mode set to NXDN 1031 Hz Calibrate");
        break;
    case STATE_POCSAGCAL:
        DEBUG1("Mode set to POCSAG Calibrate");
        break;
    default:        // STATE_IDLE
        DEBUG1("Mode set to Idle");
        break;
    }

    //if (modemState != STATE_DSTAR)
    //    dstarRX.reset();

    if (modemState != STATE_DMR) {
        sdr.setStreamState(false);
        //dmrIdleRX.reset();
        //dmrDMORX.reset();
        //dmrRX.reset();
    }

    //if (modemState != STATE_YSF)
    //    ysfRX.reset();

    //if (modemState != STATE_P25)
    //    p25RX.reset();

    //if (modemState != STATE_NXDN)
    //    nxdnRX.reset();

    //cwIdTX.reset();

    m_modemState = modemState;
    mvprintw(2, 0, "SerialPort::setMode() Set modemState to %d", modemState);
    insertln();
    refresh();
    //std::cout << "SerialPort::setMode() Set modemState to " << std::to_string(modemState) << std::endl;
}

void CSerialPort::process()
{
    char c;
    while (read(m_serial_fd, &c, 1) == 1) {
        //uint8_t c = readInt(1U);

        if (m_ptr == 0U) {
            if (c == MMDVM_FRAME_START) {
                // Handle the frame start correctly
                m_buffer[0U] = c;
                m_ptr = 1U;
                m_len = 0U;
            }
            else {
                m_ptr = 0U;
                m_len = 0U;
            }
        }
        else if (m_ptr == 1U) {
            // Handle the frame length
            m_len = m_buffer[m_ptr] = c;
            m_ptr = 2U;
        }
        else {
            // Any other bytes are added to the buffer
            m_buffer[m_ptr] = c;
            m_ptr++;

            // The full packet has been received, process it
            if (m_ptr == m_len) {
                uint8_t err = 2U;

                switch (m_buffer[2U]) {
                case MMDVM_GET_STATUS:
                    //std::cout << "MMDVM_GET_STATUS" << std::endl;
                    //mvprintw(2, 0, "MMDVM_GET_STATUS");
                    insertln();
                    refresh();
                    getStatus();
                    break;

                case MMDVM_GET_VERSION:
                    //std::cout << "MMDVM_GET_VERSION" << std::endl;
                    mvprintw(2, 0, "MMDVM_GET_VERSION");
                    insertln();
                    refresh();
                    getVersion();
                    break;

                case MMDVM_SET_CONFIG:
                    //std::cout << "MMDVM_SET_CONFIG" << std::endl;
                    mvprintw(2, 0, "MMDVM_SET_CONFIG");
                    insertln();
                    refresh();
                    err = setConfig(m_buffer + 3U, m_len - 3U);
                    if (err == 0U)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case MMDVM_SET_MODE:
                    //std::cout << "MMDVM_SET_MODE" << std::endl;
                    mvprintw(2, 0, "MMDVM_SET_MODE");
                    insertln();
                    refresh();
                    err = setMode(m_buffer + 3U, m_len - 3U);
                    if (err == 0U)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case MMDVM_SET_FREQ:
                    //std::cout << "MMDVM_SET_FREQ" << std::endl;
                    mvprintw(2, 0, "MMDVM_SET_FREQ");
                    insertln();
                    refresh();
                    err = sdr.setFrequency(m_buffer + 4U, m_len - 4U);
                    if (err == 0U)
                        sendACK();
                    else
                        sendNAK(err);
                    break;

                case MMDVM_CAL_DATA:
                    std::cout << "MMDVM_CAL_DATA (not implemented)" << std::endl;
                    if (m_modemState == STATE_DSTARCAL)
                        err = 4U;// calDStarTX.write(m_buffer + 3U, m_len - 3U);
                    if (m_modemState == STATE_DMRCAL || m_modemState == STATE_LFCAL || m_modemState == STATE_DMRCAL1K || m_modemState == STATE_DMRDMO1K)
                        err = 4U;// calDMR.write(m_buffer + 3U, m_len - 3U);
                    if (m_modemState == STATE_P25CAL1K)
                        err = 4U;// calP25.write(m_buffer + 3U, m_len - 3U);
                    if (m_modemState == STATE_NXDNCAL1K)
                        err = 4U;// calNXDN.write(m_buffer + 3U, m_len - 3U);
                    if (m_modemState == STATE_POCSAGCAL)
                        err = 4U;// calPOCSAG.write(m_buffer + 3U, m_len - 3U);
                    if (err == 0U) {
                        sendACK();
                    }
                    else {
                        DEBUG2("Received invalid calibration data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_SEND_CWID:
                    std::cout << "MMDVM_SEND_CWID" << std::endl;
                    err = 5U;
                    if (m_modemState == STATE_IDLE)
                        err = 4U;// cwIdTX.write(m_buffer + 3U, m_len - 3U);
                    if (err != 0U) {
                        DEBUG2("Invalid CW Id data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DSTAR_HEADER:
                    std::cout << "MMDVM_DSTAR_HEADER" << std::endl;
                    if (m_dstarEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                            err = 4U;// dstarTX.writeHeader(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_DSTAR);
                    }
                    else {
                        DEBUG2("Received invalid D-Star header", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DSTAR_DATA:
                    std::cout << "MMDVM_DSTAR_DATA" << std::endl;
                    if (m_dstarEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                            err = 4U;// dstarTX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_DSTAR);
                    }
                    else {
                        DEBUG2("Received invalid D-Star data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DSTAR_EOT:
                    std::cout << "MMDVM_DSTAR_EOT" << std::endl;
                    if (m_dstarEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                            err = 4U;// dstarTX.writeEOT();
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_DSTAR);
                    }
                    else {
                        DEBUG2("Received invalid D-Star EOT", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DMR_DATA1:
                    //std::cout << "MMDVM_DMR_DATA1" << std::endl;
                    //std::cout << "1" << std::flush;
                    mvprintw(2, 0, "MMDVM_DMR_DATA1");
                    insertln();
                    refresh();
                    if (m_dmrEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR) {
                            if (m_duplex)
                                err = dmrTX.writeData1(m_buffer + 3U, m_len - 3U);
                        }
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_DMR);
                    }
                    else {
                        DEBUG2("Received invalid DMR data", err);
                        sendNAK(err);
                    }
                    //std::cout << "MMDVM_DMR_DATA1 after modemState:" << std::to_string(m_modemState) << std::endl;
                    break;

                case MMDVM_DMR_DATA2:
                    //std::cout << "MMDVM_DMR_DATA2" << std::endl;
                    //std::cout << "2" << std::flush;
                    mvprintw(2, 0, "MMDVM_DMR_DATA2");
                    insertln();
                    refresh();
                    if (m_dmrEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR) {
                            if (m_duplex)
                                err = dmrTX.writeData2(m_buffer + 3U, m_len - 3U);
                            else
                                err = 4U;// dmrDMOTX.writeData(m_buffer + 3U, m_len - 3U);
                        }
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_DMR);
                    }
                    else {
                        DEBUG2("Received invalid DMR data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DMR_START:
                    //std::cout << "MMDVM_DMR_START" << std::endl;
                    mvprintw(2, 0, "MMDVM_DMR_START");
                    insertln();
                    refresh();
                    if (m_dmrEnable) {
                        err = 4U;
                        if (m_len == 4U) {
                            if (m_buffer[3U] == 0x01U && m_modemState == STATE_DMR) {
                                dmrTX.setStart(true);
                                err = 0U;
                            }
                            else if (m_buffer[3U] == 0x00U && m_modemState == STATE_DMR) {
                                dmrTX.setStart(false);
                                err = 0U;
                            }
                        }
                    }
                    if (err != 0U) {
                        DEBUG2("Received invalid DMR start", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DMR_SHORTLC:
                    //std::cout << "MMDVM_DMR_SHORTLC" << std::endl;
                    mvprintw(2, 0, "MMDVM_DMR_SHORTLC");
                    insertln();
                    refresh();
                    if (m_dmrEnable)
                        err = dmrTX.writeShortLC(m_buffer + 3U, m_len - 3U);
                    if (err != 0U) {
                        DEBUG2("Received invalid DMR Short LC", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_DMR_ABORT:
                    //std::cout << "MMDVM_DMR_ABORT" << std::endl;
                    mvprintw(2, 0, "MMDVM_DMR_ABORT");
                    insertln();
                    refresh();
                    if (m_dmrEnable)
                        err = dmrTX.writeAbort(m_buffer + 3U, m_len - 3U);
                    if (err != 0U) {
                        DEBUG2("Received invalid DMR Abort", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_YSF_DATA:
                    std::cout << "MMDVM_YSF_DATA" << std::endl;
                    if (m_ysfEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_YSF)
                            err = 4U;// ysfTX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_YSF);
                    }
                    else {
                        DEBUG2("Received invalid System Fusion data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_P25_HDR:
                    std::cout << "MMDVM_P25_HDR" << std::endl;
                    if (m_p25Enable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                            err = 4U;// p25TX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_P25);
                    }
                    else {
                        DEBUG2("Received invalid P25 header", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_P25_LDU:
                    std::cout << "MMDVM_P25_LDU" << std::endl;
                    if (m_p25Enable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                            err = 4U;// p25TX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_P25);
                    }
                    else {
                        DEBUG2("Received invalid P25 LDU", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_NXDN_DATA:
                    std::cout << "MMDVM_NXDN_DATA" << std::endl;
                    if (m_nxdnEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_NXDN)
                            err = 4U;// nxdnTX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_NXDN);
                    }
                    else {
                        DEBUG2("Received invalid NXDN data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_POCSAG_DATA:
                    std::cout << "MMDVM_POCSAG_DATA" << std::endl;
                    if (m_pocsagEnable) {
                        if (m_modemState == STATE_IDLE || m_modemState == STATE_POCSAG)
                            err = 4U;// pocsagTX.writeData(m_buffer + 3U, m_len - 3U);
                    }
                    if (err == 0U) {
                        if (m_modemState == STATE_IDLE)
                            setMode(STATE_POCSAG);
                    }
                    else {
                        DEBUG2("Received invalid POCSAG data", err);
                        sendNAK(err);
                    }
                    break;

                case MMDVM_TRANSPARENT:
                case MMDVM_QSO_INFO:
                    // Do nothing on the MMDVM.
                    break;
                default:
                    // Handle this, send a NAK back
                    sendNAK(1U);
                    break;
                }

                m_ptr = 0U;
                m_len = 0U;
                break; // After processing the packet, continue to the other loops
            }
        }
    }

    if (io.getWatchdog() >= std::chrono::steady_clock::duration(std::chrono::seconds(2))) {
        m_ptr = 0U;
        m_len = 0U;
    }
}

void CSerialPort::writeDStarHeader(const uint8_t* header, uint8_t length)
{
    if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
        return;

    if (!m_dstarEnable)
        return;

    uint8_t reply[50U];
    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DSTAR_HEADER;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = header[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDStarData(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
        return;

    if (!m_dstarEnable)
        return;

    uint8_t reply[20U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DSTAR_DATA;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDStarLost()
{
    if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
        return;

    if (!m_dstarEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = MMDVM_DSTAR_LOST;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeDStarEOT()
{
    if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
        return;

    if (!m_dstarEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = MMDVM_DSTAR_EOT;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
        return;

    if (!m_dmrEnable)
        return;

    uint8_t reply[40U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = slot ? MMDVM_DMR_DATA2 : MMDVM_DMR_DATA1;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDMRLost(bool slot)
{
    if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
        return;

    if (!m_dmrEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = slot ? MMDVM_DMR_LOST2 : MMDVM_DMR_LOST1;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeYSFData(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_YSF && m_modemState != STATE_IDLE)
        return;

    if (!m_ysfEnable)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_YSF_DATA;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeYSFLost()
{
    if (m_modemState != STATE_YSF && m_modemState != STATE_IDLE)
        return;

    if (!m_ysfEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = MMDVM_YSF_LOST;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeP25Hdr(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
        return;

    if (!m_p25Enable)
        return;

    uint8_t reply[120U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_P25_HDR;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeP25Ldu(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
        return;

    if (!m_p25Enable)
        return;

    uint8_t reply[250U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_P25_LDU;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeP25Lost()
{
    if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
        return;

    if (!m_p25Enable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = MMDVM_P25_LOST;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeNXDNData(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_NXDN && m_modemState != STATE_IDLE)
        return;

    if (!m_nxdnEnable)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_NXDN_DATA;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeNXDNLost()
{
    if (m_modemState != STATE_NXDN && m_modemState != STATE_IDLE)
        return;

    if (!m_nxdnEnable)
        return;

    uint8_t reply[3U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 3U;
    reply[2U] = MMDVM_NXDN_LOST;

    write(m_serial_fd, reply, 3);
}

void CSerialPort::writeCalData(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_DSTARCAL)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_CAL_DATA;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeRSSIData(const uint8_t* data, uint8_t length)
{
    if (m_modemState != STATE_RSSICAL)
        return;

    uint8_t reply[30U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_RSSI_DATA;

    uint8_t count = 3U;
    for (uint8_t i = 0U; i < length; i++, count++)
        reply[count] = data[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDebug(const char* text)
{
    if (!m_debug)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DEBUG1;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1)
{
    if (!m_debug)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DEBUG2;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2)
{
    if (!m_debug)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DEBUG3;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3)
{
    if (!m_debug)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DEBUG4;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[count++] = (n3 >> 8) & 0xFF;
    reply[count++] = (n3 >> 0) & 0xFF;

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4)
{
    if (!m_debug)
        return;

    uint8_t reply[130U];

    reply[0U] = MMDVM_FRAME_START;
    reply[1U] = 0U;
    reply[2U] = MMDVM_DEBUG5;

    uint8_t count = 3U;
    for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
        reply[count] = text[i];

    reply[count++] = (n1 >> 8) & 0xFF;
    reply[count++] = (n1 >> 0) & 0xFF;

    reply[count++] = (n2 >> 8) & 0xFF;
    reply[count++] = (n2 >> 0) & 0xFF;

    reply[count++] = (n3 >> 8) & 0xFF;
    reply[count++] = (n3 >> 0) & 0xFF;

    reply[count++] = (n4 >> 8) & 0xFF;
    reply[count++] = (n4 >> 0) & 0xFF;

    reply[1U] = count;

    write(m_serial_fd, reply, count);
}
