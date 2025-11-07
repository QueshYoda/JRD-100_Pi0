#include "JRD100.h"
#include "CMD.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

static std::string hex2str(uint8_t num) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)num;
    return ss.str();
}

/*! @brief Initialize serial port for UHF RFID module. */
bool Unit_UHF_RFID::begin(const std::string &device, int baud, bool debug) {
    _debug = debug;

    serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port: " << device << std::endl;
        return false;
    }

    struct termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        return false;
    }

    if (_debug)
        std::cout << "Serial port opened: " << device << std::endl;
    return true;
}

/*! @brief Clear the buffer. */
void Unit_UHF_RFID::cleanBuffer() {
    memset(buffer, 0, sizeof(buffer));
}

/*! @brief Clear the card buffer. */
void Unit_UHF_RFID::cleanCardsBuffer() {
    memset(cards, 0, sizeof(cards));
}

/*! @brief Wait for message with timeout */
bool Unit_UHF_RFID::waitMsg(unsigned long timeout_ms) {
    cleanBuffer();
    auto start = std::chrono::steady_clock::now();
    uint8_t i = 0;

    while (true) {
        uint8_t b;
        int n = read(serial_fd, &b, 1);
        if (n > 0) {
            buffer[i++] = b;
            if (b == 0x7E)
                break;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= timeout_ms)
            break;
    }

    return (buffer[0] == 0xBB && buffer[i - 1] == 0x7E);
}

/*! @brief Send command to RFID module. */
void Unit_UHF_RFID::sendCMD(const uint8_t *data, size_t size) {
    if (serial_fd < 0)
        return;
    write(serial_fd, data, size);
    tcdrain(serial_fd);
}

/*! @brief Save card info to struct */
bool Unit_UHF_RFID::saveCardInfo(CARD *card) {
    std::string rssi = hex2str(buffer[5]);
    std::string pc = hex2str(buffer[6]) + hex2str(buffer[7]);
    std::string epc = "";

    for (uint8_t i = 8; i < 20; i++)
        epc += hex2str(buffer[i]);

    if (!filterCardInfo(epc))
        return false;

    memcpy(card->epc, buffer + 8, 12);
    card->rssi = buffer[5];
    card->pc[0] = buffer[6];
    card->pc[1] = buffer[7];
    card->rssi_str = rssi;
    card->pc_str = pc;
    card->epc_str = epc;

    if (_debug) {
        std::cout << "pc: " << pc << " rssi: " << rssi << " epc: " << epc << std::endl;
    }

    return true;
}

/*! @brief Filter duplicate EPCs */
bool Unit_UHF_RFID::filterCardInfo(const std::string &epc) {
    for (int i = 0; i < 200; i++) {
        if (cards[i].epc_str == epc)
            return false;
    }
    return true;
}

/*! @brief Example: get version info */
std::string Unit_UHF_RFID::getVersion() {
    sendCMD((uint8_t *)HARDWARE_VERSION_CMD, sizeof(HARDWARE_VERSION_CMD));
    if (waitMsg()) {
        std::string info;
        for (uint8_t i = 6; i < 56 && buffer[i] != 0x7E; i++)
            info += (char)buffer[i];
        return info;
    } else {
        return "ERROR";
    }
}
