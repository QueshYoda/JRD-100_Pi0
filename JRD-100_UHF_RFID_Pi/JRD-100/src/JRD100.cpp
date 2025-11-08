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
bool UHF_RFID::begin(const std::string &device, int baud, bool debug) {
    _debug = debug;

    serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port: " << device << std::endl;
        return false;
    }

    struct termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(serial_fd); 
        serial_fd = -1;
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
        close(serial_fd); 
        serial_fd = -1;
        return false;
    }

    if (_debug)
        std::cout << "Serial port opened: " << device << std::endl;
    return true;
}

/*! @brief Clear the buffer. */
void UHF_RFID::cleanBuffer() {
    memset(buffer, 0, sizeof(buffer));
}

/*! @brief Clear the tag buffer. */
void UHF_RFID::cleanTagsBuffer() {
    for (int i = 0; i < 200; ++i) {
        tags[i] = TagInfo{}; 
    }
}

/*! @brief Wait for message with timeout */
bool UHF_RFID::waitMsg(unsigned long timeout_ms) {
    cleanBuffer();
    auto start = std::chrono::steady_clock::now();
    uint8_t i = 0;
    bool started = false;

    while (true) {
        uint8_t b;
        int n = read(serial_fd, &b, 1);
        if (n > 0) {
            if (!started && b == 0xBB) { 
                started = true;
                buffer[i++] = b;
            } else if (started) {
                buffer[i++] = b;
                if (b == 0x7E) // Mesaj sonu
                    break;
            }
            if (i >= sizeof(buffer)) { 
                 if (_debug) std::cerr << "waitMsg buffer overflow!" << std::endl;
                 break; 
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= timeout_ms) {
            if (_debug) std::cerr << "waitMsg timeout!" << std::endl;
            break;
        }
    }

    return (started && buffer[0] == 0xBB && i > 0 && buffer[i - 1] == 0x7E);
}

/*! @brief Send command to RFID module. */
void UHF_RFID::sendCMD(const uint8_t *data, size_t size) {
    std::lock_guard<std::mutex> lock(serial_mutex); 
    if (serial_fd < 0)
        return;
    write(serial_fd, data, size);
    tcdrain(serial_fd); 
}

/*! @brief Save tag info to struct */
bool UHF_RFID::saveTagInfo(TagInfo *tag) {
    std::string rssi = hex2str(buffer[5]);
    std::string pc = hex2str(buffer[6]) + hex2str(buffer[7]);
    std::string epc = "";

    for (uint8_t i = 8; i < 20; i++)
        epc += hex2str(buffer[i]);

    if (!filterTagInfo(epc))
        return false;

    memcpy(tag->epc, buffer + 8, 12);
    tag->rssi = buffer[5];
    tag->pc[0] = buffer[6];
    tag->pc[1] = buffer[7];
    tag->rssi_str = rssi;
    tag->pc_str = pc;
    tag->epc_str = epc;

    if (_debug) {
        std::cout << "pc: " << pc << " rssi: " << rssi << " epc: " << epc << std::endl;
    }

    return true;
}

/*! @brief Filter duplicate EPCs */
bool UHF_RFID::filterTagInfo(const std::string &epc) {
    for (int i = 0; i < 200; i++) {
        if (tags[i].epc_str == epc)
            return false;
    }
    return true;
}

/*! @brief  get version info */
std::string UHF_RFID::getVersion() {
    sendCMD((uint8_t *)HARDWARE_VERSION_CMD, sizeof(HARDWARE_VERSION_CMD));
    if (waitMsg()) {
        std::string info;
        // 56 buffer boyutu, 0x7E ise bitiş karakteri
        for (uint8_t i = 6; i < 56 && i < sizeof(buffer) && buffer[i] != 0x7E; i++)
            info += (char)buffer[i];
        return info;
    } else {
        return "ERROR";
    }
}

/*! @brief  Set Single Polling  */
uint8_t UHF_RFID::pollingOnce() {
    cleanTagsBuffer();
    sendCMD((uint8_t *)POLLING_ONCE_CMD, sizeof(POLLING_ONCE_CMD));
    uint8_t count = 0;
    while (waitMsg()) {
        if (buffer[1] == 0x01 && buffer[2] == 0x22 && buffer[23] == 0x7E) { 
            if (count < 200) {
                if (saveTagInfo(&tags[count])) {
                    count++;
                }
            } else {
                return 200; 
            }
        } else if (buffer[1] == 0x01 && buffer[2] == 0x22 && buffer[4] == 0x01) {
            break; 
        }
    }
    return count;
}


/*! @brief  Set Multiple Polling  */
uint8_t UHF_RFID::pollingMultiple(uint16_t polling_count) {
    cleanTagsBuffer();
    uint8_t cmd[sizeof(POLLING_MULTIPLE_CMD)];
    memcpy(cmd, POLLING_MULTIPLE_CMD, sizeof(POLLING_MULTIPLE_CMD));

    cmd[6] = (polling_count >> 8) & 0xff;
    cmd[7] = (polling_count) & 0xff;

    uint8_t check = 0;
    for (uint8_t i = 1; i < 8; i++) { 
        check += cmd[i];
    }

    cmd[8] = check & 0xff;

    if (_debug) {
        std::cout << "send cmd: ";
        for (uint8_t i = 0; i < sizeof(POLLING_MULTIPLE_CMD); i++) {
            std::cout << hex2str(cmd[i]) << " ";
        }
        std::cout << std::endl;
    }

    sendCMD(cmd, sizeof(POLLING_MULTIPLE_CMD));

    uint8_t count = 0;
    while (waitMsg()) {
        if (buffer[1] == 0x01 && buffer[2] == 0x27 && buffer[23] == 0x7E) { 
            if (count < 200) {
                if (saveTagInfo(&tags[count])) {
                    count++;
                }
            } else {
                break; 
            }
        } else if (buffer[1] == 0x01 && buffer[2] == 0x27 && buffer[4] == 0x00) {
            break; 
        }
    }
    return count;
}

std::string UHF_RFID::selectInfo() {
    sendCMD((uint8_t *)GET_SELECT_PARAMETER_CMD, sizeof(GET_SELECT_PARAMETER_CMD));
    if (waitMsg()) {
        std::string Info = "";
        for (uint8_t i = 12; i < 24; i++) {
            Info += hex2str(buffer[i]);
        }
        if (_debug) {
            for (uint8_t i = 0; i < 26; i++) {
                std::cout << hex2str(buffer[i]) << " ";
            }
            std::cout << std::endl;
        }
        return Info;
    } else {
        return "ERROR";
    }
}

bool UHF_RFID::select(const uint8_t *epc) {
    uint8_t cmd[sizeof(SET_SELECT_PARAMETER_CMD)];
    memcpy(cmd, SET_SELECT_PARAMETER_CMD, sizeof(SET_SELECT_PARAMETER_CMD));

    if (_debug) {
        std::cout << "---raw---cmd-------------------" << std::endl;
        for (uint8_t i = 0; i < 26; i++) {
            std::cout << hex2str(cmd[i]) << " ";
        }
        std::cout << std::endl; 
        std::cout << "-------------------------" << std::endl; 
    }

    uint8_t check = 0;

    for (uint8_t i = 12; i < 24; i++) {
        cmd[i] = epc[i - 12];
    }

    for (uint8_t i = 1; i < 24; i++) {
        check += cmd[i];
    }

    cmd[24] = check & 0xff;

    if (_debug) {
        std::cout << "------cmd-------------------" << std::endl; 
        for (uint8_t i = 0; i < 26; i++) {
            std::cout << hex2str(cmd[i]) << " ";
        }
        std::cout << std::endl; 
        std::cout << "-------------------------" << std::endl; 
    }

    sendCMD(cmd, sizeof(SET_SELECT_PARAMETER_CMD));
    if (waitMsg()) {
        if (_debug) {
            for (uint8_t i = 0; i < 25; i++) { 
                std::cout << hex2str(buffer[i]) << " ";
            }
            std::cout << std::endl; 
            std::cout << "-------------------------" << std::endl; 
        }
        for (uint8_t i = 0; i < sizeof(SET_SELECT_OK); i++) {
            if (SET_SELECT_OK[i] != buffer[i]) {
                return false;
            }
        }
        return true;
    } else {
        return false;
    }
}

bool UHF_RFID::readTag(uint8_t *data, size_t size, uint8_t membank, uint16_t sa, uint32_t access_password) {
    uint8_t cmd[sizeof(READ_STORAGE_CMD)];
    memcpy(cmd, READ_STORAGE_CMD, sizeof(READ_STORAGE_CMD));

    cmd[5] = (access_password >> 24) & 0xff;
    cmd[6] = (access_password >> 16) & 0xff;
    cmd[7] = (access_password >> 8) & 0xff;
    cmd[8] = access_password & 0xff;
    cmd[9] = membank;
    

    cmd[10] = (sa >> 8) & 0xff; 
    cmd[11] = sa & 0xff;        

    uint8_t word = size / 2;
    cmd[12] = (word >> 8) & 0xff;
    cmd[13] = word & 0xff;

    uint8_t check = 0;

    for (uint8_t i = 1; i < 14; i++) { 
        check += cmd[i];
    }

    cmd[14] = check & 0xff;

    if (_debug) {
        std::cout << "send cmd:" << std::endl; 
        for (uint8_t i = 0; i < sizeof(READ_STORAGE_CMD); i++) {
            std::cout << hex2str(cmd[i]) << " ";
        }
        std::cout << std::endl; 
        std::cout << "-------------------------" << std::endl; 
    }

    sendCMD(cmd, sizeof(READ_STORAGE_CMD));
    if (waitMsg()) {
        if (_debug) {
            std::cout << "result:" << std::endl; 
            
            for (uint8_t i = 0; buffer[i] != 0x7E; i++) {
                 std::cout << hex2str(buffer[i]) << " ";
                 if (i >= 254) break; 
            }
            std::cout << "7E" << std::endl; 
            std::cout << "-------------------------" << std::endl; 
        }
        
        if (buffer[1] == 0x01 && buffer[2] == 0x82 && buffer[4] > 0x00) { 
             if (_debug) std::cerr << "readTag error code: " << hex2str(buffer[4]) << std::endl;
             return false;
        }

        uint8_t temp[size];

        for (uint8_t i = 0; i < size; i++) {
            if ((20 + i) >= sizeof(buffer)) break; 
            temp[i] = buffer[20 + i];
        }
        memcpy(data, temp, size);
        return true;
    } else {
        return false;
    }
}