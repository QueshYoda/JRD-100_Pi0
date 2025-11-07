#ifndef JRD100_H
#define JRD100_H

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <iostream>

struct TagInfo {
    uint8_t rssi;
    uint8_t pc[2];
    uint8_t epc[12];
    std::string rssi_str;
    std::string pc_str;
    std::string epc_str;
};

// Placeholder for CARD struct
struct CARD {
    uint8_t epc[12];
    int rssi;
};

class Unit_UHF_RFID {
   private:
    int serial_fd = -1;
    std::mutex serial_mutex;

    bool waitMsg(unsigned long timeout_ms = 500);
    void sendCMD(const uint8_t *data, size_t size);
    void cleanBuffer();
    void cleanCardsBuffer();
    bool saveCardInfo(CARD *card);
    bool filterCardInfo(const std::string &epc);

   public:
    bool _debug = false;
    uint8_t buffer[256] = {0};
    CARD cards[200];

   public:
    bool begin(const std::string &device = "/dev/serial0", int baud = 115200, bool debug = false);
    std::string getVersion();
    std::string selectInfo();
    uint8_t pollingOnce();
    uint8_t pollingMultiple(uint16_t polling_count);
    bool select(const uint8_t *epc);
    bool setTxPower(uint16_t db);
    bool writeCard(const uint8_t *data, size_t size, uint8_t membank, uint16_t sa, uint32_t access_password = 0);
    bool readCard(uint8_t *data, size_t size, uint8_t membank, uint16_t sa, uint32_t access_password = 0);
};

#endif // JRD100_H
