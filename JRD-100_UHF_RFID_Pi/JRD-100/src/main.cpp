#include "JRD100.h"

int main() {
    UHF_RFID uhf;

    if (!uhf.begin("/dev/serial0", 115200, true)) {
        std::cerr << "Failed to open RFID serial port.\n";
        return 1;
    }

    std::string version = uhf.getVersion();
    std::cout << "Module version: " << version << std::endl;

    return 0;
}
