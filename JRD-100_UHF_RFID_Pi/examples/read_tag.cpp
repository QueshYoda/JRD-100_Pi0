#include <iostream>
#include <string>
#include <iomanip> 
#include "../JRD-100/src/JRD100.h"

int main() {
    UHF_RFID rfid;

    std::cout << "RFID modülü başlatılıyor..." << std::endl;
    bool status = rfid.begin("/dev/serial0", 115200, true);

    if (!status) {
        std::cerr << "RFID modülü başlatılamadı. Portu kontrol edin." << std::endl;
        return 1;
    }

    std::cout << "Modül başarıyla başlatıldı." << std::endl;
    std::cout << "Etiket verisi okunuyor (EPC Bank, 12 Byte)..." << std::endl;

    const size_t dataSize = 12;
    uint8_t read_data[dataSize];

    uint8_t membank = 0x01;  
    uint16_t sa = 0x02;       
    uint32_t password = 0;   

    bool success = rfid.readTag(read_data, dataSize, membank, sa, password);

    if (success) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Veri başarıyla okundu (HEX):" << std::endl;
        
        std::cout << ">> ";
        for (size_t i = 0; i < dataSize; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(read_data[i]) << " ";
        }
        std::cout << std::dec << std::endl; 
        std::cout << "----------------------------------------" << std::endl;
    } else {
        std::cerr << "Etiket verisi okunamadı." << std::endl;
        std::cerr << "Alıcıda bir etiket olduğundan emin olun." << std::endl;
        return 1;
    }

    return 0;
}