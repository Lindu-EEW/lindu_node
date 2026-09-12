#include <iostream>
#include <cmath>
#include <cassert>

// 1. Simulasi Logika STA/LTA C++ (Algoritma Deteksi Node)
class StaLtaFilter {
private:
    float sta = 0.0;
    float lta = 0.0;
    int sta_window = 10;
    int lta_window = 100;
    
public:
    float update(float pga) {
        sta = sta + (pga - sta) / sta_window;
        lta = lta + (pga - lta) / lta_window;
        if (lta == 0) return 0;
        return sta / lta;
    }
};

void test_sta_lta_trigger() {
    StaLtaFilter filter;
    
    // Background noise (0.01G)
    for(int i=0; i<100; i++) {
        filter.update(0.01);
    }
    
    // Tiba-tiba gempa (PGA 1.5G)
    float ratio = filter.update(1.5);
    
    std::cout << "Ratio STA/LTA saat gempa: " << ratio << std::endl;
    assert(ratio > 5.0 && "Gagal mendeteksi lonjakan gempa!");
}

int main() {
    std::cout << "[C++ Native Tests] Memulai pengujian Node ESP32..." << std::endl;
    
    test_sta_lta_trigger();
    
    std::cout << "[C++ Native Tests] 1/1 Pengujian Berhasil: STA/LTA Filter C++" << std::endl;
    std::cout << "ALL C++ TESTS PASSED!" << std::endl;
    return 0;
}
