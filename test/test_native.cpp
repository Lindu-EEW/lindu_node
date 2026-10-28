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

void test_biquad_filter() {
    struct Biquad {
        float b0 = 0.131106f, b1 = 0.262213f, b2 = 0.131106f;
        float a1 = -0.747789f, a2 = 0.272215f;
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        float process(float in) {
            float out = (b0 * in) + (b1 * x1) + (b2 * x2) - (a1 * y1) - (a2 * y2);
            x2 = x1; x1 = in; y2 = y1; y1 = out;
            return out;
        }
    } filter;

    // Test 1: DC Step Response
    float step_val = 0.0f;
    for (int i = 0; i < 30; i++) {
        step_val = filter.process(1.0f);
    }
    assert(std::abs(step_val - 1.0f) < 0.01f && "Biquad filter must have unity DC gain!");

    // Test 2: Attenuation of 30 Hz noise at fs=100Hz
    filter.x1 = filter.x2 = filter.y1 = filter.y2 = 0.0f;
    float max_out = 0.0f;
    for (int n = 0; n < 100; n++) {
        float in = std::sin(2.0f * M_PI * 30.0f * (float)n / 100.0f);
        float out = filter.process(in);
        if (n > 20 && std::abs(out) > max_out) max_out = std::abs(out);
    }
    std::cout << "Peak gain at 30 Hz noise: " << max_out << " (attenuated by " << (1.0f - max_out) * 100.0f << "%)" << std::endl;
    assert(max_out < 0.20f && "Biquad filter must attenuate 30 Hz noise by >80%!");
}

int main() {
    std::cout << "[C++ Native Tests] Memulai pengujian Node ESP32..." << std::endl;
    
    test_sta_lta_trigger();
    test_biquad_filter();
    
    std::cout << "[C++ Native Tests] 2/2 Pengujian Berhasil: STA/LTA & Butterworth Filter C++" << std::endl;
    std::cout << "ALL C++ TESTS PASSED!" << std::endl;
    return 0;
}
