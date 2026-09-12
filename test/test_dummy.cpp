#include <unity.h>

void test_basic_logic(void) {
    TEST_ASSERT_EQUAL(1, 1);
}

// Simulasi fungsi hitung PGA
float hitung_pga(float ax, float ay, float az) {
    // Rumus PGA absolut
    return ax * ax + ay * ay + az * az; 
}

void test_hitung_pga(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01, 3.0, hitung_pga(1.0, 1.0, 1.0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 14.0, hitung_pga(1.0, 2.0, 3.0));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_basic_logic);
    RUN_TEST(test_hitung_pga);
    return UNITY_END();
}
