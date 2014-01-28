
#ifndef __AP_HAL_PX4_RCOUTPUT_H__
#define __AP_HAL_PX4_RCOUTPUT_H__

#include <AP_HAL_PX4.h>
#include <systemlib/perf_counter.h>

#define PX4_NUM_OUTPUT_CHANNELS 16

class PX4::PX4RCOutput : public AP_HAL::RCOutput 
{
public:
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     enable_mask(uint32_t chmask);
    void     disable_ch(uint8_t ch);
    void     disable_mask(uint32_t chmask);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

    void _timer_tick(void);

private:
    int _pwm_fd;
    int _alt_fd;
    uint16_t _freq_hz;
    uint16_t _period[PX4_NUM_OUTPUT_CHANNELS];
    volatile uint8_t _max_channel;
    volatile bool _need_update;
    perf_counter_t  _perf_rcout;
    uint32_t _last_output;
    unsigned _servo_count;
    unsigned _alt_servo_count;
    uint32_t _rate_mask;
};

#endif // __AP_HAL_PX4_RCOUTPUT_H__
