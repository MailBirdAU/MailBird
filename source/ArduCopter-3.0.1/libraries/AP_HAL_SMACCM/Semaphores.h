
#ifndef __AP_HAL_SMACCM_SEMAPHORE_H__
#define __AP_HAL_SMACCM_SEMAPHORE_H__

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <AP_HAL_SMACCM.h>
#include <FreeRTOS.h>
#include <semphr.h>

class SMACCM::SMACCMSemaphore : public AP_HAL::Semaphore {
public:
    SMACCMSemaphore();

    void init();
    virtual bool take(uint32_t timeout_ms);
    virtual bool take_nonblocking();
    virtual bool give();

private:
    xSemaphoreHandle m_semaphore;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
#endif // __AP_HAL_SMACCM_SEMAPHORE_H__
