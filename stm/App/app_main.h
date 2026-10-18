#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called from main.c (USER CODE 2); never returns. */
void app_main(void);

#include <stdint.h>

/* IMU data ready (INT1 EXTI), from stm32l4xx_it.c USER CODE; stamp = DWT cycle counter. */
void app_imu_data_ready_irq(uint32_t stamp);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
