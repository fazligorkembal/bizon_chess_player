/*
 * The seam between CubeMX-generated C and the bench firmware. Generated files call only
 * these four functions, each from inside a USER CODE block, so regenerating the project
 * from codes_stm32.ioc never touches the bench logic.
 */
#ifndef JOINT_BENCH_APP_H_
#define JOINT_BENCH_APP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* main.c, USER CODE BEGIN 2: once, after every MX_*_Init() call. */
void app_init(void);

/* main.c, USER CODE BEGIN 3: every iteration of the main while(1). */
void app_loop(void);

/* usbd_cdc_if.c, CDC_Receive_FS, USER CODE BEGIN 6. USB interrupt context. */
void app_usb_rx(const uint8_t * buf, uint32_t len);

/* usbd_cdc_if.c, CDC_TransmitCplt_FS, USER CODE BEGIN 13. USB interrupt context. */
void app_usb_tx_done(void);

#ifdef __cplusplus
}
#endif

#endif  /* JOINT_BENCH_APP_H_ */
