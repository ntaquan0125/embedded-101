/*
 * serial.h
 *
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "ring_buffer.h"

#define TX_BUF_SIZE 512
#define RX_BUF_SIZE 512

#define min(a, b) ((a) < (b) ? (a) : (b))

typedef struct {
	UART_HandleTypeDef* huart;

	ring_buffer_t tx_rbuf_handle;
	uint8_t tx_rbuf[TX_BUF_SIZE];
	uint8_t tx_dma_buf[TX_BUF_SIZE];

	ring_buffer_t rx_rbuf_handle;
	uint8_t rx_rbuf[TX_BUF_SIZE];
	uint8_t rx_dma_buf[RX_BUF_SIZE];

	bool tx_completed;
} serial_drv_t;

// Function prototypes
void serial_drv_init(UART_HandleTypeDef *huart);
void process_pending_Tx_transfers(void);
void serial_Rx_check(void);
void serial_drv_callback(void);
int serial_read(uint8_t *data, size_t *len);
int serial_write(uint8_t *data, size_t len);
void serial_Tx_reset(void);
void serial_Rx_reset(void);

#endif /* SERIAL_H_ */
