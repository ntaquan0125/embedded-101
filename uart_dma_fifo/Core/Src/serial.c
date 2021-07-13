/*
 * serial.c
 *
 */

#include "stm32f1xx.h"
#include "serial.h"

static serial_drv_t serial_drv;

void serial_drv_init(UART_HandleTypeDef *huart)
{
	serial_drv.huart = huart;

	// Create 2 ring buffers
	ring_buffer_init(&serial_drv.tx_rbuf_handle, (uint8_t *)serial_drv.tx_rbuf, sizeof(serial_drv.tx_rbuf));
	ring_buffer_init(&serial_drv.rx_rbuf_handle, (uint8_t *)serial_drv.rx_rbuf, sizeof(serial_drv.rx_rbuf));

	HAL_UART_Receive_DMA(serial_drv.huart, (uint8_t*)serial_drv.rx_dma_buf, sizeof(serial_drv.rx_dma_buf));

	serial_drv.tx_completed = true;
}

void process_pending_Tx_transfers(void)
{
	if(serial_drv.tx_completed)
	{
		size_t len = min(ring_buffer_space(&serial_drv.tx_rbuf_handle), sizeof(serial_drv.tx_dma_buf));
		if (len > 0)
		{
			for (size_t i = 0; i < len; i++)
			{
				ring_buffer_read_byte(&serial_drv.tx_rbuf_handle, &serial_drv.tx_dma_buf[i]);
			}
			HAL_UART_Transmit_DMA(serial_drv.huart, serial_drv.tx_dma_buf, len);
			serial_drv.tx_completed = false;
		}
	}
}

void serial_Rx_check(void)
{
	size_t len = sizeof(serial_drv.rx_dma_buf) - __HAL_DMA_GET_COUNTER(serial_drv.huart->hdmarx);
	if (len > 0)
	{
		for (size_t i = 0; i < len; i++)
		{
			ring_buffer_write_byte(&serial_drv.rx_rbuf_handle, serial_drv.rx_dma_buf[i]);
		}
	}
	__HAL_DMA_DISABLE(serial_drv.huart->hdmarx);
	serial_drv.huart->hdmarx->Instance->CNDTR = sizeof(serial_drv.rx_dma_buf);
//	__HAL_DMA_SET_COUNTER(serial_drv.huart->hdmarx, sizeof(serial_drv.rx_dma_buf));
	__HAL_DMA_ENABLE(serial_drv.huart->hdmarx);
}

void serial_drv_callback(void)
{
	process_pending_Tx_transfers();
	serial_Rx_check();
}


int serial_read(uint8_t *data, size_t *len)
{
	for (size_t i = 0; i < *len; i++)
	{
		if(!ring_buffer_read_byte(&serial_drv.rx_rbuf_handle, &data[i]))
		{
			*len = i;
		}
	}
	return 0;
}

int serial_write(uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
	{
		if(!ring_buffer_write_byte(&serial_drv.tx_rbuf_handle, data[i]))
		{
			return i;
		}
	}
	return 0;
}

void serial_Tx_reset(void)
{
	ring_buffer_reset(&serial_drv.tx_rbuf_handle);
}

void serial_Rx_reset(void)
{
	ring_buffer_reset(&serial_drv.rx_rbuf_handle);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == serial_drv.huart->Instance)
	{
		serial_Rx_check();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == serial_drv.huart->Instance)
	{
		serial_Rx_check();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == serial_drv.huart->Instance)
	{
		serial_drv.tx_completed = true;
	}
}
