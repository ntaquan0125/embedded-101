/*
 * ring_buffer.h
 *
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint8_t *buf;
	size_t head;
	size_t tail;
	size_t size;
	bool full;
} ring_buffer_t;

// Function prototypes
void ring_buffer_init(ring_buffer_t *rbuf, uint8_t *buf, size_t size);
void ring_buffer_reset(ring_buffer_t *rbuf);
bool ring_buffer_is_full(ring_buffer_t *rbuf);
bool ring_buffer_is_empty(ring_buffer_t *rbuf);
size_t ring_buffer_space(ring_buffer_t *rbuf);
bool ring_buffer_read_byte(ring_buffer_t *rbuf, uint8_t *data);
bool ring_buffer_write_byte(ring_buffer_t *rbuf, uint8_t data);

#endif /* RING_BUFFER_H_ */
