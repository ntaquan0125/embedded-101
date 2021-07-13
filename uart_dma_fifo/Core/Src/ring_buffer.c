/*
 * ring_buffer.c
 *
 */

#include <stdlib.h>

#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *rbuf, uint8_t *buf, size_t size)
{
	rbuf->buf = buf;
	rbuf->size = size;
	ring_buffer_reset(rbuf);
}

void ring_buffer_reset(ring_buffer_t *rbuf)
{
	rbuf->head = rbuf->tail = 0;
	rbuf->full = false;
}

bool ring_buffer_is_full(ring_buffer_t *rbuf)
{
    return rbuf->full;
}

bool ring_buffer_is_empty(ring_buffer_t *rbuf)
{
    return (!rbuf->full && (rbuf->head == rbuf->tail));
}

size_t ring_buffer_space(ring_buffer_t *rbuf)
{
	size_t size = rbuf->size;

	if (!ring_buffer_is_full(rbuf))
	{
		if(rbuf->head >= rbuf->tail)
		{
			size = rbuf->head - rbuf->tail;
		}
		else
		{
			size = rbuf->size + rbuf->head - rbuf->tail;
		}
	}
	return size;
}

bool ring_buffer_read_byte(ring_buffer_t *rbuf, uint8_t *data)
{
    if (!ring_buffer_is_empty(rbuf))
    {
        *data = rbuf->buf[rbuf->tail];
    	rbuf->tail = (rbuf->tail + 1) % rbuf->size;
    	rbuf->full = false;
        return true;
    }
    return false;
}

bool ring_buffer_write_byte(ring_buffer_t *rbuf, uint8_t data)
{
	if (!ring_buffer_is_full(rbuf))
	{
		rbuf->buf[rbuf->head] = data;
		rbuf->head = (rbuf->head + 1) % rbuf->size;
		rbuf->full = (rbuf->head == rbuf->tail);
		return true;
	}
	return 0;
}
