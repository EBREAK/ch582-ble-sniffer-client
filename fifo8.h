#pragma once

#include <stdbool.h>
#include <stdint.h>

struct fifo8 {
	uint8_t *buf;
	uint32_t size;
	uint32_t head;
	uint32_t num;
};

static inline void fifo8_reset(struct fifo8 *fifo)
{
	fifo->head = 0;
	fifo->num = 0;
}

static inline uint32_t fifo8_num_used(struct fifo8 *fifo)
{
	return fifo->num;
}

static inline uint32_t fifo8_num_free(struct fifo8 *fifo)
{
	return (fifo->size - fifo->num);
}

static inline void fifo8_push(struct fifo8 *fifo, uint8_t value)
{
	fifo->buf[(fifo->head + fifo->num) % fifo->size] = value;
	fifo->num += 1;
}

static inline uint8_t fifo8_pop(struct fifo8 *fifo)
{
	uint8_t value;
	value = fifo->buf[fifo->head % fifo->size];
	fifo->head += 1;
	fifo->head %= fifo->size;
	fifo->num -= 1;
	return value;
}

#define FIFO8_INIT(SIZE)                  \
	{                                 \
		.buf = (uint8_t[SIZE]){}, \
		.size = SIZE,             \
		.head = 0,                \
		.num = 0,                 \
	}

extern void fifo8_selftest(void);
