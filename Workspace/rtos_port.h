#ifndef RTOS_PORT_H
#define RTOS_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RTOS_MAX_QUEUE_DEPTH 16

typedef void *task_handle_t;
typedef void (*task_entry_fn_t)(void *arg);

typedef struct
{
    uint8_t buffer[RTOS_MAX_QUEUE_DEPTH * 64];
    size_t item_size;
    size_t head;
    size_t tail;
    size_t count;
    size_t capacity;
} rtos_queue_t;

/* Queue Management API */
rtos_queue_t *rtos_queue_create(size_t capacity, size_t item_size);
bool rtos_queue_send(rtos_queue_t *q, const void *item, uint32_t timeout_ticks);
bool rtos_queue_receive(rtos_queue_t *q, void *item, uint32_t timeout_ticks);
size_t rtos_queue_messages_waiting(const rtos_queue_t *q);

/* Task Control & Timekeeping API */
void rtos_task_delay_ms(uint32_t ms);
uint32_t rtos_get_tick_count_ms(void);

#endif /* RTOS_PORT_H */