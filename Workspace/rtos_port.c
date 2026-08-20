#include "rtos_port.h"
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif

static rtos_queue_t g_queue_pool[8];
static size_t g_allocated_queues = 0;

rtos_queue_t *rtos_queue_create(size_t capacity, size_t item_size)
{
    if (g_allocated_queues >= 8 || capacity > RTOS_MAX_QUEUE_DEPTH || item_size > 64)
    {
        return NULL;
    }
    rtos_queue_t *q = &g_queue_pool[g_allocated_queues++];
    q->item_size = item_size;
    q->capacity = capacity;
    q->head = 0;
    q->tail = 0;
    q->count = 0;
    memset(q->buffer, 0, sizeof(q->buffer));
    return q;
}

bool rtos_queue_send(rtos_queue_t *q, const void *item, uint32_t timeout_ticks)
{
    (void)timeout_ticks;
    if (q == NULL || item == NULL || q->count >= q->capacity)
    {
        return false;
    }

    uint8_t *dest = &q->buffer[q->head * q->item_size];
    memcpy(dest, item, q->item_size);
    q->head = (q->head + 1) % q->capacity;
    q->count++;
    return true;
}

bool rtos_queue_receive(rtos_queue_t *q, void *item, uint32_t timeout_ticks)
{
    (void)timeout_ticks;
    if (q == NULL || item == NULL || q->count == 0)
    {
        return false;
    }

    uint8_t *src = &q->buffer[q->tail * q->item_size];
    memcpy(item, src, q->item_size);
    q->tail = (q->tail + 1) % q->capacity;
    q->count--;
    return true;
}

size_t rtos_queue_messages_waiting(const rtos_queue_t *q)
{
    return (q != NULL) ? q->count : 0;
}

void rtos_task_delay_ms(uint32_t ms)
{
#if defined(_WIN32) || defined(_WIN64)
    Sleep((DWORD)ms);
#else
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
#endif
}

uint32_t rtos_get_tick_count_ms(void)
{
#if defined(_WIN32) || defined(_WIN64)
    return (uint32_t)GetTickCount();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000L));
#endif
}