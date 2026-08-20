#include "can_hal.h"
#include <string.h>

#define CAN_RING_BUFFER_SIZE 32U
#define CAN_MAX_FILTER_BANKS 4U

typedef struct
{
    can_frame_t frames[CAN_RING_BUFFER_SIZE];
    size_t head;
    size_t tail;
    size_t count;
    uint32_t overflow_count;
} can_ring_buffer_t;

static can_ring_buffer_t g_rx_ring_buffer;
static can_filter_t g_filters[CAN_MAX_FILTER_BANKS];
static bool g_driver_initialized = false;

static bool passes_filter(uint32_t id)
{
    bool has_active_filters = false;

    for (uint8_t i = 0; i < CAN_MAX_FILTER_BANKS; i++)
    {
        if (g_filters[i].is_active)
        {
            has_active_filters = true;
            /* Acceptance Equation: (ID & MASK) == (FILTER_ID & MASK) */
            if ((id & g_filters[i].filter_mask) == (g_filters[i].filter_id & g_filters[i].filter_mask))
            {
                return true;
            }
        }
    }

    return !has_active_filters;
}

bool can_hal_init(void)
{
    memset(&g_rx_ring_buffer, 0, sizeof(g_rx_ring_buffer));
    memset(g_filters, 0, sizeof(g_filters));
    g_driver_initialized = true;
    return true;
}

can_status_t can_hal_set_filter(uint8_t filter_bank, uint32_t id, uint32_t mask)
{
    if (filter_bank >= CAN_MAX_FILTER_BANKS)
    {
        return CAN_STATUS_OVERFLOW_ERR;
    }

    g_filters[filter_bank].filter_id = id;
    g_filters[filter_bank].filter_mask = mask;
    g_filters[filter_bank].is_active = true;
    return CAN_STATUS_OK;
}

can_status_t can_hal_transmit(const can_frame_t *frame)
{
    if (!g_driver_initialized || frame == NULL || frame->dlc > CAN_MAX_DLC)
    {
        return CAN_STATUS_OVERFLOW_ERR;
    }

    /* SIL Loopback: Check acceptance filter and push to RX ring buffer */
    if (passes_filter(frame->id))
    {
        if (g_rx_ring_buffer.count >= CAN_RING_BUFFER_SIZE)
        {
            g_rx_ring_buffer.overflow_count++;
            return CAN_STATUS_OVERFLOW_ERR;
        }

        g_rx_ring_buffer.frames[g_rx_ring_buffer.head] = *frame;
        g_rx_ring_buffer.head = (g_rx_ring_buffer.head + 1U) % CAN_RING_BUFFER_SIZE;
        g_rx_ring_buffer.count++;
    }

    return CAN_STATUS_OK;
}

can_status_t can_hal_receive(can_frame_t *frame)
{
    if (!g_driver_initialized || frame == NULL)
    {
        return CAN_STATUS_OVERFLOW_ERR;
    }

    if (g_rx_ring_buffer.count == 0U)
    {
        return CAN_STATUS_RX_EMPTY;
    }

    *frame = g_rx_ring_buffer.frames[g_rx_ring_buffer.tail];
    g_rx_ring_buffer.tail = (g_rx_ring_buffer.tail + 1U) % CAN_RING_BUFFER_SIZE;
    g_rx_ring_buffer.count--;

    return CAN_STATUS_OK;
}

uint32_t can_hal_get_rx_count(void)
{
    return (uint32_t)g_rx_ring_buffer.count;
}

void can_hal_reset_bus(void)
{
    g_rx_ring_buffer.head = 0;
    g_rx_ring_buffer.tail = 0;
    g_rx_ring_buffer.count = 0;
}