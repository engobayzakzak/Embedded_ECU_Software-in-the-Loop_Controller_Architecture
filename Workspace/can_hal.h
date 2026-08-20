#ifndef CAN_HAL_H
#define CAN_HAL_H

#include "can_types.h"

/* Core CAN HAL API */
bool can_hal_init(void);
can_status_t can_hal_set_filter(uint8_t filter_bank, uint32_t id, uint32_t mask);
can_status_t can_hal_transmit(const can_frame_t *frame);
can_status_t can_hal_receive(can_frame_t *frame);
uint32_t can_hal_get_rx_count(void);
void can_hal_reset_bus(void);

#endif /* CAN_HAL_H */