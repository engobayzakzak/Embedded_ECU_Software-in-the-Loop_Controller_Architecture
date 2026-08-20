#ifndef CAN_TYPES_H
#define CAN_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_MAX_DLC 8U

/**
 * @brief CAN Frame Identifier Type
 */
typedef enum
{
    CAN_ID_STANDARD = 0, /* 11-bit identifier */
    CAN_ID_EXTENDED = 1  /* 29-bit identifier */
} can_id_type_t;

/**
 * @brief CAN 2.0B Protocol Message Frame
 */
typedef struct
{
    uint32_t id;
    can_id_type_t id_type;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DLC];
    uint32_t timestamp_us;
} can_frame_t;

/**
 * @brief Hardware Filter Configuration
 */
typedef struct
{
    uint32_t filter_id;
    uint32_t filter_mask;
    bool is_active;
} can_filter_t;

/**
 * @brief Driver Error / Status Flags
 */
typedef enum
{
    CAN_STATUS_OK = 0x00,
    CAN_STATUS_TX_BUSY = 0x01,
    CAN_STATUS_RX_EMPTY = 0x02,
    CAN_STATUS_OVERFLOW_ERR = 0x04,
    CAN_STATUS_BUS_OFF = 0x08
} can_status_t;

#endif /* CAN_TYPES_H */