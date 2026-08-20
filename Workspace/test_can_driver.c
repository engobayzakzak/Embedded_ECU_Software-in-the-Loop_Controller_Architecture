#include "can_hal.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

int main(void)
{
    printf("=== Low-Latency CAN Driver HAL SIL Verification ===\n");

    /* 1. Initialize Driver */
    bool initialized = can_hal_init();
    assert(initialized == true);
    printf("[1] CAN HAL initialized successfully.\n");

    /* 2. Configure Acceptance Filters */
    /* Accept IDs: 0x100 to 0x10F (Filter: 0x100, Mask: 0x7F0) */
    can_hal_set_filter(0, 0x100, 0x7F0);
    printf("[2] Filter Bank 0 configured: Match ID=0x100, Mask=0x7F0\n");

    /* 3. Transmit Frame with Matching ID (0x105) */
    can_frame_t tx_frame_match = {
        .id = 0x105,
        .id_type = CAN_ID_STANDARD,
        .dlc = 4,
        .data = {0xDE, 0xAD, 0xBE, 0xEF},
        .timestamp_us = 1000};
    can_status_t tx_status = can_hal_transmit(&tx_frame_match);
    assert(tx_status == CAN_STATUS_OK);
    assert(can_hal_get_rx_count() == 1);
    printf("[3] Matching Frame 0x105 transmitted and accepted into RX buffer.\n");

    /* 4. Transmit Frame with Rejected ID (0x200) */
    can_frame_t tx_frame_reject = {
        .id = 0x200,
        .id_type = CAN_ID_STANDARD,
        .dlc = 2,
        .data = {0xAA, 0xBB},
        .timestamp_us = 1050};
    tx_status = can_hal_transmit(&tx_frame_reject);
    assert(tx_status == CAN_STATUS_OK);
    assert(can_hal_get_rx_count() == 1); /* Non-matching dropped */
    printf("[4] Non-matching Frame 0x200 rejected by acceptance filter.\n");

    /* 5. Read Frame back from RX Ring Buffer */
    can_frame_t rx_frame;
    can_status_t rx_status = can_hal_receive(&rx_frame);
    assert(rx_status == CAN_STATUS_OK);
    assert(rx_frame.id == 0x105);
    assert(rx_frame.dlc == 4);
    assert(rx_frame.data[0] == 0xDE && rx_frame.data[3] == 0xEF);
    printf("[5] RX Frame parsed correctly: ID=0x%X, DLC=%d, Payload=[%02X %02X %02X %02X]\n",
           rx_frame.id, rx_frame.dlc,
           rx_frame.data[0], rx_frame.data[1], rx_frame.data[2], rx_frame.data[3]);

    /* 6. Verify Empty Buffer State */
    rx_status = can_hal_receive(&rx_frame);
    assert(rx_status == CAN_STATUS_RX_EMPTY);
    printf("[6] Empty queue check verified (returned CAN_STATUS_RX_EMPTY).\n");

    printf("\n>> SUCCESS: Module 3 Low-Latency CAN Driver HAL fully verified.\n");
    return 0;
}