/*
 * usb_bridge -- USB 2.0 High-Speed CDC-ACM link to the Raspberry Pi (runs on Core 0).
 *
 *   Core 1 control loop --usb_bridge_push_state()--> queue --> TX task --> CDC --> Pi
 *   Pi --> CDC --> RX callback --> CRC-verified parser --> mailbox <--usb_bridge_get_action()-- Core 1
 *
 * All Core-1-facing calls are non-blocking (bounded spinlock sections / zero-timeout queue ops),
 * so a stalled USB host can never delay the 200 Hz control loop; stale state frames are
 * dropped (oldest first) and counted instead.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "robot_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_bridge_start(int tx_task_core);

/* Core 1: enqueue a StatePacket for the Pi. Never blocks. */
void usb_bridge_push_state(const robot_state_packet_t *s);
/* Any core: enqueue a foveal ROI frame (drops if the previous one is still waiting). */
void usb_bridge_push_roi(const robot_roi_frame_t *r);

/* Core 1: latest valid ActionPacket. Returns false if none has ever arrived.
 * *age_ms is the time since it arrived; compare to ROBOT_ACTION_TIMEOUT_MS. */
bool usb_bridge_get_action(robot_action_packet_t *out, uint32_t *age_ms);

/* Any core: copy of the most recently pushed StatePacket (for UI / diagnostics). */
bool usb_bridge_peek_state(robot_state_packet_t *out);

typedef struct {
    uint32_t rx_frames, rx_crc_errors, rx_len_errors, tx_frames, tx_dropped_state, tx_dropped_usb;
} usb_bridge_stats_t;
void usb_bridge_get_stats(usb_bridge_stats_t *s);

#ifdef __cplusplus
}
#endif
