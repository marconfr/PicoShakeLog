/**
 * @file psl_core0.c
 * @brief Core 0 implementation: WiFi/TCP server and network communication
 *
 * This file implements all Core 0 operations for the PicoShakeLog system,
 * responsible for network communication via WiFi SoftAP and TCP server.
 * It manages client connections, receives commands, and transmits IMU data
 * from the inter-core FIFO to connected TCP clients.
 *
 * Main functions:
 * - WiFi SoftAP setup and management
 * - TCP server and client connection handling
 * - Command protocol processing (START/STOP/STATUS)
 * - FIFO-to-TCP data streaming
 * - Client watchdog and disconnection handling
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "psl_core0.h"
#include "psl_api.h"
#include "psl_config.h"
#include "psl_fifo.h"
#include "psl_log.h"
#include "psl_shared.h"

#include "dhcpserver.h"
#include "dnsserver.h"
#include "icm42688_driver.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

/*=============================================================================
 * EXTERNAL VARIABLES
 *===========================================================================*/

extern icm42688_connection_handler_t icm42688_device;

/*=============================================================================
 * CORE 0 LOCAL VARIABLES
 *===========================================================================*/

static dhcp_server_t g_dhcp_server;
static dns_server_t  g_dns_server;

typedef struct {
    struct tcp_pcb*           server_pcb;
    struct tcp_pcb*           client_pcb;
    psl_state_handler_t* state_handler;
} tcp_server_state_t;

static tcp_server_state_t g_tcp_server = {0};

/*=============================================================================
 * DEFERRED ACTIONS (set in IRQ callbacks, executed in main loop)
 * With threadsafe_background, lwIP callbacks run in IRQ context.
 * Functions using sleep_ms (SPI sensor calls) MUST NOT be called from IRQ.
 *===========================================================================*/
typedef enum {
    DEFERRED_NONE = 0,
    DEFERRED_SENSOR_START, /* CMD_START: clear FIFO + start sensor */
    DEFERRED_SENSOR_STOP,  /* CMD_STOP or disconnect: stop sensor */
} deferred_action_t;

static volatile deferred_action_t g_deferred_action = DEFERRED_NONE;

/*=============================================================================
 * TCP CALLBACKS (PROTOTYPES)
 *===========================================================================*/

static err_t tcp_accept_cb(void* arg, struct tcp_pcb* newpcb, err_t err);
static err_t tcp_recv_cb(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);
static void  tcp_err_cb(void* arg, err_t err);
static err_t tcp_poll_cb(void* arg, struct tcp_pcb* tpcb);

/*=============================================================================
 * TCP CALLBACKS (IMPLEMENTATION)
 *===========================================================================*/

/**
 * @brief Callback function invoked when a TCP client connection is accepted.
 *
 * Handles the acceptance of incoming TCP connections. Rejects new connections
 * if a client is already connected. Upon successful acceptance, registers the
 * new client PCB and sets up receive, error, and polling callbacks.
 *
 * @param arg Pointer to the TCP server state structure (tcp_server_state_t).
 * @param newpcb Pointer to the newly created TCP protocol control block for the client connection.
 * @param err Error code from the TCP stack. Should be ERR_OK for successful acceptance.
 *
 * @return err_t Error code indicating the result of the accept operation.
 *         - ERR_OK: Connection successfully accepted.
 *         - ERR_VAL: Connection error or NULL PCB received.
 *         - ERR_ABRT: Connection rejected because a client is already connected.
 *
 * @note This function updates the shared state to indicate a client connection
 *       and transitions the device state to STATE_IDLE.
 *
 * @see tcp_recv_cb, tcp_err_cb, tcp_poll_cb, psl_set_state
 */
static err_t tcp_accept_cb(void* arg, struct tcp_pcb* newpcb, err_t err) {
    tcp_server_state_t* state = (tcp_server_state_t*)arg;

    LOG_D(TCP, "accept_cb entered: arg=%p newpcb=%p err=%d", arg, (void*)newpcb, err);

    if (err != ERR_OK || newpcb == NULL) {
        LOG_E(TCP, "Accept error");
        return ERR_VAL;
    }

    /* If a stale client_pcb exists (e.g. previous client crashed and sent RST
     * which was processed by tcp_err_cb but PCB not yet cleaned up), force-
     * close it and accept the new client instead of sending RST. */
    if (state->client_pcb != NULL) {
        LOG_W(TCP, "Stale client_pcb detected, force-closing old connection");
        /* Deregister callbacks first to avoid use-after-free */
        tcp_arg(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_abort(state->client_pcb);
        state->client_pcb = NULL;
        /* Stop any ongoing acquisition */
        shared_state_set_acquisition(&g_shared_state, false);
        g_deferred_action = DEFERRED_SENSOR_STOP;
    }

    LOG_I(TCP, "Client connected from %s", ip4addr_ntoa(ip_2_ip4(&newpcb->remote_ip)));

    /* Store client PCB */
    state->client_pcb                                     = newpcb;
    state->state_handler->modules_states.client_connected = true;
    g_shared_state.client_connected                       = true;
    shared_state_update_heartbeat(&g_shared_state);

    /* Register callbacks */
    tcp_arg(newpcb, state);
    tcp_recv(newpcb, tcp_recv_cb);
    tcp_err(newpcb, tcp_err_cb);
    tcp_poll(newpcb, tcp_poll_cb, 4);

    /* Transition to IDLE */
    psl_set_state(state->state_handler, STATE_IDLE);

    return ERR_OK;
}

/**
 * @brief TCP receive callback handler for processing client commands
 *
 * Handles incoming TCP data from connected clients. Processes single-byte commands
 * to control sensor acquisition (START/STOP) and query system status. Manages client
 * disconnection events and maintains synchronization between TCP state and shared
 * system state.
 *
 * @param[in] arg Pointer to tcp_server_state_t containing server state and handlers
 * @param[in] tpcb Pointer to TCP protocol control block
 * @param[in] p Pointer to pbuf structure containing received data (NULL on disconnect)
 * @param[in] err Error code from lwIP stack
 *
 * @return ERR_OK on successful processing, or lwIP error code on failure
 *
 * @details
 * Command processing:
 * - CMD_START: Clears ICM42688 FIFO and starts sensor acquisition (only in IDLE state)
 * - CMD_STOP: Stops sensor acquisition and halts interrupts (only in RUNNING state)
 * - CMD_STATUS: Returns current acquisition state (RUNNING or IDLE)
 *
 * Client disconnection (p == NULL):
 * - Clears client PCB reference
 * - Updates module and shared connection states
 * - Stops sensor acquisition
 * - Transitions state machine to WAIT_FOR_CLIENT
 *
 * @note Responses are sent via TCP_WRITE_FLAG_COPY and followed by tcp_output()
 * @note FIXME: Verify if FIFO clear operation clears interrupt flags
 */
static err_t tcp_recv_cb(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err) {
    tcp_server_state_t* state = (tcp_server_state_t*)arg;
    uint8_t             cmd   = 0;
    uint8_t             response;

    /* Client disconnection: p == NULL means FIN received (remote side closed).
     * We MUST call tcp_close(tpcb) here to release our side of the connection
     * and free the PCB. Without it the PCB stays in CLOSE_WAIT forever and
     * client_pcb remains non-NULL, causing every subsequent accept to be
     * rejected with RST.
     * Note: tcp_err_cb must NOT call tcp_close (PCB already freed on error). */
    if (p == NULL) {
        LOG_I(TCP, "Client disconnected");
        state->client_pcb                                     = NULL;
        state->state_handler->modules_states.client_connected = false;
        g_shared_state.client_connected                       = false;
        /* Stop acquisition flag immediately (safe - volatile bool) */
        shared_state_set_acquisition(&g_shared_state, false);
        /* Defer sensor stop to main loop - sleep_ms forbidden in IRQ context */
        g_deferred_action = DEFERRED_SENSOR_STOP;
        psl_set_state(state->state_handler, STATE_WAIT_FOR_CLIENT);
        tcp_close(tpcb);
        return ERR_OK;
    }

    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    /* Parse command (single byte) */
    pbuf_copy_partial(p, &cmd, 1, 0);

    LOG_I(TCP, "Received command: 0x%02X ('%c')", cmd, cmd);

    /* Command processing */
    switch (cmd) {
    case CMD_START:
        if (state->state_handler->current_state == STATE_IDLE) {
            /* Defer sensor start to main loop (sleep_ms forbidden in IRQ) */
            g_deferred_action = DEFERRED_SENSOR_START;
            psl_set_state(state->state_handler, STATE_RUNNING);
            response = RESP_ACK;
            tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
            LOG_I(TCP, "START accepted - sensor start deferred");
        } else {
            response = RESP_NAK;
            tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
            LOG_W(TCP, "START rejected (not IDLE)");
        }
        tcp_output(tpcb);
        break;

    case CMD_STOP:
        if (state->state_handler->current_state == STATE_RUNNING) {
            shared_state_set_acquisition(&g_shared_state, false);
            /* Defer sensor stop to main loop (sleep_ms forbidden in IRQ) */
            g_deferred_action = DEFERRED_SENSOR_STOP;
            psl_set_state(state->state_handler, STATE_IDLE);
            response = RESP_ACK;
            tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
            LOG_I(TCP, "STOP accepted - sensor stop deferred");
        } else {
            response = RESP_NAK;
            tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
            LOG_W(TCP, "STOP rejected (not RUNNING)");
        }
        tcp_output(tpcb);
        break;

    case CMD_STATUS:
        if (state->state_handler->current_state == STATE_RUNNING) {
            response = RESP_RUNNING;
        } else {
            response = RESP_IDLE;
        }
        tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        LOG_I(TCP, "STATUS query");
        break;

    default:
        response = RESP_NAK;
        tcp_write(tpcb, &response, 1, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        LOG_W(TCP, "Unknown command: 0x%02X", cmd);
        break;
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    return ERR_OK;
}

/**
 * @brief TCP error callback handler
 *
 * Handles TCP connection errors and loss of client connection. When an error occurs,
 * this callback updates the connection state, sets error flags, and stops any ongoing
 * acquisition if the system was in a running state.
 *
 * @param[in] arg Pointer to the #tcp_server_state_t structure containing server state
 * @param[in] err Error code indicating the type of TCP error that occurred
 *
 * @details
 * This function performs the following actions:
 * - Logs the TCP error code to console
 * - Invalidates the client PCB (Protocol Control Block)
 * - Updates the client connection status in both server state and shared state
 * - Sets the ERROR_CLIENT_LOST flag
 * - If acquisition is running:
 *   - Stops data acquisition
 *   - Stops the ICM42688 sensor
 *   - Transitions the system to STATE_WAIT_FOR_CLIENT
 *
 * @note This function is designed to be used as a callback for lwIP TCP error events
 * @see tcp_server_state_t
 * @see shared_state_t
 */
static void tcp_err_cb(void* arg, err_t err) {
    tcp_server_state_t* state = (tcp_server_state_t*)arg;

    LOG_E(TCP, "Error %d - connection lost", err);

    state->client_pcb                                     = NULL;
    state->state_handler->modules_states.client_connected = false;
    state->state_handler->error_flags |= ERROR_CLIENT_LOST;
    g_shared_state.client_connected = false;

    /* Stop acquisition immediately (flag safe in IRQ), defer sensor stop */
    if (state->state_handler->current_state == STATE_RUNNING) {
        shared_state_set_acquisition(&g_shared_state, false);
        g_deferred_action = DEFERRED_SENSOR_STOP;
        psl_set_state(state->state_handler, STATE_WAIT_FOR_CLIENT);
    }
}

/**
 * @brief TCP polling callback function for lwIP connection monitoring.
 *
 * @details This callback is invoked periodically by lwIP to monitor the state of a TCP connection.
 *          Since lwIP already handles TCP timeouts internally, this callback serves as a hook
 *          for any additional connection state monitoring if needed in the future.
 *
 * @param[in] arg Pointer to user-defined argument data (context).
 * @param[in] tpcb Pointer to the TCP protocol control block associated with the connection.
 *
 * @return err_t Error code. Returns ERR_OK to indicate successful polling.
 *
 * @note This callback is registered with lwIP's TCP PCB and is called at regular intervals
 *       during the lifetime of the TCP connection.
 */
static err_t tcp_poll_cb(void* arg, struct tcp_pcb* tpcb) {
    /* Monitoring (lwIP already handles TCP timeouts) */
    return ERR_OK;
}

/*=============================================================================
 * CORE 0 PUBLIC FUNCTIONS
 *===========================================================================*/

bool core0_init_wifi_tcp(psl_state_handler_t* state_handler) {
    ip4_addr_t      gw, mask;
    struct tcp_pcb* pcb;
    err_t           err;

    /* Init WiFi */
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE) != 0) {
        state_handler->error_flags = ERROR_WIFI_INIT;
        psl_set_state(state_handler, STATE_ERROR);
        return false;
    }

    /* Enable AP mode - open network (no password) for easier debugging */
    cyw43_arch_enable_ap_mode(WIFI_SSID, "", CYW43_AUTH_OPEN);
    LOG_OK(WIFI, "AP mode enabled");

    /* Configure IP addresses */
    IP4_ADDR(ip_2_ip4(&gw), 192, 168, 4, 1);
    IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

    /* Start DHCP server */
    dhcp_server_init(&g_dhcp_server, &gw, &mask);
    LOG_I(WIFI, "DHCP active (192.168.4.2-10)");

    /* Start DNS server */
    dns_server_init(&g_dns_server, &gw);
    LOG_I(WIFI, "DNS active");

    /* Create TCP server */
    pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (pcb == NULL) {
        LOG_E(TCP, "Failed to create PCB");
        state_handler->error_flags = ERROR_WIFI_INIT;
        return false;
    }

    /* Bind to port 4242 */
    err = tcp_bind(pcb, IP_ANY_TYPE, 4242);
    if (err != ERR_OK) {
        LOG_E(TCP, "Bind failed: %d", err);
        tcp_close(pcb);
        state_handler->error_flags = ERROR_WIFI_INIT;
        return false;
    }

    /* Set to listen */
    g_tcp_server.server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (g_tcp_server.server_pcb == NULL) {
        LOG_E(TCP, "Listen failed");
        tcp_close(pcb);
        state_handler->error_flags = ERROR_WIFI_INIT;
        return false;
    }

    /* Register accept callback */
    g_tcp_server.state_handler = state_handler;
    tcp_arg(g_tcp_server.server_pcb, &g_tcp_server);
    tcp_accept(g_tcp_server.server_pcb, tcp_accept_cb);

    LOG_OK(TCP, "Server listening on port 4242");
    LOG_I(WIFI, "SSID: %s", WIFI_SSID);
    LOG_D(WIFI, "Pass: %s", WIFI_PASS);
    LOG_I(WIFI, "IP  : 192.168.4.1:4242");

    return true;
}

void core0_main_loop(void) {
    uint8_t* temp_buffer;
    err_t    err;

    temp_buffer = NULL;

    /* Allocate temp buffer for FIFO read */
    if (g_fifo.slot_size > 0) {
        temp_buffer = malloc(g_fifo.slot_size);
        if (!temp_buffer) {
            LOG_E(CORE0, "Failed to allocate temp buffer");
            return;
        }
    }

    LOG_I(CORE0, "Main loop started");

    uint32_t loop_count = 0;
    uint32_t tcp_write_fail_count = 0; /* consecutive tcp_write failures on the current slot */
    (void)loop_count; /* suppress warning */

    while (true) {
        /* threadsafe_background arch: lwIP runs in IRQ context automatically.
         * Do NOT call cyw43_arch_poll() - only valid for _poll arch variant.
         * Calling it with threadsafe_background corrupts lwIP state and
         * prevents TCP SYN-ACK from being sent. */

        /* Execute deferred sensor actions (forbidden in IRQ/callback context) */
        if (g_deferred_action != DEFERRED_NONE) {
            deferred_action_t action = g_deferred_action;
            g_deferred_action        = DEFERRED_NONE;
            if (action == DEFERRED_SENSOR_START) {
                if (icm42688_start_sensor(&icm42688_device) == ICM42688_OK) {
                    /* Clear FIFO AFTER the 200ms stabilisation delay so that
                     * no stale startup samples remain.  Core 1's ISR won't
                     * miss the first FIFO_THS edge because acquisition_active
                     * is only set true right after the flush. */
                    icm42688_clear_fifo(&icm42688_device);
                    shared_state_set_acquisition(&g_shared_state, true);
                    LOG_OK(CORE0, "Sensor started");
                } else {
                    /* Start failed: revert state, notify client if still connected */
                    psl_set_state(g_tcp_server.state_handler, STATE_IDLE);
                    LOG_E(CORE0, "Sensor start failed");
                    if (g_tcp_server.client_pcb != NULL) {
                        uint8_t nak = RESP_NAK;
                        cyw43_arch_lwip_begin();
                        tcp_write(g_tcp_server.client_pcb, &nak, 1, TCP_WRITE_FLAG_COPY);
                        tcp_output(g_tcp_server.client_pcb);
                        cyw43_arch_lwip_end();
                    }
                }
            } else if (action == DEFERRED_SENSOR_STOP) {
                icm42688_stop_sensor(&icm42688_device);
                LOG_I(CORE0, "Sensor stopped");
            }
        }

        /* Drain FIFO → TCP.
         * The slot is only removed from the FIFO once tcp_write() succeeded (peek, write, drop):
         * if lwIP refuses it (ERR_MEM: segment queue or pbufs exhausted although tcp_sndbuf()
         * looked large enough), the slot stays at the head of the FIFO and is retried on the
         * next pass instead of being lost. */
        while (g_tcp_server.client_pcb != NULL && !fifo_is_empty(&g_fifo)) {
            cyw43_arch_lwip_begin();
            u16_t available = tcp_sndbuf(g_tcp_server.client_pcb);
            cyw43_arch_lwip_end();

            /* Stop draining if not enough space for one slot */
            if (available < g_fifo.slot_size) {
                break;
            }

            if (!fifo_peek(&g_fifo, temp_buffer)) {
                break;
            }

            cyw43_arch_lwip_begin();
            err = tcp_write(g_tcp_server.client_pcb, temp_buffer, g_fifo.slot_size,
                            TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(g_tcp_server.client_pcb);
                cyw43_arch_lwip_end();
                fifo_drop(&g_fifo);
                shared_state_update_heartbeat(&g_shared_state);
            } else {
                cyw43_arch_lwip_end();
                /* Slot kept in the FIFO: retried on the next pass. Log the first failure of a
                 * series and then every 100th, to keep the serial output light. */
                tcp_write_fail_count++;
                if (tcp_write_fail_count == 1 || tcp_write_fail_count % 100 == 0) {
                    LOG_W(CORE0, "tcp_write failed: err=%d (sndbuf=%u), slot kept, retry #%lu "
                                 "(FIFO=%u/%d)",
                          (int)err, (unsigned)available, (unsigned long)tcp_write_fail_count,
                          (unsigned)(FIFO_SLOTS - fifo_available_slots(&g_fifo)), FIFO_SLOTS);
                }
                sleep_ms(2); /* let lwIP/ACKs free memory instead of spinning on the lock */
                break;
            }
            if (tcp_write_fail_count > 0) {
                LOG_W(CORE0, "tcp_write recovered after %lu failed attempt(s)",
                      (unsigned long)tcp_write_fail_count);
                tcp_write_fail_count = 0;
            }
        }

        loop_count++;

        /* Sleep only when FIFO is empty to maximize throughput.
         * When data is available, loop as fast as possible. */
        if (fifo_is_empty(&g_fifo)) {
            sleep_ms(1);
        }
    }

    if (temp_buffer) {
        free(temp_buffer);
    }
}

bool core0_tcp_send(const uint8_t* data, size_t length) {
    err_t err;

    if (g_tcp_server.client_pcb == NULL) {
        return false;
    }

    err = tcp_write(g_tcp_server.client_pcb, data, length, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(g_tcp_server.client_pcb);
        shared_state_update_heartbeat(&g_shared_state);
        return true;
    }

    return false;
}

void core0_cleanup(void) {
    if (g_tcp_server.client_pcb) {
        tcp_close(g_tcp_server.client_pcb);
        g_tcp_server.client_pcb = NULL;
    }
    if (g_tcp_server.server_pcb) {
        tcp_close(g_tcp_server.server_pcb);
        g_tcp_server.server_pcb = NULL;
    }
    cyw43_arch_deinit();
}
