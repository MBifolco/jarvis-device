#include "l2cap_stream.h"
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "nimble/nimble_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "config.h"
#include "audio_rx.h"

#define TAG "L2CAP"
#define JARVIS_PSM 0x0040
#define L2CAP_MTU  768

// Packet statistics
static uint32_t g_packets_received = 0;
static uint32_t g_bytes_received = 0;

// Memory pool for L2CAP receive buffers - increased for better throughput
static struct os_mbuf_pool g_l2cap_coc_mbuf_pool;
static struct os_mempool g_l2cap_coc_mempool;
static os_membuf_t g_l2cap_coc_membuf[OS_MEMPOOL_SIZE(16, 800)]; // 16 buffers of 800 bytes each

// Channel structure to track connections
typedef struct {
    struct ble_l2cap_chan *chan;
    uint16_t conn_handle;
    bool connected;
} l2cap_channel_t;

static l2cap_channel_t g_channel = {0};

// Task-based deferral for recv_ready calls
static QueueHandle_t recv_ready_queue = NULL;

typedef struct {
    struct ble_l2cap_chan *chan;
} recv_ready_msg_t;

// Forward declarations
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx);
static int l2cap_server_accept(uint16_t conn_handle, uint16_t peer_sdu_size, struct ble_l2cap_chan *chan);
static void recv_ready_task(void *pvParameters);

/**
 * Task to handle deferred recv_ready calls when BLE_HS_EBUSY occurs
 */
static void recv_ready_task(void *pvParameters)
{
    recv_ready_msg_t msg;
    
    while (1) {
        if (xQueueReceive(recv_ready_queue, &msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Processing deferred recv_ready request...");
            // Minimal delay to ensure L2CAP state is settled - reduced for throughput
            vTaskDelay(pdMS_TO_TICKS(1));
            
            struct os_mbuf *next_sdu_rx = os_mbuf_get_pkthdr(&g_l2cap_coc_mbuf_pool, 0);
            if (next_sdu_rx) {
                int rc = ble_l2cap_recv_ready(msg.chan, next_sdu_rx);
                if (rc == 0) {
                    ESP_LOGI(TAG, "Deferred recv_ready succeeded - waiting for next packet");
                } else {
                    ESP_LOGE(TAG, "Deferred recv_ready failed: %d", rc);
                    os_mbuf_free_chain(next_sdu_rx);
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate mbuf for deferred recv_ready");
            }
        }
    }
}

static int l2cap_event_cb(struct ble_l2cap_event *event, void *arg)
{
    struct ble_l2cap_chan_info chan_info;
    
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:
            ESP_LOGI(TAG, "L2CAP channel connected - conn_handle=%d", event->connect.conn_handle);
            if (ble_l2cap_get_chan_info(event->connect.chan, &chan_info) == 0) {
                ESP_LOGI(TAG, "Final negotiated MTUs: our_l2cap_mtu=%d, peer_l2cap_mtu=%d, psm=%d, our_coc_mtu=%d, peer_coc_mtu=%d",
                        chan_info.our_l2cap_mtu, chan_info.peer_l2cap_mtu, chan_info.psm, 
                        chan_info.our_coc_mtu, chan_info.peer_coc_mtu);
            } else {
                ESP_LOGE(TAG, "Failed to get channel info after connection");
            }
            g_channel.chan = event->connect.chan;
            g_channel.conn_handle = event->connect.conn_handle;
            g_channel.connected = true;
            
            // Initial buffer setup for receiving
            struct os_mbuf *initial_sdu_rx = os_mbuf_get_pkthdr(&g_l2cap_coc_mbuf_pool, 0);
            if (initial_sdu_rx) {
                int rc = ble_l2cap_recv_ready(event->connect.chan, initial_sdu_rx);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to prepare initial receive buffer: %d", rc);
                    os_mbuf_free_chain(initial_sdu_rx);
                } else {
                    ESP_LOGI(TAG, "Initial receive buffer prepared successfully");
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate initial receive buffer");
            }
            break;

        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            ESP_LOGI(TAG, "L2CAP channel disconnected");
            g_channel.connected = false;
            g_channel.chan = NULL;
            break;

        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            ESP_LOGI(TAG, "L2CAP data received - packet arrived!");
            l2cap_server_recv(event->receive.chan, event->receive.sdu_rx);
            break;

        case BLE_L2CAP_EVENT_COC_ACCEPT:
            ESP_LOGI(TAG, "L2CAP accept request - conn_handle=%d, peer_sdu_size=%d", 
                    event->accept.conn_handle, event->accept.peer_sdu_size);
            return l2cap_server_accept(event->accept.conn_handle, 
                                     event->accept.peer_sdu_size, 
                                     event->accept.chan);

        case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
            ESP_LOGI(TAG, "L2CAP TX unstalled - can send more data");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled L2CAP event: %d", event->type);
            break;
    }

    return 0;
}

/**
 * Handle L2CAP accept request and prepare receive buffer
 */
static int l2cap_server_accept(uint16_t conn_handle, uint16_t peer_sdu_size, struct ble_l2cap_chan *chan) {
    struct os_mbuf *sdu_rx;
    struct ble_l2cap_chan_info chan_info;
    int rc;
    
    ESP_LOGI(TAG, "Accepting L2CAP connection: conn_handle=%d, peer_sdu_size=%d", 
             conn_handle, peer_sdu_size);
    
    // Try to get channel info during accept to see negotiation
    if (ble_l2cap_get_chan_info(chan, &chan_info) == 0) {
        ESP_LOGI(TAG, "Channel info during accept: our_l2cap_mtu=%d, peer_l2cap_mtu=%d, our_coc_mtu=%d, peer_coc_mtu=%d",
                chan_info.our_l2cap_mtu, chan_info.peer_l2cap_mtu, 
                chan_info.our_coc_mtu, chan_info.peer_coc_mtu);
    } else {
        ESP_LOGW(TAG, "Channel info not available during accept");
    }
    
    // Allocate receive buffer from our dedicated pool
    sdu_rx = os_mbuf_get_pkthdr(&g_l2cap_coc_mbuf_pool, 0);
    if (!sdu_rx) {
        ESP_LOGE(TAG, "Failed to allocate receive buffer");
        return BLE_HS_ENOMEM;
    }
    
    // Mark channel as ready to receive with the allocated buffer
    rc = ble_l2cap_recv_ready(chan, sdu_rx);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to mark channel ready: %d", rc);
        os_mbuf_free_chain(sdu_rx);
        return rc;
    }
    
    ESP_LOGI(TAG, "L2CAP channel ready for receive");
    return 0;
}

/**
 * Handle received L2CAP data - echo or streaming mode based on config
 */
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx) {
    struct os_mbuf *sdu_tx;
    int len, rc;
    uint8_t buf[L2CAP_MTU];  // Now 1024 bytes
    
    len = OS_MBUF_PKTLEN(sdu_rx);
    ESP_LOGI(TAG, "Received %d bytes via L2CAP", len);
    
    // Update packet statistics
    g_packets_received++;
    g_bytes_received += len;
    
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    
    // Extract data from mbuf
    rc = os_mbuf_copydata(sdu_rx, 0, len, buf);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy data from mbuf");
        os_mbuf_free_chain(sdu_rx);
        return rc;
    }
    
    // Check for status query command
    if (len >= 7 && memcmp(buf, "STATUS?", 7) == 0) {
        // Report packet statistics
        sdu_tx = os_msys_get_pkthdr(0, 0);
        if (sdu_tx) {
            char status[64];
            snprintf(status, sizeof(status), "STATUS:%lu,%lu", (unsigned long)g_packets_received, (unsigned long)g_bytes_received);
            if (os_mbuf_append(sdu_tx, status, strlen(status)) == 0) {
                ble_l2cap_send(chan, sdu_tx);
                ESP_LOGI(TAG, "Sent status: %s", status);
            } else {
                os_mbuf_free_chain(sdu_tx);
            }
        }
    } else if (config_get_l2cap_audio()) {
        // Audio mode - forward data to audio playback system
        ESP_LOGI(TAG, "Audio mode: received %d bytes (total: %lu packets, %lu bytes)", 
                 len, (unsigned long)g_packets_received, (unsigned long)g_bytes_received);
        
        // Forward to audio playback system
        esp_err_t err = audio_rx_on_write(buf, len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Audio playback failed: %s", esp_err_to_name(err));
        }
    } else if (config_get_l2cap_streaming()) {
        // Streaming mode - just receive, no echo
        ESP_LOGI(TAG, "Streaming mode: received %d bytes (total: %lu packets, %lu bytes)", 
                 len, (unsigned long)g_packets_received, (unsigned long)g_bytes_received);
    } else {
        // Echo mode - send data back with prefix
        sdu_tx = os_msys_get_pkthdr(0, 0);
        if (!sdu_tx) {
            ESP_LOGE(TAG, "Failed to allocate TX mbuf");
            os_mbuf_free_chain(sdu_rx);
            return BLE_HS_ENOMEM;
        }
        
        // Create echo response with prefix - handle binary data properly
        int max_data_len = L2CAP_MTU - 6;  // Reserve 6 bytes for "ECHO: "
        if (len > max_data_len) {
            len = max_data_len;
        }
        
        uint8_t response[L2CAP_MTU];
        memcpy(response, "ECHO: ", 6);
        memcpy(response + 6, buf, len);
        int response_len = 6 + len;
        
        // Add data to mbuf
        rc = os_mbuf_append(sdu_tx, response, response_len);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to append data to mbuf");
            os_mbuf_free_chain(sdu_tx);
            os_mbuf_free_chain(sdu_rx);
            return rc;
        }
        
        // Send response
        rc = ble_l2cap_send(chan, sdu_tx);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to send L2CAP data: %d", rc);
            os_mbuf_free_chain(sdu_tx);
        } else {
            ESP_LOGI(TAG, "Echoed %d bytes back via L2CAP", response_len);
        }
    }
    
    // Free the received mbuf
    os_mbuf_free_chain(sdu_rx);
    
    // Try immediate buffer preparation first
    ESP_LOGI(TAG, "Attempting to prepare next receive buffer...");
    struct os_mbuf *next_sdu_rx = os_mbuf_get_pkthdr(&g_l2cap_coc_mbuf_pool, 0);
    if (next_sdu_rx) {
        ESP_LOGI(TAG, "Got mbuf, calling ble_l2cap_recv_ready...");
        rc = ble_l2cap_recv_ready(chan, next_sdu_rx);
        if (rc == 0) {
            ESP_LOGI(TAG, "Successfully prepared next receive buffer immediately - waiting for next packet");
            return 0;
        } else if (rc == BLE_HS_EBUSY) {
            // Defer the call using task
            ESP_LOGW(TAG, "recv_ready returned EBUSY, deferring to task");
            os_mbuf_free_chain(next_sdu_rx);
            
            recv_ready_msg_t msg = { .chan = chan };
            if (xQueueSend(recv_ready_queue, &msg, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to queue recv_ready message");
            }
            return 0;
        } else {
            ESP_LOGE(TAG, "recv_ready failed with error: %d", rc);
            os_mbuf_free_chain(next_sdu_rx);
            return rc;
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate next receive buffer - pool exhausted?");
        return BLE_HS_ENOMEM;
    }
}

esp_err_t l2cap_stream_init(void)
{
    int rc;
    
    ESP_LOGI(TAG, "Initializing L2CAP server on PSM 0x%04x", JARVIS_PSM);
    
    // Initialize memory pool for L2CAP receive buffers
    rc = os_mempool_init(&g_l2cap_coc_mempool, 16, 800, 
                         g_l2cap_coc_membuf, "l2cap_coc_pool");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP memory pool: %d", rc);
        return ESP_FAIL;
    }
    
    rc = os_mbuf_pool_init(&g_l2cap_coc_mbuf_pool, &g_l2cap_coc_mempool, 
                           800, 16);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP mbuf pool: %d", rc);
        return ESP_FAIL;
    }
    
    // Create queue and task for deferred recv_ready calls
    recv_ready_queue = xQueueCreate(8, sizeof(recv_ready_msg_t));
    if (!recv_ready_queue) {
        ESP_LOGE(TAG, "Failed to create recv_ready_queue");
        return ESP_FAIL;
    }
    
    if (xTaskCreate(recv_ready_task, "l2cap_recv_ready", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create recv_ready task");
        vQueueDelete(recv_ready_queue);
        return ESP_FAIL;
    }
    
    // Create L2CAP server
    ESP_LOGI(TAG, "Creating L2CAP server: PSM=0x%04x, MTU=%d", JARVIS_PSM, L2CAP_MTU);
    rc = ble_l2cap_create_server(JARVIS_PSM, L2CAP_MTU, l2cap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to create L2CAP server; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "L2CAP CoC server registered on PSM 0x%04x with MTU %d", JARVIS_PSM, L2CAP_MTU);
    return ESP_OK;
}

esp_err_t l2cap_stream_send(const uint8_t *data, size_t len)
{
    if (!g_channel.connected || !g_channel.chan) {
        ESP_LOGW(TAG, "No active L2CAP channel");
        return ESP_FAIL;
    }

    struct os_mbuf *om = os_msys_get_pkthdr(0, 0);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate L2CAP mbuf");
        return ESP_FAIL;
    }

    if (os_mbuf_append(om, data, len) != 0) {
        ESP_LOGE(TAG, "Failed to append data to mbuf");
        os_mbuf_free_chain(om);
        return ESP_FAIL;
    }

    int rc = ble_l2cap_send(g_channel.chan, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_l2cap_send failed; rc=%d", rc);
        os_mbuf_free_chain(om);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %d bytes via L2CAP", (int)len);
    return ESP_OK;
}

uint16_t l2cap_stream_get_psm(void)
{
    return JARVIS_PSM;
}

void l2cap_stream_reset_stats(void)
{
    g_packets_received = 0;
    g_bytes_received = 0;
    ESP_LOGI(TAG, "Reset L2CAP packet statistics");
}