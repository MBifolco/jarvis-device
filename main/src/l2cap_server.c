/**
 * L2CAP Connection-Oriented Channel (COC) Server
 * Simple text echo server for testing L2CAP implementation
 */

#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_l2cap.h"
#include "host/util/util.h"
#include "os/os_mbuf.h"

// Memory pool for L2CAP receive buffers
static struct os_mbuf_pool g_l2cap_coc_mbuf_pool;
static struct os_mempool g_l2cap_coc_mempool;
static os_membuf_t g_l2cap_coc_membuf[OS_MEMPOOL_SIZE(4, 512)]; // 4 buffers of 512 bytes each

static const char *TAG = "L2CAP_SERVER";

// PSM (Protocol Service Multiplexer) for our L2CAP service
// Using dynamic PSM range 0x80-0xFF
#define L2CAP_PSM 0x80

// L2CAP COC MTU
#define L2CAP_COC_MTU 512

// Forward declarations
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx);
static int l2cap_server_event(struct ble_l2cap_event *event, void *arg);
static int l2cap_server_accept(uint16_t conn_handle, uint16_t peer_sdu_size, struct ble_l2cap_chan *chan);

// Channel structure to track connections
typedef struct {
    struct ble_l2cap_chan *chan;
    uint16_t conn_handle;
    bool connected;
} l2cap_channel_t;

static l2cap_channel_t g_channel = {0};

/**
 * L2CAP event handler
 */
static int l2cap_server_event(struct ble_l2cap_event *event, void *arg) {
    struct ble_l2cap_chan_info chan_info;
    
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:
            ESP_LOGI(TAG, "L2CAP channel connected");
            if (ble_l2cap_get_chan_info(event->connect.chan, &chan_info) == 0) {
                ESP_LOGI(TAG, "Channel info: our_mtu=%d, peer_mtu=%d, psm=%d",
                        chan_info.our_l2cap_mtu, chan_info.peer_l2cap_mtu, chan_info.psm);
            }
            g_channel.chan = event->connect.chan;
            g_channel.conn_handle = event->connect.conn_handle;
            g_channel.connected = true;
            break;
            
        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            ESP_LOGI(TAG, "L2CAP channel disconnected");
            g_channel.connected = false;
            g_channel.chan = NULL;
            break;
            
        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            ESP_LOGI(TAG, "L2CAP data received");
            l2cap_server_recv(event->receive.chan, event->receive.sdu_rx);
            break;
            
        case BLE_L2CAP_EVENT_COC_ACCEPT:
            ESP_LOGI(TAG, "L2CAP accept request");
            return l2cap_server_accept(event->accept.conn_handle, 
                                     event->accept.peer_sdu_size, 
                                     event->accept.chan);
            
        case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
            ESP_LOGI(TAG, "L2CAP TX unstalled - can send more data");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown L2CAP event type: %d", event->type);
            break;
    }
    
    return 0;
}

/**
 * Handle received data - echo it back
 */
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx) {
    int rc;
    uint16_t len;
    uint8_t buf[L2CAP_COC_MTU];
    struct os_mbuf *sdu_tx;
    
    if (!chan || !sdu_rx) {
        return BLE_HS_EINVAL;
    }
    
    // Get data length
    len = OS_MBUF_PKTLEN(sdu_rx);
    ESP_LOGI(TAG, "Received %d bytes", len);
    
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    
    // Copy data from mbuf to buffer
    rc = os_mbuf_copydata(sdu_rx, 0, len, buf);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy mbuf data");
        return rc;
    }
    
    // Log received text (assume it's text for testing)
    if (len < sizeof(buf)) {
        buf[len] = '\0';  // Null terminate for printing
        ESP_LOGI(TAG, "Received text: '%s'", (char*)buf);
    } else {
        ESP_LOGI(TAG, "Received %d bytes (truncated)", len);
    }
    
    // Create response mbuf
    sdu_tx = os_msys_get_pkthdr(0, 0);
    if (!sdu_tx) {
        ESP_LOGE(TAG, "Failed to allocate TX mbuf");
        return BLE_HS_ENOMEM;
    }
    
    // Create echo response with prefix
    char response[L2CAP_COC_MTU];
    int response_len = snprintf(response, sizeof(response), "ECHO: %.*s", 
                               (int)(sizeof(response) - 7), (char*)buf);
    if (response_len >= sizeof(response)) {
        response_len = sizeof(response) - 1;
    }
    
    // Add data to mbuf
    rc = os_mbuf_append(sdu_tx, response, response_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to append data to mbuf");
        os_mbuf_free_chain(sdu_tx);
        return rc;
    }
    
    // Send response
    rc = ble_l2cap_send(chan, sdu_tx);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send L2CAP data: %d", rc);
        os_mbuf_free_chain(sdu_tx);
        return rc;
    }
    
    ESP_LOGI(TAG, "Sent echo response: '%s'", response);
    
    // Free the received mbuf
    os_mbuf_free_chain(sdu_rx);
    
    // Prepare for next receive - allocate new buffer and mark ready
    struct os_mbuf *next_sdu_rx = os_mbuf_get_pkthdr(&g_l2cap_coc_mbuf_pool, 0);
    if (next_sdu_rx) {
        rc = ble_l2cap_recv_ready(chan, next_sdu_rx);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to prepare next receive buffer: %d", rc);
            os_mbuf_free_chain(next_sdu_rx);
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate next receive buffer");
    }
    
    return 0;
}

/**
 * Handle L2CAP accept request and prepare receive buffer
 */
static int l2cap_server_accept(uint16_t conn_handle, uint16_t peer_sdu_size, struct ble_l2cap_chan *chan) {
    struct os_mbuf *sdu_rx;
    int rc;
    
    ESP_LOGI(TAG, "Accepting L2CAP connection: conn_handle=%d, peer_sdu_size=%d", 
             conn_handle, peer_sdu_size);
    
    // Allocate receive buffer from our pool
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
    
    ESP_LOGI(TAG, "L2CAP channel ready to receive data");
    return 0;
}

/**
 * Initialize L2CAP server
 */
int l2cap_server_init(void) {
    int rc;
    
    ESP_LOGI(TAG, "Initializing L2CAP server on PSM 0x%02x", L2CAP_PSM);
    
    // Initialize memory pool for L2CAP receive buffers
    rc = os_mempool_init(&g_l2cap_coc_mempool, 4, 512, 
                         g_l2cap_coc_membuf, "l2cap_coc_pool");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP memory pool: %d", rc);
        return rc;
    }
    
    rc = os_mbuf_pool_init(&g_l2cap_coc_mbuf_pool, &g_l2cap_coc_mempool, 
                           512, 4);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP mbuf pool: %d", rc);
        return rc;
    }
    
    // Create L2CAP server
    rc = ble_l2cap_create_server(L2CAP_PSM, L2CAP_COC_MTU, l2cap_server_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to create L2CAP server: %d", rc);
        return rc;
    }
    
    ESP_LOGI(TAG, "L2CAP server created successfully");
    
    return 0;
}

/**
 * Send text message over L2CAP
 */
int l2cap_server_send_text(const char *text) {
    struct os_mbuf *sdu_tx;
    int rc;
    
    if (!g_channel.connected || !g_channel.chan) {
        ESP_LOGW(TAG, "L2CAP channel not connected");
        return -1;
    }
    
    // Create mbuf for transmission
    sdu_tx = os_msys_get_pkthdr(0, 0);
    if (!sdu_tx) {
        ESP_LOGE(TAG, "Failed to allocate TX mbuf");
        return BLE_HS_ENOMEM;
    }
    
    // Add text to mbuf
    rc = os_mbuf_append(sdu_tx, text, strlen(text));
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to append data to mbuf");
        os_mbuf_free_chain(sdu_tx);
        return rc;
    }
    
    // Send data
    rc = ble_l2cap_send(g_channel.chan, sdu_tx);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send L2CAP data: %d", rc);
        os_mbuf_free_chain(sdu_tx);
        return rc;
    }
    
    ESP_LOGI(TAG, "Sent text: '%s'", text);
    
    return 0;
}

/**
 * Get L2CAP PSM for advertising via GATT
 */
uint16_t l2cap_server_get_psm(void) {
    return L2CAP_PSM;
}

/**
 * Check if L2CAP channel is connected
 */
bool l2cap_server_is_connected(void) {
    return g_channel.connected;
}