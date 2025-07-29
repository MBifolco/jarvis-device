#include "l2cap_stream.h"
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "nimble/nimble_port.h"

#define TAG "L2CAP"
#define JARVIS_PSM 0x0040
#define L2CAP_MTU  512

// Channel structure to track connections
typedef struct {
    struct ble_l2cap_chan *chan;
    uint16_t conn_handle;
    bool connected;
} l2cap_channel_t;

static l2cap_channel_t g_channel = {0};

// Forward declarations
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx);
static int l2cap_server_accept(uint16_t conn_handle, uint16_t peer_sdu_size, struct ble_l2cap_chan *chan);

static int l2cap_event_cb(struct ble_l2cap_event *event, void *arg)
{
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
    int rc;
    
    ESP_LOGI(TAG, "Accepting L2CAP connection: conn_handle=%d, peer_sdu_size=%d", 
             conn_handle, peer_sdu_size);
    
    // Allocate receive buffer from system pool
    sdu_rx = os_msys_get_pkthdr(0, 0);
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
 * Handle received L2CAP data and echo it back with prefix
 */
static int l2cap_server_recv(struct ble_l2cap_chan *chan, struct os_mbuf *sdu_rx) {
    struct os_mbuf *sdu_tx;
    int len, rc;
    uint8_t buf[L2CAP_MTU];
    
    len = OS_MBUF_PKTLEN(sdu_rx);
    ESP_LOGI(TAG, "Received %d bytes via L2CAP", len);
    
    if (len > sizeof(buf) - 7) {  // Reserve space for "ECHO: " prefix
        len = sizeof(buf) - 7;
    }
    
    // Extract data from mbuf
    rc = os_mbuf_copydata(sdu_rx, 0, len, buf);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy data from mbuf");
        os_mbuf_free_chain(sdu_rx);
        return rc;
    }
    
    // Create response mbuf
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
    
    // Free the received mbuf - the stack will handle buffer management
    os_mbuf_free_chain(sdu_rx);
    
    // Prepare next receive buffer - this is critical for continuous operation
    struct os_mbuf *next_sdu_rx = os_msys_get_pkthdr(0, 0);
    if (next_sdu_rx) {
        int ready_rc = ble_l2cap_recv_ready(chan, next_sdu_rx);
        if (ready_rc != 0) {
            ESP_LOGE(TAG, "Failed to prepare next receive buffer: %d", ready_rc);
            os_mbuf_free_chain(next_sdu_rx);
        }
    } else {
        ESP_LOGE(TAG, "Failed to allocate next receive buffer");
    }
    
    return 0;
}

esp_err_t l2cap_stream_init(void)
{
    int rc = ble_l2cap_create_server(JARVIS_PSM, L2CAP_MTU, l2cap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to create L2CAP server; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "L2CAP CoC server registered on PSM 0x%04x", JARVIS_PSM);
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