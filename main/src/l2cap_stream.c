// l2cap_stream.c

#include "l2cap_stream.h"
#include "common.h"            // for AUDIO_L2CAP_PSM, AUDIO_L2CAP_MTU, etc.
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "os/os_mbuf.h"

#define TAG "L2CAP"

static struct ble_l2cap_chan *audio_chan = NULL;
static uint16_t      local_credits = AUDIO_L2CAP_INITIAL_CREDITS;

/**
 * L2CAP event callback: handles connection, disconnection, and received data.
 */
static int l2cap_event_cb(struct ble_l2cap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:
            ESP_LOGI(TAG, "L2CAP channel connected (cid=%u)",
                     event->connect.chan->chan_id);
            audio_chan = event->connect.chan;
            // Send initial credits to peer
            ble_l2cap_credits_pending(audio_chan->chan_id, AUDIO_L2CAP_INITIAL_CREDITS);
            local_credits = AUDIO_L2CAP_INITIAL_CREDITS;
            break;

        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            ESP_LOGI(TAG, "L2CAP channel disconnected (cid=%u)",
                     event->disconnect.chan->chan_id);
            audio_chan = NULL;
            break;

        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            {
                int len = OS_MBUF_PKTLEN(event->receive.sdu_rx);
                ESP_LOGI(TAG, "Received %d bytes via L2CAP (discarded)", len);
                os_mbuf_free_chain(event->receive.sdu_rx);
                // Replenish credits so peer can keep sending if needed
                ble_l2cap_credits_pending(audio_chan->chan_id, 1);
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled L2CAP event: %d", event->type);
            break;
    }
    return 0;
}

/**
 * Initialize the L2CAP CoC server on our AUDIO_L2CAP_PSM.
 */
esp_err_t l2cap_stream_init(void)
{
    int rc = ble_l2cap_create_server(
        AUDIO_L2CAP_PSM,
        AUDIO_L2CAP_MTU,
        l2cap_event_cb,
        NULL
    );
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to create L2CAP server on PSM 0x%04X; rc=%d",
                 AUDIO_L2CAP_PSM, rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "L2CAP CoC server registered on PSM 0x%04X, MTU=%d",
             AUDIO_L2CAP_PSM, AUDIO_L2CAP_MTU);
    return ESP_OK;
}

/**
 * Send a buffer of audio data over the established L2CAP channel,
 * fragmenting it into MTU-sized chunks.
 */
esp_err_t l2cap_stream_send(const uint8_t *data, size_t len)
{
    if (!audio_chan) {
        ESP_LOGW(TAG, "No active L2CAP channel");
        return ESP_FAIL;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = MIN(AUDIO_L2CAP_MTU, len - offset);
        struct os_mbuf *om = os_mbuf_get_pkthdr(NULL, 0);
        if (!om) {
            ESP_LOGE(TAG, "Failed to allocate mbuf");
            return ESP_FAIL;
        }

        if (os_mbuf_append(om, data + offset, chunk) != 0) {
            ESP_LOGE(TAG, "Failed to append %u bytes to mbuf", (unsigned)chunk);
            os_mbuf_free_chain(om);
            return ESP_FAIL;
        }

        int rc = ble_l2cap_send(audio_chan, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_l2cap_send failed; rc=%d", rc);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Sent %u bytes via L2CAP", (unsigned)chunk);
        offset += chunk;
    }

    return ESP_OK;
}

/**
 * Close the L2CAP channel (if open).
 */
void l2cap_stream_close(void)
{
    if (audio_chan) {
        ble_l2cap_disconnect(audio_chan);
        audio_chan = NULL;
    }
}
