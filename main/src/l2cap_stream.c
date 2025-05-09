#include "l2cap_stream.h"
#include "common.h"
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "os/os_mbuf.h"

#undef TAG
#define TAG "L2CAP"

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

static struct ble_l2cap_chan *audio_chan = NULL;

// L2CAP event handler
static int l2cap_event_cb(struct ble_l2cap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:
            ESP_LOGI(TAG, "L2CAP connected");
            audio_chan = event->connect.chan;
            break;

        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            ESP_LOGI(TAG, "L2CAP disconnected");
            audio_chan = NULL;
            break;

        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            ESP_LOGI(TAG, "Received %u bytes (discarded)",
                     (unsigned)OS_MBUF_PKTLEN(event->receive.sdu_rx));
            os_mbuf_free_chain(event->receive.sdu_rx);
            break;

        default:
            ESP_LOGW(TAG, "Unhandled event %d", event->type);
            break;
    }
    return 0;
}

esp_err_t l2cap_stream_init(void)
{
    int rc = ble_l2cap_create_server(
        AUDIO_L2CAP_PSM,
        AUDIO_L2CAP_MTU,
        l2cap_event_cb,
        NULL
    );
    if (rc) {
        ESP_LOGE(TAG, "ble_l2cap_create_server rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "L2CAP server PSM=0x%04X MTU=%d",
             AUDIO_L2CAP_PSM, AUDIO_L2CAP_MTU);
    return ESP_OK;
}

bool l2cap_stream_is_connected(void)
{
    return audio_chan != NULL;
}

esp_err_t l2cap_stream_send(const uint8_t *data, size_t len)
{
    if (!audio_chan) {
        ESP_LOGW(TAG, "No L2CAP channel");
        return ESP_FAIL;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = MIN(AUDIO_L2CAP_MTU, len - offset);
        struct os_mbuf *om = os_mbuf_get_pkthdr(NULL, 0);
        if (!om) {
            ESP_LOGE(TAG, "os_mbuf_get_pkthdr failed");
            return ESP_FAIL;
        }
        if (os_mbuf_append(om, data + offset, chunk)) {
            ESP_LOGE(TAG, "os_mbuf_append %u bytes failed", (unsigned)chunk);
            os_mbuf_free_chain(om);
            return ESP_FAIL;
        }
        int rc = ble_l2cap_send(audio_chan, om);
        if (rc) {
            ESP_LOGE(TAG, "ble_l2cap_send rc=%d", rc);
            return ESP_FAIL;
        }
        offset += chunk;
        ESP_LOGI(TAG, "Sent %u bytes", (unsigned)chunk);
    }
    return ESP_OK;
}

void l2cap_stream_close(void)
{
    if (audio_chan) {
        ble_l2cap_disconnect(audio_chan);
        audio_chan = NULL;
    }
}
