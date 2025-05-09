#include "l2cap_stream.h"
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "os/os_mbuf.h"

#define TAG "L2CAP"
#define JARVIS_PSM 0x0040
#define L2CAP_MTU  512

static struct ble_l2cap_chan *audio_chan = NULL;

static int l2cap_event_cb(struct ble_l2cap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:
            ESP_LOGI(TAG, "L2CAP channel connected");
            audio_chan = event->connect.chan;
            break;

        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            ESP_LOGI(TAG, "L2CAP channel disconnected");
            audio_chan = NULL;
            break;

        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            ESP_LOGI(TAG, "Received %d bytes via L2CAP (discarded)",
                     OS_MBUF_PKTLEN(event->receive.sdu_rx));
            os_mbuf_free_chain(event->receive.sdu_rx);
            break;

        default:
            ESP_LOGW(TAG, "Unhandled L2CAP event: %d", event->type);
            break;
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
    if (!audio_chan) {
        ESP_LOGW(TAG, "No active L2CAP channel");
        return ESP_FAIL;
    }

    struct os_mbuf *om = os_mbuf_get_pkthdr(NULL, 0);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate L2CAP mbuf");
        return ESP_FAIL;
    }

    if (os_mbuf_append(om, data, len) != 0) {
        ESP_LOGE(TAG, "Failed to append data to mbuf");
        os_mbuf_free_chain(om);
        return ESP_FAIL;
    }

    int rc = ble_l2cap_send(audio_chan, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_l2cap_send failed; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %d bytes via L2CAP", (int)len);
    return ESP_OK;
}
