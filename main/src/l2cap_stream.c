#include "l2cap_stream.h"
#include "esp_log.h"
#include "host/ble_l2cap.h"
#include "os/os_mbuf.h"

#define TAG "L2CAP"
#define JARVIS_PSM 0x0040  // Custom PSM for your L2CAP service

static struct ble_l2cap_chan *audio_chan = NULL;

static int on_l2cap_accept(uint16_t psm, struct ble_l2cap_chan **chan)
{
    ESP_LOGI(TAG, "L2CAP accept on PSM 0x%04x", psm);
    *chan = ble_l2cap_chan_new();
    return *chan ? 0 : BLE_HS_ENOMEM;
}

static int on_l2cap_receive(struct ble_l2cap_chan *chan, struct os_mbuf **sdu)
{
    ESP_LOGI(TAG, "Received %d bytes via L2CAP", OS_MBUF_PKTLEN(*sdu));
    os_mbuf_free_chain(*sdu);
    return 0;
}

int l2cap_stream_init(void)
{
    int rc = ble_l2cap_register_psm(JARVIS_PSM, on_l2cap_accept, on_l2cap_receive, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "L2CAP PSM 0x%04x registered", JARVIS_PSM);
        return rc;
    } else {
        ESP_LOGE(TAG, "Failed to register PSM; rc=%d", rc);
        return 0;
    }
}

esp_err_t l2cap_stream_send(const uint8_t *data, size_t len)
{
    if (!audio_chan || !audio_chan->coc_tx_mbuf) {
        ESP_LOGW(TAG, "No active L2CAP channel");
        return ESP_FAIL;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return ESP_FAIL;

    int rc = ble_l2cap_send(audio_chan, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send over L2CAP; rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %d bytes over L2CAP", (int)len);
    return ESP_OK;
}
