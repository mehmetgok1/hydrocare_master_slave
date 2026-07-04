/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"

/* Private function declarations */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */

/* Custom Service UUID */
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);

/* Characteristic UUIDs */
static const ble_uuid128_t gatt_bat_uuid =
    BLE_UUID128_INIT(0x12, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_lux_uuid =
    BLE_UUID128_INIT(0x13, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_pir_uuid =
    BLE_UUID128_INIT(0x14, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_mmwave_uuid =
    BLE_UUID128_INIT(0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_action_uuid =
    BLE_UUID128_INIT(0x16, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_version_uuid =
    BLE_UUID128_INIT(0x17, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_amb_int_uuid =
    BLE_UUID128_INIT(0x18, 0x11, 0x11, 0x11, 0x11, 0x11, 0x22, 0x22,
                     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static const ble_uuid128_t gatt_rgb_uuid =
    BLE_UUID128_INIT(0x46, 0xa5, 0xf6, 0x86, 0x60, 0x5e, 0xe7, 0x99,
                     0x08, 0x4e, 0xe9, 0x16, 0xf6, 0x69, 0xa9, 0xc2);
static const ble_uuid128_t gatt_ir_uuid =
    BLE_UUID128_INIT(0x47, 0xa5, 0xf6, 0x86, 0x60, 0x5e, 0xe7, 0x99,
                     0x08, 0x4e, 0xe9, 0x16, 0xf6, 0x69, 0xb9, 0xd3);


/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Custom Service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &gatt_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {.uuid = &gatt_version_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ},
             {.uuid = &gatt_bat_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_lux_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_pir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_mmwave_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_amb_int_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_action_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_WRITE},
             {.uuid = &gatt_rgb_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {.uuid = &gatt_ir_uuid.u,
              .access_cb = gatt_svr_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
             {0} /* No more characteristics in this service */
         }},
    {
        0, /* No more services. */
    },
};

/* Private functions */
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Characteristic read; conn_handle=%d, attr_handle=%d",
                 conn_handle, attr_handle);
        // Here you would typically fetch the current value of the characteristic
        // and append it to ctxt->om. For now, this is a placeholder.
        // For example, for the version characteristic:
        // if (ble_uuid_cmp(&ctxt->chr->uuid, &gatt_version_uuid.u) == 0) {
        //     os_mbuf_append(ctxt->om, "1.0.0", strlen("1.0.0"));
        // }
        return 0;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "Characteristic write; conn_handle=%d, attr_handle=%d",
                 conn_handle, attr_handle);
        // This is where you would handle incoming commands, similar to your
        // ActionCallbacks::onWrite method. You can inspect the UUID to know
        // which characteristic was written to.
        // For example:
        // if (ble_uuid_cmp(&ctxt->chr->uuid, &gatt_action_uuid.u) == 0) {
        //     // process command in ctxt->om
        // }
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* Public functions */
void send_heart_rate_indication(void) {
    // This function is now obsolete with the new service structure.
    // You will need a new mechanism to send notifications for your custom characteristics.
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
             "reason=%d prevn=%d curn=%d previ=%d curi=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle,
             event->subscribe.reason, event->subscribe.prev_notify,
             event->subscribe.cur_notify, event->subscribe.prev_indicate,
             event->subscribe.cur_indicate);
}

void gatt_svr_reset_heart_rate_subscription(void) {
    // This function is now obsolete.
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc = 0;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
