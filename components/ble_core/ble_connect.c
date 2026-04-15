#include "ble_core_internal.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"

static const char *TAG = "ble_core";

/* -------------------------------------------------------------------------
 * Shared state (declared extern in ble_core_internal.h)
 * ------------------------------------------------------------------------- */
bool       s_connecting             = false;
ble_addr_t s_pending_reconnect[CONFIG_BT_NIMBLE_MAX_BONDS];
int        s_pending_count          = 0;
int        s_pending_idx            = 0;

/* -------------------------------------------------------------------------
 * Boot reconnect helpers
 *
 * On sync, ble_init.c walks the NimBLE peer-security store to collect all
 * bonded peer addresses, then calls reconnect_next() to attempt a direct
 * ble_gap_connect() to each one in sequence.  No scan phase required.
 * ------------------------------------------------------------------------- */

void reconnect_next(void)
{
    if (s_pending_idx >= s_pending_count) {
        ESP_LOGI(TAG, "Boot reconnect phase complete");
        start_scan_if_needed();
        return;
    }

    ble_addr_t *addr = &s_pending_reconnect[s_pending_idx++];
    const uint8_t *a = addr->val;

    /* Skip if already connected (shouldn't happen on boot, but be safe) */
    struct ble_gap_conn_desc existing;
    if (ble_gap_conn_find_by_addr(addr, &existing) == 0) {
        ESP_LOGI(TAG, "Already connected to %02X:%02X:%02X:%02X:%02X:%02X — skipping",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        reconnect_next();
        return;
    }

    ESP_LOGI(TAG, "Boot reconnect %d/%d — %02X:%02X:%02X:%02X:%02X:%02X",
             s_pending_idx, s_pending_count,
             a[5], a[4], a[3], a[2], a[1], a[0]);

    /* Connect directly — no scan required.
     * BLE_HS_FOREVER: waits indefinitely so a camera that is still booting
     * when the ESP32 powers on is not skipped. */
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, addr,
                             BLE_HS_FOREVER, NULL, connection_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d — skipping", rc);
        reconnect_next();
    }
}

/* -------------------------------------------------------------------------
 * Bond purge helpers
 *
 * Collects all peer-security store entries that are NOT in the keep list,
 * then deletes them after the walk completes (safe to mutate after iterate).
 * Runs on the NimBLE host task via event queue.
 * ------------------------------------------------------------------------- */

static struct {
    ble_addr_t keep[CONFIG_BT_NIMBLE_MAX_BONDS];
    int        keep_count;
} s_purge_ctx;

static ble_addr_t s_delete_list[CONFIG_BT_NIMBLE_MAX_BONDS];
static int        s_delete_count;

static int collect_purge_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    ble_addr_t *addr = &val->sec.peer_addr;

    for (int i = 0; i < s_purge_ctx.keep_count; i++) {
        if (memcmp(&s_purge_ctx.keep[i], addr, sizeof(ble_addr_t)) == 0) {
            return 0; /* on the keep list — retain */
        }
    }

    if (s_delete_count < CONFIG_BT_NIMBLE_MAX_BONDS) {
        s_delete_list[s_delete_count++] = *addr;
    }
    return 0;
}

static int count_bonds_cb(int obj_type, union ble_store_value *val, void *cookie)
{
    (*(int *)cookie)++;
    return 0;
}

static void log_bond_count(void)
{
    int count = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, count_bonds_cb, &count);
    ESP_LOGI(TAG, "Stored bonds: %d / %d", count, CONFIG_BT_NIMBLE_MAX_BONDS);
}

static void purge_bonds_cb(struct ble_npl_event *ev)
{
    /* Cancel any in-flight ble_gap_connect() so the camera cannot slip in and
     * connect between now and the bond deletion below.  ble_gap_connect_cancel()
     * is a no-op if no connection attempt is pending.  If it does cancel one,
     * NimBLE fires BLE_GAP_EVENT_CONNECT with a non-zero status, which our
     * handler treats as a failure and calls reconnect_next() / start_scan()
     * without re-entering the connect loop (camera_manager will have been
     * cleared already by the time that event fires). */
    ble_gap_conn_cancel();

    log_bond_count();

    s_delete_count = 0;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, collect_purge_cb, NULL);

    for (int i = 0; i < s_delete_count; i++) {
        const uint8_t *a = s_delete_list[i].val;
        ESP_LOGI(TAG, "Removing bond: %02X:%02X:%02X:%02X:%02X:%02X",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        ble_store_util_delete_peer(&s_delete_list[i]);
    }

    if (s_delete_count > 0) {
        ESP_LOGI(TAG, "Purged %d bond(s)", s_delete_count);
        log_bond_count();
    }
}

static struct ble_npl_event s_purge_event;

void ble_core_purge_unknown_bonds(const ble_addr_t *keep, int keep_count)
{
    int n = keep_count < CONFIG_BT_NIMBLE_MAX_BONDS
          ? keep_count : CONFIG_BT_NIMBLE_MAX_BONDS;
    memset(&s_purge_ctx, 0, sizeof(s_purge_ctx));
    if (keep && keep_count > 0) {
        for (int i = 0; i < n; i++) {
            s_purge_ctx.keep[i] = keep[i];
        }
    }
    s_purge_ctx.keep_count = n;

    ble_npl_event_init(&s_purge_event, purge_bonds_cb, NULL);
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_purge_event);
}

/* -------------------------------------------------------------------------
 * GAP connection event callback
 *
 * Fires on_connected, on_encrypted, and on_disconnected callbacks so the
 * gopro_ble layer can handle GoPro-specific logic without ble_core knowing
 * about it.
 * ------------------------------------------------------------------------- */

int connection_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            uint16_t handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected — handle: %d", handle);

            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(handle, &desc);

            if (g_ble_core_cbs.on_connected) {
                g_ble_core_cbs.on_connected(handle, &desc.peer_id_addr);
            }

            /* Initiate security (re-uses saved keys for known cameras, or
             * performs first-time pairing for new ones). */
            struct ble_store_key_sec key;
            struct ble_store_value_sec sec;
            memset(&key, 0, sizeof(key));
            key.peer_addr = desc.peer_id_addr;
            bool bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            if (!bonded) {
                key.peer_addr = desc.peer_ota_addr;
                bonded = (ble_store_read_peer_sec(&key, &sec) == 0);
            }

            if (bonded) {
                ESP_LOGI(TAG, "Known camera — restoring encryption with saved keys");
            } else {
                ESP_LOGI(TAG, "New camera — initiating first-time pairing");
            }

            int sec_rc = ble_gap_security_initiate(handle);
            if (sec_rc != 0) {
                ESP_LOGE(TAG, "Security initiate failed: %d", sec_rc);
            }
        } else {
            ESP_LOGE(TAG, "Connection failed — status: %d", event->connect.status);
        }

        /* Advance the boot reconnect chain regardless of success/failure.
         * Once the chain is exhausted, reconnect_next() becomes a no-op
         * start_scan(). */
        s_connecting = false;
        reconnect_next();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            uint16_t enc_handle = event->enc_change.conn_handle;
            ESP_LOGI(TAG, "Encryption established — handle: %d", enc_handle);

            struct ble_gap_conn_desc enc_desc;
            if (ble_gap_conn_find(enc_handle, &enc_desc) == 0) {
                if (g_ble_core_cbs.on_encrypted) {
                    g_ble_core_cbs.on_encrypted(enc_handle, &enc_desc.peer_id_addr);
                }
            }
        } else {
            ESP_LOGE(TAG, "Pairing/encryption failed — status: %d",
                     event->enc_change.status);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Stale bond — delete and retry so the camera can re-pair */
        ESP_LOGW(TAG, "Repeat pairing — deleting stale bond and re-pairing");
        struct ble_gap_conn_desc rp_desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &rp_desc);
        ble_store_util_delete_peer(&rp_desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t conn_h = event->disconnect.conn.conn_handle;
        const ble_addr_t *peer = &event->disconnect.conn.peer_id_addr;
        int reason = event->disconnect.reason;

        ESP_LOGI(TAG, "Disconnected — reason: %d", reason);

        /* Notify higher layers first so they can still look up the slot by
         * conn_handle before it is cleared. */
        if (g_ble_core_cbs.on_disconnected) {
            g_ble_core_cbs.on_disconnected(conn_h, peer, reason);
        }

        s_connecting = false;

        /* Only reconnect if this address is still known to the higher layer
         * (e.g. it was not removed via a bond reset).  is_known_addr is
         * implemented by camera_manager_is_known_addr() but kept here as a
         * callback so ble_core stays camera-agnostic. */
        if (g_ble_core_cbs.is_known_addr && !g_ble_core_cbs.is_known_addr(peer)) {
            const uint8_t *a = peer->val;
            ESP_LOGI(TAG, "Disconnected peer %02X:%02X:%02X:%02X:%02X:%02X is no longer "
                     "known — skipping reconnect", a[5], a[4], a[3], a[2], a[1], a[0]);
            start_scan_if_needed();
            break;
        }

        /* Cancel any active scan — ble_gap_connect() returns BLE_HS_EBUSY
         * if a scan is already running. */
        ble_gap_disc_cancel();

        /* Attempt direct reconnect — BLE_HS_FOREVER: the controller stays in
         * initiating state until the peer starts advertising, however long
         * that takes. */
        const uint8_t *a = peer->val;
        ESP_LOGI(TAG, "Attempting reconnect to %02X:%02X:%02X:%02X:%02X:%02X",
                 a[5], a[4], a[3], a[2], a[1], a[0]);

        int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, peer,
                                 BLE_HS_FOREVER, NULL, connection_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Post-disconnect reconnect failed: %d — scanning", rc);
            start_scan();
        }
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t conn_h = event->notify_rx.conn_handle;
        uint16_t attr_h = event->notify_rx.attr_handle;

        if (g_ble_core_cbs.on_notify_rx) {
            struct os_mbuf *om = event->notify_rx.om;
            uint16_t pkt_len  = OS_MBUF_PKTLEN(om);
            /* 512 bytes matches the maximum ATT MTU after negotiation.
             * The previous limit of 64 bytes silently dropped long responses
             * such as GetHardwareInfoRsp (~88 bytes). */
            if (pkt_len > 0 && pkt_len <= 512) {
                uint8_t buf[512];
                os_mbuf_copydata(om, 0, pkt_len, buf);
                g_ble_core_cbs.on_notify_rx(conn_h, attr_h, buf, pkt_len);
            } else if (pkt_len > 512) {
                ESP_LOGW(TAG, "notify_rx: pkt_len=%d exceeds buffer — dropped "
                         "(conn=%d attr=0x%04x)", pkt_len, conn_h, attr_h);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGW(TAG, "Passkey action requested — action: %d",
                 event->passkey.params.action);
        break;

    default:
        break;
    }

    return 0;
}
