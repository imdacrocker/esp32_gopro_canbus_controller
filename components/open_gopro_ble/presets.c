/*
 * presets.c — OpenGoPro preset commands.
 *
 * Implements the two-phase flow recommended by the OpenGoPro spec to load
 * the Video preset on every connection:
 *
 *   Phase 1 — gopro_presets_request_video()
 *     Send RequestGetPresetStatus (Protobuf, Feature 0xF5, Action 0x72) to
 *     GP-0076 (query_write).  The camera responds with NotifyPresetStatus on
 *     GP-0077 (query_resp_notify).
 *
 *   Phase 2 — parse NotifyPresetStatus and send Load Preset
 *     Handled by gopro_presets_handle_notify_status() for single-packet
 *     responses and gopro_presets_handle_query_fragment() for fragmented ones.
 *     Both ultimately call load_first_video_preset(), which parses the Protobuf
 *     payload and sends Load Preset (TLV 0x40) to GP-0072 (cmd_write).
 *
 * GPBS fragmentation note
 * =======================
 * The OpenGoPro spec states that cameras fragment GPBS responses at the
 * application layer regardless of the negotiated ATT MTU.  NotifyPresetStatus
 * for a HERO13 Black (which can have many custom presets) WILL arrive as a
 * first-fragment (frame_type=0b001) followed by continuation fragments
 * (frame_type >= 0b010).
 *
 * Reassembly follows the same pattern as GetHardwareInfo in query.c:
 *   - First fragment:   2-byte GPBS header, payload starts at byte 2.
 *                       Payload = [Feature ID][Action ID][Protobuf...]
 *                       13-bit total length = ((data[0] & 0x1F) << 8) | data[1]
 *   - Continuation:     1-byte GPBS header (sequence byte), payload at byte 1.
 *
 * Per-connection reassembly contexts (preset_rx_ctx_t) mirror query_ctx_t in
 * query.c.  gopro_presets_free() must be called on disconnect to release them.
 *
 * Load Preset packet format
 * =========================
 * Written to GP-0072 (cmd_write) once the target preset ID is known.
 *   Byte 0: 0x06              GPBS single-packet length (6 bytes follow)
 *   Byte 1: 0x40              Load Preset command ID
 *   Byte 2: 0x04              parameter length (4 bytes, uint32)
 *   Bytes 3–6: big-endian uint32 — Preset ID from NotifyPresetStatus
 *
 * The camera's Load Preset (0x40) response arrives on GP-0073
 * (cmd_resp_notify) and is handled by the 0x40 case in
 * gopro_query_handle_cmd_response() (query.c).
 */

#include "open_gopro_ble_internal.h"

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "camera_manager.h"
#include "ble_core.h"

static const char *TAG = "open_gopro_ble";

/* EnumPresetGroup value for Video (HERO9 Black through HERO13 Black). */
#define PRESET_GROUP_VIDEO    1000

/* Feature / Action IDs for the Preset Protobuf API. */
#define PRESET_FEATURE_ID     0xF5
#define PRESET_ACTION_REQ     0x72
#define PRESET_ACTION_RESP    0xF2

/* Maximum size of a reassembled Preset response payload (Feature + Action +
 * Protobuf).  A HERO13 Black with custom presets can produce ~756 bytes. */
#define PRESET_RX_BUF_SIZE    1024

/* -------------------------------------------------------------------------
 * Per-connection reassembly context for multi-packet GP-0077 responses
 * Mirrors query_ctx_t in query.c.
 * ------------------------------------------------------------------------- */

typedef struct {
    bool     in_use;
    uint16_t conn_handle;
    bool     rx_assembling;
    uint16_t rx_total;
    uint16_t rx_filled;
    uint8_t  rx_buf[PRESET_RX_BUF_SIZE];
} preset_rx_ctx_t;

static preset_rx_ctx_t s_rx_ctx[CONFIG_BT_NIMBLE_MAX_BONDS];

static preset_rx_ctx_t *find_rx_ctx(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_rx_ctx[i].in_use && s_rx_ctx[i].conn_handle == conn_handle) {
            return &s_rx_ctx[i];
        }
    }
    return NULL;
}

static preset_rx_ctx_t *alloc_rx_ctx(uint16_t conn_handle)
{
    preset_rx_ctx_t *ctx = find_rx_ctx(conn_handle);
    if (ctx) {
        return ctx;
    }
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (!s_rx_ctx[i].in_use) {
            memset(&s_rx_ctx[i], 0, sizeof(s_rx_ctx[i]));
            s_rx_ctx[i].in_use      = true;
            s_rx_ctx[i].conn_handle = conn_handle;
            return &s_rx_ctx[i];
        }
    }
    return NULL;
}

void gopro_presets_free(uint16_t conn_handle)
{
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_BONDS; i++) {
        if (s_rx_ctx[i].conn_handle == conn_handle) {
            memset(&s_rx_ctx[i], 0, sizeof(s_rx_ctx[i]));
        }
    }
}

/* -------------------------------------------------------------------------
 * Minimal Protobuf wire decoder
 * ------------------------------------------------------------------------- */

/*
 * Read a Protobuf varint from buf[0..len-1].
 * Returns bytes consumed, or -1 on truncation/overflow.
 */
static int pb_read_varint(const uint8_t *buf, int len, uint64_t *out)
{
    uint64_t val   = 0;
    int      shift = 0;

    for (int i = 0; i < len && i < 10; i++) {
        val |= (uint64_t)(buf[i] & 0x7F) << shift;
        shift += 7;
        if (!(buf[i] & 0x80)) {
            *out = val;
            return i + 1;
        }
    }
    return -1;
}

/*
 * Return the total byte size of a field value at buf[0..len-1] for the given
 * Protobuf wire type.  For wire type 2 (length-delimited), this includes the
 * varint length prefix and the content bytes.  Returns -1 on error.
 */
static int pb_value_size(uint8_t wire_type, const uint8_t *buf, int len)
{
    switch (wire_type) {
    case 0: {
        uint64_t dummy;
        return pb_read_varint(buf, len, &dummy);
    }
    case 1:
        return (len >= 8) ? 8 : -1;
    case 2: {
        uint64_t sz;
        int n = pb_read_varint(buf, len, &sz);
        if (n < 0 || sz > (uint64_t)(len - n)) return -1;
        return n + (int)sz;
    }
    case 5:
        return (len >= 4) ? 4 : -1;
    default:
        return -1;
    }
}

/*
 * Parse a single PresetGroup Protobuf message (grp[0..grp_len-1]).
 *
 * Scans for field 1 (group id, varint) and field 2 (preset_array entries).
 * If the group id matches video_group_id and the group has at least one
 * preset, writes that preset's id (Preset field 1) to *first_preset_id and
 * returns true.  Returns false otherwise.
 */
static bool parse_preset_group(const uint8_t *grp, int grp_len,
                                int32_t video_group_id,
                                int32_t *first_preset_id)
{
    int32_t group_id   = -1;
    int32_t pid        = -1;
    bool    has_group  = false;
    bool    has_preset = false;
    int     gi         = 0;

    while (gi < grp_len) {
        uint64_t tag_val;
        int n = pb_read_varint(grp + gi, grp_len - gi, &tag_val);
        if (n < 0) break;
        gi += n;

        uint8_t  wire_type = (uint8_t)(tag_val & 0x07);
        uint32_t field_num = (uint32_t)(tag_val >> 3);

        if (field_num == 1 && wire_type == 0) {
            /* PresetGroup.id */
            uint64_t v;
            n = pb_read_varint(grp + gi, grp_len - gi, &v);
            if (n < 0) break;
            gi += n;
            group_id  = (int32_t)v;
            has_group = true;

        } else if (field_num == 2 && wire_type == 2 && !has_preset) {
            /* First PresetGroup.preset_array entry — parse the Preset message. */
            uint64_t plen_val;
            n = pb_read_varint(grp + gi, grp_len - gi, &plen_val);
            if (n < 0 || (int)plen_val > grp_len - gi - n) break;
            gi += n;

            const uint8_t *pst     = grp + gi;
            int            pst_len = (int)plen_val;
            int            pi      = 0;

            while (pi < pst_len) {
                uint64_t p_tag;
                int pn = pb_read_varint(pst + pi, pst_len - pi, &p_tag);
                if (pn < 0) break;
                pi += pn;

                uint8_t  p_wt = (uint8_t)(p_tag & 0x07);
                uint32_t p_fn = (uint32_t)(p_tag >> 3);

                if (p_fn == 1 && p_wt == 0) {
                    /* Preset.id */
                    uint64_t id_val;
                    pn = pb_read_varint(pst + pi, pst_len - pi, &id_val);
                    if (pn < 0) break;
                    pid        = (int32_t)id_val;
                    has_preset = true;
                    break;
                }
                int fs = pb_value_size(p_wt, pst + pi, pst_len - pi);
                if (fs < 0) break;
                pi += fs;
            }

            gi += pst_len;

        } else {
            int fs = pb_value_size(wire_type, grp + gi, grp_len - gi);
            if (fs < 0) break;
            gi += fs;
        }
    }

    if (has_group && group_id == video_group_id && has_preset) {
        *first_preset_id = pid;
        return true;
    }
    return false;
}

/*
 * Walk a NotifyPresetStatus Protobuf payload and return the id of the first
 * preset in the Video preset group.  Returns true and sets *preset_id on
 * success; false if the Video group is absent or has no presets.
 */
static bool find_first_video_preset_id(const uint8_t *pb, int pb_len,
                                        int32_t *preset_id)
{
    int i = 0;

    while (i < pb_len) {
        uint64_t tag_val;
        int n = pb_read_varint(pb + i, pb_len - i, &tag_val);
        if (n < 0) break;
        i += n;

        uint8_t  wire_type = (uint8_t)(tag_val & 0x07);
        uint32_t field_num = (uint32_t)(tag_val >> 3);

        if (field_num == 1 && wire_type == 2) {
            /* NotifyPresetStatus.preset_group_array */
            uint64_t grp_len_val;
            n = pb_read_varint(pb + i, pb_len - i, &grp_len_val);
            if (n < 0 || (int)grp_len_val > pb_len - i - n) break;
            i += n;

            int32_t pid;
            if (parse_preset_group(pb + i, (int)grp_len_val,
                                   PRESET_GROUP_VIDEO, &pid)) {
                *preset_id = pid;
                return true;
            }
            i += (int)grp_len_val;

        } else {
            int fs = pb_value_size(wire_type, pb + i, pb_len - i);
            if (fs < 0) break;
            i += fs;
        }
    }

    return false;
}

/* -------------------------------------------------------------------------
 * Load Preset (0x40) — send after parsing the preset ID from Protobuf
 * ------------------------------------------------------------------------- */

static esp_err_t send_load_preset(uint16_t conn_handle, gopro_ble_ctx_t *gctx,
                                   int slot, int32_t preset_id)
{
    uint32_t id = (uint32_t)preset_id;
    uint8_t pkt[7] = {
        0x06,
        0x40,
        0x04,
        (uint8_t)((id >> 24) & 0xFF),
        (uint8_t)((id >> 16) & 0xFF),
        (uint8_t)((id >>  8) & 0xFF),
        (uint8_t)((id      ) & 0xFF),
    };

    ESP_LOGI(TAG, "slot %d: Load Preset → id=%" PRId32, slot, preset_id);

    esp_err_t err = ble_core_gatt_write(conn_handle, gctx->gatt.cmd_write,
                                        pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: Load Preset write failed (%s)",
                 slot, esp_err_to_name(err));
    }
    return err;
}

/*
 * Common endpoint: parse the raw Protobuf payload from a NotifyPresetStatus
 * response (everything after the Feature ID and Action ID bytes) and send
 * Load Preset for the first Video preset found.
 */
static void load_first_video_preset(uint16_t conn_handle,
                                     const uint8_t *pb, int pb_len)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) return;

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx) return;

    int32_t preset_id;
    if (!find_first_video_preset_id(pb, pb_len, &preset_id)) {
        ESP_LOGW(TAG, "slot %d: no Video preset currently available "
                 "(camera may be in an incompatible Control Mode)", slot);
        return;
    }

    ESP_LOGI(TAG, "slot %d: found Video preset id=%" PRId32 " — loading", slot, preset_id);
    send_load_preset(conn_handle, gctx, slot, preset_id);
}

/* -------------------------------------------------------------------------
 * Phase 1 — send RequestGetPresetStatus
 * ------------------------------------------------------------------------- */

/*
 * RequestGetPresetStatus packet — written to GP-0076 (query_write).
 *
 *   Byte 0: 0x02  GPBS single-packet length (2 bytes follow)
 *   Byte 1: 0xF5  Feature ID (Preset)
 *   Byte 2: 0x72  Action ID (RequestGetPresetStatus)
 *
 * No Protobuf payload fields are needed for a one-shot status request.
 */
static const uint8_t k_get_preset_status_pkt[] = {
    0x02,
    PRESET_FEATURE_ID,
    PRESET_ACTION_REQ,
};

void gopro_presets_request_video(uint16_t conn_handle)
{
    int slot = camera_manager_find_by_handle(conn_handle);
    if (slot < 0) {
        ESP_LOGW(TAG, "presets: no camera slot for handle %d", conn_handle);
        return;
    }

    gopro_ble_ctx_t *gctx = (gopro_ble_ctx_t *)camera_manager_get_driver_ctx(slot);
    if (!gctx || gctx->gatt.query_write == 0) {
        ESP_LOGW(TAG, "slot %d: presets: query_write handle not available", slot);
        return;
    }

    ESP_LOGI(TAG, "slot %d: RequestGetPresetStatus — querying available presets", slot);

    esp_err_t err = ble_core_gatt_write(conn_handle, gctx->gatt.query_write,
                                        k_get_preset_status_pkt,
                                        sizeof(k_get_preset_status_pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %d: RequestGetPresetStatus write failed (%s)",
                 slot, esp_err_to_name(err));
    }
}

/* -------------------------------------------------------------------------
 * Phase 2a — single-packet NotifyPresetStatus response
 *
 * Called by notify.c when a single-packet GP-0077 notification has Feature
 * ID 0xF5.  Layout (1-byte GPBS header already confirmed by caller):
 *   data[0]: GPBS single-packet header
 *   data[1]: Feature ID (0xF5)
 *   data[2]: Action ID  (0xF2)
 *   data[3..]: raw Protobuf payload
 * ------------------------------------------------------------------------- */

void gopro_presets_handle_notify_status(uint16_t conn_handle,
                                         const uint8_t *data, uint16_t len)
{
    if (len < 4) {
        ESP_LOGW(TAG, "handle_notify_status: packet too short (%d bytes)", len);
        return;
    }

    /* Protobuf payload follows the 3 framing bytes (GPBS, Feature, Action). */
    load_first_video_preset(conn_handle, &data[3], (int)len - 3);
}

/* -------------------------------------------------------------------------
 * Phase 2b — fragmented NotifyPresetStatus response
 *
 * Called by notify.c for first-fragment (frame_type=0b001) and all
 * continuation-fragment (frame_type >= 0b010) GP-0077 notifications that
 * belong to the Preset feature.
 *
 * First-fragment layout:
 *   data[0]: 0b001HHHHH  (frame_type=1, high 5 bits of 13-bit length)
 *   data[1]: LLLLLLLL    (low 8 bits of 13-bit length)
 *   data[2]: Feature ID  (0xF5)
 *   data[3]: Action ID   (0xF2)
 *   data[4..]: start of Protobuf payload
 *
 * Continuation-fragment layout:
 *   data[0]: sequence byte  (frame_type in bits[7:5])
 *   data[1..]: continuation payload
 * ------------------------------------------------------------------------- */

void gopro_presets_handle_query_fragment(uint16_t conn_handle,
                                          const uint8_t *data, uint16_t len)
{
    uint8_t frame_type = (data[0] >> 5) & 0x07;

    if (frame_type == 0x01) {
        /* First fragment: start (or restart) reassembly. */
        if (len < 4) return;  /* need at least Feature + Action in payload */

        uint16_t total = (uint16_t)(((data[0] & 0x1F) << 8) | data[1]);

        preset_rx_ctx_t *rx = alloc_rx_ctx(conn_handle);
        if (!rx) {
            int slot = camera_manager_find_by_handle(conn_handle);
            ESP_LOGE(TAG, "slot %d: no free preset reassembly context", slot);
            return;
        }

        if (total > PRESET_RX_BUF_SIZE) {
            int slot = camera_manager_find_by_handle(conn_handle);
            ESP_LOGW(TAG, "slot %d: preset response too large (%u bytes) — "
                     "increase PRESET_RX_BUF_SIZE", slot, (unsigned)total);
            rx->in_use = false;
            return;
        }

        rx->rx_total      = total;
        rx->rx_assembling = true;

        /* Payload starts at byte 2 (Feature ID, Action ID, Protobuf...) */
        uint16_t copy_len = len - 2;
        memcpy(rx->rx_buf, data + 2, copy_len);
        rx->rx_filled = copy_len;

        int slot = camera_manager_find_by_handle(conn_handle);
        ESP_LOGD(TAG, "slot %d: preset reassembly started — %u/%u bytes",
                 slot, (unsigned)rx->rx_filled, (unsigned)rx->rx_total);

    } else {
        /* Continuation fragment: append to the active reassembly buffer. */
        preset_rx_ctx_t *rx = find_rx_ctx(conn_handle);
        if (!rx || !rx->rx_assembling) {
            ESP_LOGD(TAG, "preset: stray continuation (no active reassembly) — ignored");
            return;
        }
        if (len < 2) return;

        uint16_t frag_len = len - 1;  /* skip 1-byte sequence header */
        uint16_t space    = (uint16_t)sizeof(rx->rx_buf) - rx->rx_filled;
        uint16_t copy_len = (frag_len < space) ? frag_len : space;

        memcpy(rx->rx_buf + rx->rx_filled, data + 1, copy_len);
        rx->rx_filled += copy_len;

        int slot = camera_manager_find_by_handle(conn_handle);
        ESP_LOGD(TAG, "slot %d: preset reassembly %u/%u bytes",
                 slot, (unsigned)rx->rx_filled, (unsigned)rx->rx_total);
    }

    /* Check for reassembly completion. */
    preset_rx_ctx_t *rx = find_rx_ctx(conn_handle);
    if (!rx || !rx->rx_assembling) return;
    if (rx->rx_filled < rx->rx_total) return;

    int slot = camera_manager_find_by_handle(conn_handle);
    ESP_LOGI(TAG, "slot %d: preset reassembly complete (%u bytes)",
             slot, (unsigned)rx->rx_filled);

    /* rx_buf layout: [Feature ID][Action ID][Protobuf...] */
    if (rx->rx_filled >= 2 &&
        rx->rx_buf[0] == PRESET_FEATURE_ID &&
        rx->rx_buf[1] == PRESET_ACTION_RESP) {
        load_first_video_preset(conn_handle,
                                rx->rx_buf + 2,
                                (int)rx->rx_filled - 2);
    } else {
        ESP_LOGW(TAG, "slot %d: unexpected reassembled response "
                 "feat=0x%02x act=0x%02x",
                 slot,
                 rx->rx_filled > 0 ? rx->rx_buf[0] : 0xFF,
                 rx->rx_filled > 1 ? rx->rx_buf[1] : 0xFF);
    }

    rx->rx_assembling = false;
    rx->in_use        = false;
}
