#include "meshtastic_crypto.h"
#include <string.h>
#include "aes/esp_aes.h"

const uint8_t meshtastic_default_psk[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
};

const uint8_t meshtastic_hessen_psk[32] = {
    0xfa, 0xe4, 0xcc, 0x11, 0xa3, 0x91, 0xee, 0x19, 0x2a, 0x69, 0x7b, 0xfe, 0x0d, 0x13, 0x8e, 0x11,
    0xde, 0x41, 0x86, 0xf0, 0x08, 0x41, 0x8f, 0xc2, 0x67, 0xf1, 0xeb, 0xe2, 0xca, 0x19, 0x70, 0xe5,
};

esp_err_t meshtastic_crypto_ctr(const uint8_t *key, size_t key_len,
                                uint32_t from_node, uint32_t packet_id,
                                uint8_t *buf, size_t len)
{
    if (key_len != 16 && key_len != 32) {
        return ESP_ERR_INVALID_ARG;
    }

    /* nonce[0..4)=packet_id LE, [4..8) stays 0 (packet_id is 32-bit here,
     * zero-extended to the 64-bit slot the original protocol uses),
     * [8..12)=from_node LE, [12..16) stays 0 (block counter starts at 0). */
    uint8_t nonce[16] = { 0 };
    memcpy(nonce, &packet_id, sizeof(packet_id));
    memcpy(nonce + 8, &from_node, sizeof(from_node));

    esp_aes_context ctx;
    esp_aes_init(&ctx);
    int ret = esp_aes_setkey(&ctx, key, (unsigned int)(key_len * 8));
    if (ret != 0) {
        esp_aes_free(&ctx);
        return ESP_FAIL;
    }

    size_t nc_off = 0;
    uint8_t stream_block[16] = { 0 };
    ret = esp_aes_crypt_ctr(&ctx, len, &nc_off, nonce, stream_block, buf, buf);
    esp_aes_free(&ctx);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}
