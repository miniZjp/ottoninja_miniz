#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Embedded GIF data wrapper for Otto emoji animations.
 *
 * The GIF files are embedded at build time via ESP-IDF's EMBED_FILES
 * mechanism, which exposes each file as:
 *   _binary_<name>_gif_start / _binary_<name>_gif_end
 *
 * This header provides thin wrapper structs so that code can access
 * the data via the convenient .data / .data_size interface.
 */

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t _binary_staticstate_gif_start[];
extern const uint8_t _binary_staticstate_gif_end[];

extern const uint8_t _binary_happy_gif_start[];
extern const uint8_t _binary_happy_gif_end[];

extern const uint8_t _binary_sad_gif_start[];
extern const uint8_t _binary_sad_gif_end[];

extern const uint8_t _binary_anger_gif_start[];
extern const uint8_t _binary_anger_gif_end[];

extern const uint8_t _binary_scare_gif_start[];
extern const uint8_t _binary_scare_gif_end[];

extern const uint8_t _binary_buxue_gif_start[];
extern const uint8_t _binary_buxue_gif_end[];

#ifdef __cplusplus
}
#endif

struct OttoGifData {
    const uint8_t* data;
    size_t data_size;
};

static const OttoGifData staticstate = {
    _binary_staticstate_gif_start,
    static_cast<size_t>(_binary_staticstate_gif_end - _binary_staticstate_gif_start)
};

static const OttoGifData happy = {
    _binary_happy_gif_start,
    static_cast<size_t>(_binary_happy_gif_end - _binary_happy_gif_start)
};

static const OttoGifData sad = {
    _binary_sad_gif_start,
    static_cast<size_t>(_binary_sad_gif_end - _binary_sad_gif_start)
};

static const OttoGifData anger = {
    _binary_anger_gif_start,
    static_cast<size_t>(_binary_anger_gif_end - _binary_anger_gif_start)
};

static const OttoGifData scare = {
    _binary_scare_gif_start,
    static_cast<size_t>(_binary_scare_gif_end - _binary_scare_gif_start)
};

static const OttoGifData buxue = {
    _binary_buxue_gif_start,
    static_cast<size_t>(_binary_buxue_gif_end - _binary_buxue_gif_start)
};
