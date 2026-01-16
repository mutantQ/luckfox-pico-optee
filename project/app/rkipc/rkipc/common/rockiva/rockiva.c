// Copyright 2021 Rockchip Electronics Co., Ltd. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// Stub implementation when RockIVA library is not available

#include "rockiva.h"
#include <stdio.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "rockiva.c"

// Stub implementations - RockIVA is not available in this build

int rkipc_rockiva_init() {
    printf("[%s] RockIVA not available - stub init\n", LOG_TAG);
    return 0;
}

int rkipc_rockiva_deinit() {
    printf("[%s] RockIVA not available - stub deinit\n", LOG_TAG);
    return 0;
}

int rkipc_rockiva_write_rgb888_frame(uint16_t width, uint16_t height, uint32_t frame_id,
                                     unsigned char *buffer) {
    (void)width;
    (void)height;
    (void)frame_id;
    (void)buffer;
    return 0;
}

int rkipc_rockiva_write_rgb888_frame_by_fd(uint16_t width, uint16_t height, uint32_t frame_id,
                                           int32_t fd) {
    (void)width;
    (void)height;
    (void)frame_id;
    (void)fd;
    return 0;
}

int rkipc_rockiva_write_nv12_frame_by_fd(uint16_t width, uint16_t height, uint32_t frame_id,
                                         int32_t fd) {
    (void)width;
    (void)height;
    (void)frame_id;
    (void)fd;
    return 0;
}

int rkipc_rockiva_write_nv12_frame_by_phy_addr(uint16_t width, uint16_t height, uint32_t frame_id,
                                               uint8_t *phy_addr) {
    (void)width;
    (void)height;
    (void)frame_id;
    (void)phy_addr;
    return 0;
}

int rkipc_rknn_object_get(RockIvaBaResult *ba_result) {
    if (ba_result) {
        ba_result->objNum = 0;
    }
    return -1; // No objects available
}
