/*
 * examples/minimal.c - the smallest possible libfpv integration.
 *
 * Headless: a flat ground, a drone, scripted "radio" channels. It arms,
 * climbs for a second, and prints where the drone is. A real mod replaces
 * the scripted channels with nothing at all (libfpv reads the player's
 * radio itself) and draws frame.camera / frame.osd instead of printing.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later OR MIT
 */
#include <stdio.h>

#include "libfpv.h"

static void log_cb(void *user, fpv_log_level level, const char *msg) {
    (void)user;
    if (level >= FPV_LOG_WARN) fprintf(stderr, "[libfpv] %s\n", msg);
}

int main(int argc, char **argv) {
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = argc > 1 ? argv[1] : "data";
    cfg.coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);
    cfg.timing = FPV_TIMING_MANUAL; /* deterministic: we advance time with fpv_step */
    cfg.flags = FPV_INIT_NO_DEVICE_INPUT | FPV_INIT_NO_CONFIGURATOR;
    cfg.log = log_cb;
    if (fpv_global_init(&cfg) != FPV_OK) {
        fprintf(stderr, "init failed: %s\n", fpv_last_error());
        return 1;
    }

    /* World: a 200 m x 200 m ground quad at y = 0. */
    float ground[] = {-100, 0, -100, 100, 0, -100, 100, 0, 100, -100, 0, 100};
    uint32_t idx[] = {0, 1, 2, 0, 2, 3};
    fpv_mesh mesh = {0};
    mesh.positions = ground;
    mesh.vertex_count = 4;
    mesh.indices = idx;
    mesh.triangle_count = 2;
    fpv_static_surfaces_load(&mesh, 1);

    fpv_drone *drone;
    if (fpv_drone_create(NULL, &drone) != FPV_OK) {
        fprintf(stderr, "drone: %s\n", fpv_last_error());
        return 1;
    }

    /* AETR + AUX1..4. Arm switch (AUX1) off first, then on, then throttle. */
    float ch[8] = {0, 0, -1, 0, -1, -1, -1, -1};
    fpv_frame frame;
    frame.struct_size = sizeof(frame);
    for (int i = 0; i < 300; i++) { /* 3 s at 100 Hz host frames */
        if (i == 100) ch[4] = 1.0f;  /* arm */
        if (i == 150) ch[2] = -0.3f; /* some throttle */
        fpv_input_set_channels(ch, 8);
        fpv_step(0.01f);
        fpv_drone_tick(drone, 0.01f, &frame);
        if (i % 50 == 49)
            printf("t=%.2fs armed=%d height=%.2fm speed=%.1fkm/h battery=%.2fV %s\n", frame.sim_time,
                   (frame.flags & FPV_DRONE_ARMED) != 0, frame.position[1], frame.speed_kmh, frame.battery.voltage,
                   frame.arming_disable_text);
    }

    fpv_drone_delete(drone);
    fpv_global_terminate();
    return 0;
}
