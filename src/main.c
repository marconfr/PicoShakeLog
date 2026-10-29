/**
 * @file main.c
 * @brief Main entry point for PicoShakeLog IMU data logger
 *
 * This file contains the main() function that initializes the PicoShakeLog system
 * and launches both cores. Core 0 handles WiFi/TCP communication while Core 1
 * manages real-time IMU data acquisition.
 *
 * Initialization sequence:
 * 1. Standard I/O and system services
 * 2. All subsystems (WiFi, IMU, DMA, FIFO)
 * 3. Core 1 launch for acquisition
 * 4. Core 0 enters main loop for network handling
 *
 * @author Frédéric Marçon
 * @date 2026
 * @copyright Academic Free License v3.0
 */

#include "icm42688_driver.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "psl_api.h"
#include "psl_core0.h"
#include "psl_core1.h"
#include "psl_log.h"
#include <stdio.h>
#include <stdlib.h>

/* Global variables defined in psl_api.c */
extern psl_state_handler_t pv_state_handler;

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("    ____  _           _____ __          __        __\n");
    printf("   / __ \\(_)________ / ___// /_  ____ _/ /_____  / /   ____  ____ _\n");
    printf("  / /_/ / / ___/ __ \\\\__ \\/ __ \\/ __ `/ //_/ _ \\/ /   / __ \\/ __ `/\n");
    printf(" / ____/ / /__/ /_/ /__/ / / / / /_/ / ,< /  __/ /___/ /_/ / /_/ /\n");
    printf("/_/   /_/\\___/\\____/____/_/ /_/\\__,_/_/|_|\\___/_____/\\____/\\__, /\n");
    printf("                                                          /____/\n");
    printf("PicoShakeLog - IMU Data Logger v0.2.0-rc.1\n");
    printf("(c) 2026 Frédéric MARÇON - Academic Free License v3.0\n");
    printf("\n");
    LOG_I(BOOT, "Initializing system");

    /* Initialize all subsystems */
    if (!init_system_dev(&pv_state_handler)) {
        LOG_F(BOOT, "System initialization failed");
        while (true) {
            sleep_ms(1000);
        }
    }

    /* Launch Core 1 (real-time acquisition) */
    LOG_I(BOOT, "Launching Core 1 (acquisition)");
    multicore_launch_core1(core1_entry);
    LOG_I(BOOT, "Core 1 launched");

    /* Core 0: WiFi + TCP management + FIFO→client transfer */
    LOG_I(BOOT, "Core 0 entering main loop (WiFi/TCP)");
    core0_main_loop(); /* Never returns */

    /* Unreachable code */
    return 0;
}
