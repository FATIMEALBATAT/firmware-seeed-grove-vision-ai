/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include ----------------------------------------------------------------- */
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <cmath>
#include "board_config.h"
#include "hx_drv_timer.h"
#include "hx_drv_uart.h"
#include "grove_ai_config.h"
#include "logger.h"
#include "console_io.h"
#include "debugger.h"
#include "external_flash.h"


extern "C" {
#include "datapath.h"
#include "sensor_core.h"
};

#include "external_flash.h"
#include "webusb.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/at_base64_lib.h"
#include "firmware-sdk/at-server/ei_at_server.h"
#include "firmware-sdk/at-server/ei_at_command_set.h"
#include "edge_impulse_sensecap.h"
#include "ei_camera_himax.h"
#include "ei_device_sense_cap.h"
#include "ei_at_handlers.h"
#include "ei_i2c_handlers.h"
#include "ei_run_impulse.h"

static DEV_UART *console_uart;

struct Point {
    float x;
    float y;
};

struct Rect {
    int x;
    int y;
    int width;
    int height;
};

struct Car {
    int id;
    Point centroid;
    bool isNew;  // Flag to indicate if this car has been announced
};

std::vector<Car> trackedCars;
int nextCarID = 0;  // Global ID counter for new cars

Point calculateCentroid(const Rect& rect) {
    return Point{rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f};
}

void updateCarTracking(const std::vector<Rect>& detections, float distanceThreshold = 50.0f) {
    bool newCarDetected = false;

    for (const auto& det : detections) {
        Point newCentroid = calculateCentroid(det);
        bool found = false;

        for (auto& car : trackedCars) {
            float distance = sqrt(pow(car.centroid.x - newCentroid.x, 2) + pow(car.centroid.y - newCentroid.y, 2));
            if (distance < distanceThreshold) {
                car.centroid = newCentroid;  // Update centroid
                found = true;
                break;
            }
        }

        if (!found) {
            trackedCars.push_back({nextCarID++, newCentroid, true});
            newCarDetected = true;
        }
    }

    // Announce new cars detected in this cycle
    if (newCarDetected) {
        int count = trackedCars.size();  // Total unique cars detected so far
        printf("New car detected! Total cars tracked: %d\n", count);
    }
}

extern "C" int edge_impulse_sensecap(void)
{
    hx_drv_timer_init();
    static DEV_UART *console_uart = hx_drv_uart_get_dev((USE_SS_UART_E)CONSOLE_UART_ID);
    
    std::vector<Rect> detections;  // Placeholder for real detection logic
    char c;

    while (true) {
        if(console_uart->uart_read_nonblock((void*)&c, 1) != 0) {
            // Handle UART commands or system queries here
        }

        // Simulate a detection update; replace with actual sensor/model output handling
        detections.push_back({100, 100, 50, 50}); // Example: Simulated detection

        // Update tracking and manage detection announcements
        updateCarTracking(detections);

        // Clear detections after processing
        detections.clear();
    }

    return 0;
}