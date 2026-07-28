#include "openarm_idle_controller.h"

#include "application.h"

#include <algorithm>

#include <esp_log.h>
#include <esp_random.h>
#include <freertos/task.h>

#define TAG "OpenArmIdle"

OpenArmIdleController::OpenArmIdleController(OpenArmRobotClient& robot) : robot_(robot) {
}

void OpenArmIdleController::Start() {
#if CONFIG_OPENARM_IDLE_MOTION
    xTaskCreate(Task, "openarm_idle", 4096, this, 1, nullptr);
#endif
}

void OpenArmIdleController::Task(void* context) {
    static_cast<OpenArmIdleController*>(context)->Loop();
}

void OpenArmIdleController::Loop() {
    constexpr const char* kIdleActions[] = {
        "idle-look-around",
        "idle-neck-stretch",
    };
    uint32_t idle_seconds = 0;
    uint32_t next_motion_at = CONFIG_OPENARM_IDLE_MIN_SECONDS;
    bool dozing = false;
    bool was_idle = false;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const bool is_idle = Application::GetInstance().GetDeviceState() == kDeviceStateIdle;

        if (!is_idle) {
            if (was_idle && robot_.GetAutonomousActive()) {
                ESP_LOGI(TAG, "interaction interrupted autonomous motion");
                robot_.InterruptAutonomous();
            }
            idle_seconds = 0;
            dozing = false;
            was_idle = false;
            next_motion_at = CONFIG_OPENARM_IDLE_MIN_SECONDS;
            continue;
        }

        was_idle = true;
        ++idle_seconds;
        if (!dozing && idle_seconds >= CONFIG_OPENARM_DOZE_SECONDS) {
            if (robot_.Perform("idle-doze", true)) {
                ESP_LOGI(TAG, "queued autonomous doze");
                dozing = true;
            }
            continue;
        }

        if (!dozing && idle_seconds >= next_motion_at) {
            const size_t index = esp_random() % (sizeof(kIdleActions) / sizeof(kIdleActions[0]));
            if (robot_.Perform(kIdleActions[index], true)) {
                ESP_LOGI(TAG, "queued autonomous action %s", kIdleActions[index]);
            }
            const uint32_t minimum = CONFIG_OPENARM_IDLE_MIN_SECONDS;
            const uint32_t maximum = std::max<uint32_t>(minimum, CONFIG_OPENARM_IDLE_MAX_SECONDS);
            next_motion_at = idle_seconds + minimum + (esp_random() % (maximum - minimum + 1));
        }
    }
}
