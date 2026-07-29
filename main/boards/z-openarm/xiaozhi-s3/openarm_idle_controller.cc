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
#if CONFIG_OPENARM_WAKE_MOTION || CONFIG_OPENARM_IDLE_MOTION
    Application::GetInstance().AddDeviceStateChangeListener(
        [this](DeviceState old_state, DeviceState new_state) { HandleStateChange(old_state, new_state); });
#endif
#if CONFIG_OPENARM_IDLE_MOTION
    xTaskCreate(Task, "openarm_idle", 4096, this, 1, nullptr);
#endif
}

void OpenArmIdleController::Task(void* context) {
    static_cast<OpenArmIdleController*>(context)->Loop();
}

void OpenArmIdleController::HandleStateChange(DeviceState old_state, DeviceState new_state) {
#if CONFIG_OPENARM_WAKE_MOTION
    if (old_state == kDeviceStateIdle && new_state == kDeviceStateConnecting) {
        if (robot_.Perform(CONFIG_OPENARM_WAKE_ACTION)) {
            ESP_LOGI(TAG, "queued wake action %s", CONFIG_OPENARM_WAKE_ACTION);
        } else {
            ESP_LOGW(TAG, "failed to queue wake action %s", CONFIG_OPENARM_WAKE_ACTION);
        }
    }
#endif

#if CONFIG_OPENARM_IDLE_MOTION
    if (old_state == kDeviceStateIdle && new_state != kDeviceStateIdle &&
        robot_.GetAutonomousActive()) {
        ESP_LOGI(TAG, "interaction interrupted autonomous motion");
        robot_.InterruptAutonomous();
    }
#endif
}

void OpenArmIdleController::Loop() {
    constexpr const char* kIdleActions[] = {
        "idle-look-around",
        "idle-neck-stretch",
        "idle-look-around",
        "idle-neck-stretch",
        "rest",
    };
    uint32_t idle_seconds = 0;
    uint32_t next_motion_at = CONFIG_OPENARM_IDLE_MIN_SECONDS;
    bool dozing = false;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const bool is_idle = Application::GetInstance().GetDeviceState() == kDeviceStateIdle;

        if (!is_idle) {
            idle_seconds = 0;
            dozing = false;
            next_motion_at = CONFIG_OPENARM_IDLE_MIN_SECONDS;
            continue;
        }

        if (robot_.HasInteractionLease()) {
            continue;
        }

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
