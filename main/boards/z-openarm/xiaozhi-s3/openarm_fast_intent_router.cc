#include "openarm_fast_intent_router.h"

#include "application.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>

#include <esp_log.h>
#include <esp_timer.h>

#define TAG "OpenArmFastIntent"

namespace {

std::string Normalize(std::string text) {
    static constexpr std::array<const char*, 8> kPunctuation = {
        "。", "！", "？", "，", ".", "!", "?", ",",
    };
    for (const char* punctuation : kPunctuation) {
        size_t position = 0;
        while ((position = text.find(punctuation, position)) != std::string::npos) {
            text.erase(position, std::strlen(punctuation));
        }
    }
    text.erase(std::remove_if(text.begin(), text.end(), [](unsigned char ch) {
                   return std::isspace(ch) != 0;
               }),
               text.end());
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return ch < 0x80 ? static_cast<char>(std::tolower(ch)) : static_cast<char>(ch);
    });
    return text;
}

template <size_t N>
bool Matches(const std::string& text, const std::array<const char*, N>& phrases) {
    return std::any_of(phrases.begin(), phrases.end(),
                       [&text](const char* phrase) { return text == phrase; });
}

}  // namespace

OpenArmFastIntentRouter::OpenArmFastIntentRouter(OpenArmRobotClient& robot) : robot_(robot) {
}

void OpenArmFastIntentRouter::Start() {
    Application::GetInstance().AddSttListener(
        [this](const std::string& text) { HandleStt(text); });
}

void OpenArmFastIntentRouter::HandleStt(const std::string& raw_text) {
    const int64_t received_at = esp_timer_get_time();
    const std::string text = Normalize(raw_text);
    bool queued = false;
    const char* intent = nullptr;

    if (Matches(text, std::array{"停下", "停止", "停止动作", "别动", "stop"})) {
        intent = "stop";
        queued = robot_.Stop();
    } else if (Matches(text, std::array{"休息", "回正", "回到休息姿态", "rest"})) {
        intent = "rest";
        queued = robot_.Rest();
    } else if (Matches(text, std::array{"向左看", "看左边", "看看左边", "转头看左边",
                                        "lookleft"})) {
        intent = "look:left";
        queued = robot_.PerformFastIntent("quick-look-left", intent);
    } else if (Matches(text, std::array{"向右看", "看右边", "看看右边", "转头看右边",
                                        "lookright"})) {
        intent = "look:right";
        queued = robot_.PerformFastIntent("quick-look-right", intent);
    } else if (Matches(text, std::array{"点头", "点点头", "nod"})) {
        intent = "nod";
        queued = robot_.PerformFastIntent("quick-nod", intent);
    } else if (Matches(text, std::array{"摇头", "摇摇头", "shakehead"})) {
        intent = "shake_head";
        queued = robot_.PerformFastIntent("quick-shake-head", intent);
    }

    if (intent != nullptr) {
        ESP_LOGI(TAG, "matched intent=%s queued=%s stt_to_queue_ms=%lld", intent,
                 queued ? "true" : "false",
                 static_cast<long long>((esp_timer_get_time() - received_at) / 1000));
    }
}
