#include "openarm_robot_client.h"

#include "board.h"
#include "mcp_server.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <cJSON.h>
#include <esp_log.h>
#include <freertos/task.h>

#define TAG "OpenArmRobot"

namespace {

std::string JsonString(const std::string& value) {
    cJSON* item = cJSON_CreateString(value.c_str());
    char* encoded = cJSON_PrintUnformatted(item);
    std::string result(encoded == nullptr ? "\"\"" : encoded);
    cJSON_free(encoded);
    cJSON_Delete(item);
    return result;
}

}  // namespace

OpenArmRobotClient::OpenArmRobotClient() {
    queue_ = xQueueCreate(6, sizeof(Command));
    if (queue_ == nullptr) {
        throw std::runtime_error("failed to create OpenArm command queue");
    }
    xTaskCreate(WorkerTask, "openarm_mcp", 6144, this, 2, nullptr);
}

void OpenArmRobotClient::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();
    mcp.AddTool(
        "self.robot.perform",
        "Perform a published robot action by id. Prefer known actions such as high-wave-front, welcome-bow, "
        "please-left, please-right, attentive-nod, curious-tilt, or rest. The command is queued locally and "
        "returns immediately.",
        PropertyList({Property("action", kPropertyTypeString)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Perform(properties["action"].value<std::string>());
        });
    mcp.AddTool(
        "self.robot.stop",
        "Stop the current robot motion and hold near the current position.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return Stop(); });
    mcp.AddTool(
        "self.robot.rest",
        "Cancel the current robot motion and return the whole robot safely to its rest pose.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return Rest(); });
    mcp.AddTool(
        "self.robot.get_status",
        "Get the cached result of the most recent local robot command without blocking the voice task.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return StatusJson(); });
}

bool OpenArmRobotClient::Perform(const std::string& action_id, bool autonomous) {
    if (action_id.empty() || action_id.size() >= sizeof(Command{}.action_id)) {
        return false;
    }
    bool interrupt_autonomous = false;
    if (!autonomous) {
        interrupt_autonomous = autonomous_active_.exchange(false);
    }
    if (interrupt_autonomous) {
        ClearPendingCommands();
    }
    if (autonomous) {
        autonomous_active_.store(true);
    }
    const bool queued =
        Enqueue(CommandType::kPerform, action_id.c_str(), autonomous, interrupt_autonomous);
    if (autonomous && !queued) {
        autonomous_active_.store(false);
    }
    return queued;
}

bool OpenArmRobotClient::Stop() {
    autonomous_active_.store(false);
    ClearPendingCommands();
    return Enqueue(CommandType::kStop);
}

bool OpenArmRobotClient::Rest() {
    autonomous_active_.store(false);
    ClearPendingCommands();
    return Enqueue(CommandType::kRest);
}

bool OpenArmRobotClient::InterruptAutonomous() {
    if (!autonomous_active_.exchange(false)) {
        return true;
    }
    ClearPendingCommands();
    return Enqueue(CommandType::kRest);
}

void OpenArmRobotClient::ClearPendingCommands() {
    if (queue_ != nullptr) {
        xQueueReset(queue_);
    }
}

bool OpenArmRobotClient::Enqueue(CommandType type, const char* action_id, bool autonomous,
                                 bool interrupt_autonomous) {
    if (queue_ == nullptr) {
        return false;
    }
    Command command = {
        .type = type,
        .autonomous = autonomous,
        .interrupt_autonomous = interrupt_autonomous,
        .action_id = {},
    };
    std::strncpy(command.action_id, action_id, sizeof(command.action_id) - 1);
    if (xQueueSend(queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full; dropping type=%d", static_cast<int>(type));
        return false;
    }
    return true;
}

void OpenArmRobotClient::WorkerTask(void* context) {
    static_cast<OpenArmRobotClient*>(context)->WorkerLoop();
}

void OpenArmRobotClient::WorkerLoop() {
    Command command;
    while (xQueueReceive(queue_, &command, portMAX_DELAY) == pdTRUE) {
        bool ok = false;
        if (command.type == CommandType::kPerform) {
            if (command.interrupt_autonomous) {
                CallTool("cancel_motion", "{}");
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            constexpr int kInterruptSubmitAttempts = 5;
            const int attempts = command.interrupt_autonomous ? kInterruptSubmitAttempts : 1;
            for (int attempt = 1; attempt <= attempts; ++attempt) {
                ok = CallTool("execute_action", "{\"id\":" + JsonString(command.action_id) + "}");
                if (ok || attempt == attempts) {
                    break;
                }
                ESP_LOGI(TAG, "retrying action %s after autonomous cancel (%d/%d)",
                         command.action_id, attempt + 1, attempts);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            if (command.autonomous && !ok) {
                autonomous_active_.store(false);
            }
        } else if (command.type == CommandType::kStop) {
            ok = CallTool("cancel_motion", "{}");
        } else if (command.type == CommandType::kRest) {
            ok = CallTool("stop_and_rest", "{\"time_ms\":1400}");
        }
        ESP_LOGI(TAG, "command type=%d result=%s", static_cast<int>(command.type), ok ? "ok" : "failed");
    }
}

bool OpenArmRobotClient::CallTool(const char* tool_name, const std::string& arguments_json) {
    std::string token = CONFIG_OPENARM_MCP_TOKEN;
    if (token.empty()) {
        SaveResult(false, "OPENARM_MCP_TOKEN is empty");
        ESP_LOGE(TAG, "OPENARM_MCP_TOKEN is empty");
        return false;
    }

    const uint32_t id = request_id_.fetch_add(1);
    std::string body =
        "{\"jsonrpc\":\"2.0\",\"id\":" + std::to_string(id) +
        ",\"method\":\"tools/call\",\"params\":{\"name\":" + JsonString(tool_name) +
        ",\"arguments\":" + arguments_json + "}}";

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    http->SetHeader("Authorization", "Bearer " + token);
    http->SetHeader("Content-Type", "application/json");
    http->SetContent(std::move(body));
    if (!http->Open("POST", CONFIG_OPENARM_MCP_URL)) {
        SaveResult(false, "failed to open local MCP endpoint");
        return false;
    }
    const int status_code = http->GetStatusCode();
    std::string response = http->ReadAll();
    http->Close();

    bool ok = status_code == 200;
    cJSON* root = cJSON_Parse(response.c_str());
    if (root == nullptr || cJSON_GetObjectItem(root, "error") != nullptr) {
        ok = false;
    } else {
        cJSON* result = cJSON_GetObjectItem(root, "result");
        cJSON* is_error = result == nullptr ? nullptr : cJSON_GetObjectItem(result, "isError");
        if (cJSON_IsTrue(is_error)) {
            ok = false;
        }
    }
    cJSON_Delete(root);

    if (response.size() > 512) {
        response.resize(512);
    }
    SaveResult(ok, "HTTP " + std::to_string(status_code) + ": " + response);
    if (!ok) {
        ESP_LOGW(TAG, "tool %s failed with HTTP %d: %s", tool_name, status_code, response.c_str());
    }
    return ok;
}

std::string OpenArmRobotClient::StatusJson() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", last_ok_);
    cJSON_AddBoolToObject(root, "autonomous_active", autonomous_active_.load());
    cJSON_AddNumberToObject(root, "queued", queue_ == nullptr ? 0 : uxQueueMessagesWaiting(queue_));
    cJSON_AddStringToObject(root, "last_result", last_result_.c_str());
    char* encoded = cJSON_PrintUnformatted(root);
    std::string result(encoded == nullptr ? "{}" : encoded);
    cJSON_free(encoded);
    cJSON_Delete(root);
    return result;
}

void OpenArmRobotClient::SaveResult(bool ok, const std::string& result) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_ok_ = ok;
    last_result_ = result;
}
