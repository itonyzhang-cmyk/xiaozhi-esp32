#include "openarm_robot_client.h"

#include "application.h"
#include "board.h"
#include "mcp_server.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <esp_log.h>
#include <cJSON.h>
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

std::string QueueAckJson(bool queued, const char* command, const std::string& action = {}) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", queued);
    cJSON_AddStringToObject(root, "status", queued ? "queued" : "rejected");
    cJSON_AddBoolToObject(root, "execution_confirmed", false);
    cJSON_AddBoolToObject(root, "embodied", true);
    cJSON_AddStringToObject(root, "command", command);
    if (!action.empty()) {
        cJSON_AddStringToObject(root, "action", action.c_str());
    }
    char* encoded = cJSON_PrintUnformatted(root);
    std::string result(encoded == nullptr ? "{}" : encoded);
    cJSON_free(encoded);
    cJSON_Delete(root);
    return result;
}

std::string ResolveEmbodiedAlias(const std::string& requested) {
    static constexpr std::array<std::pair<const char*, const char*>, 15> kAliases = {{
        {"hand_wave", "wave"},
        {"挥手", "wave"},
        {"点头", "nod"},
        {"twist_waist", "sway_waist"},
        {"waist_twist", "sway_waist"},
        {"扭腰", "sway_waist"},
        {"raise_hand", "raise_arm"},
        {"hands_up", "raise_arm"},
        {"抬手", "raise_arm"},
        {"抬臂", "raise_arm"},
        {"shake", "shake_head"},
        {"high_wave_front", "high-wave-front"},
        {"high_wave", "high-wave-front"},
        {"高举挥手", "high-wave-front"},
        {"双臂高举挥手", "high-wave-front"},
    }};
    for (const auto& [alias, canonical] : kAliases) {
        if (requested == alias) {
            return canonical;
        }
    }
    return requested;
}

std::string ResolveLookDirection(const std::string& requested) {
    static constexpr std::array<std::pair<const char*, const char*>, 10> kLookDirections = {{
        {"look_left", "left"},
        {"turn_head_left", "left"},
        {"向左看", "left"},
        {"看左边", "left"},
        {"左转头", "left"},
        {"look_right", "right"},
        {"turn_head_right", "right"},
        {"向右看", "right"},
        {"看右边", "right"},
        {"右转头", "right"},
    }};
    for (const auto& [alias, direction] : kLookDirections) {
        if (requested == alias) {
            return direction;
        }
    }
    return "";
}

std::string FastPresetForBasicAction(const std::string& action) {
    static constexpr std::array<std::pair<const char*, const char*>, 6> kFastPresets = {{
        {"wave", "casual-wave"},
        {"nod", "quick-nod"},
        {"shake_head", "quick-shake-head"},
        {"sway_waist", "waist-sway"},
        {"raise_arm", "primitive-right-raise-forward"},
        {"rotate_forearm", "forearm-twist"},
    }};
    for (const auto& [basic, preset] : kFastPresets) {
        if (action == basic) {
            return preset;
        }
    }
    return "";
}

std::string ToolResultText(const std::string& response) {
    cJSON* root = cJSON_Parse(response.c_str());
    cJSON* result = root == nullptr ? nullptr : cJSON_GetObjectItem(root, "result");
    cJSON* content = result == nullptr ? nullptr : cJSON_GetObjectItem(result, "content");
    cJSON* first = cJSON_IsArray(content) ? cJSON_GetArrayItem(content, 0) : nullptr;
    cJSON* text = first == nullptr ? nullptr : cJSON_GetObjectItem(first, "text");
    std::string value = cJSON_IsString(text) ? text->valuestring : "";
    cJSON_Delete(root);
    return value;
}

std::string CompactCatalog(const std::string& response, bool include_parameters) {
    const std::string text = ToolResultText(response);
    cJSON* source = cJSON_Parse(text.c_str());
    cJSON* actions = source == nullptr ? nullptr : cJSON_GetObjectItem(source, "actions");
    if (!cJSON_IsArray(actions)) {
        cJSON_Delete(source);
        return "";
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "status", "synced");
    cJSON_AddNumberToObject(root, "count", cJSON_GetArraySize(actions));
    cJSON* compact_actions = cJSON_AddArrayToObject(root, "actions");
    cJSON* item = nullptr;
    cJSON_ArrayForEach (item, actions) {
        cJSON* compact = cJSON_CreateObject();
        for (const char* key : {"id", "name", "description"}) {
            cJSON* value = cJSON_GetObjectItem(item, key);
            if (cJSON_IsString(value)) {
                cJSON_AddStringToObject(compact, key, value->valuestring);
            }
        }
        if (include_parameters) {
            cJSON* parameters = cJSON_GetObjectItem(item, "parameters");
            if (cJSON_IsObject(parameters)) {
                cJSON_AddItemToObject(compact, "parameters", cJSON_Duplicate(parameters, true));
            }
        }
        cJSON_AddItemToArray(compact_actions, compact);
    }
    char* encoded = cJSON_PrintUnformatted(root);
    std::string result(encoded == nullptr ? "" : encoded);
    cJSON_free(encoded);
    cJSON_Delete(root);
    cJSON_Delete(source);
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
        "Move your own physical body with one known preset or basic action. Dispatch this silently "
        "before composing the spoken response so transport and body startup overlap TTS generation. "
        "Never say that you are checking actions, "
        "calling a tool, controlling a robot, or about to execute a command. Do not repeat the "
        "user's request. Common embodied names such as wave, nod, shake_head, look_left, "
        "look_right, twist_waist, and raise_arm are resolved locally to a published preset or "
        "basic action. A queued result "
        "only means accepted, not that physical movement has completed; never claim that the user "
        "must have seen it. After acceptance, respond naturally in first person.",
        PropertyList({Property("action", kPropertyTypeString)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const auto action = properties["action"].value<std::string>();
            return QueueAckJson(PerformEmbodied(action), "perform", action);
        });
    mcp.AddTool(
        "self.robot.list_actions",
        "Silent internal lookup of your cached physical-body presets. Use only when no known "
        "preset matches. Never announce this lookup, list capabilities to the user unless asked, "
        "or say that you are searching for an action.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return CachedPublishedCatalog(); });
    mcp.AddTool(
        "self.robot.list_basic_actions",
        "Silent internal lookup of your parameterized body movements. Each entry declares "
        "amplitude, total duration, repetitions, side, and direction. Never narrate this lookup "
        "or describe it as controlling another robot.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return CachedBasicCatalog(); });
    mcp.AddTool(
        "self.robot.perform_sequence",
        "Move your own physical body with a sequential action chain. Dispatch this silently before "
        "composing the spoken response so body startup overlaps TTS generation; never repeat the "
        "request or mention planning, tools, commands, "
        "action libraries, or controlling a robot. sequence_json must be a JSON object such as "
        "{\"name\":\"greeting\",\"actions\":[{\"action\":\"nod\",\"amplitude\":0.4,"
        "\"duration_ms\":1800},{\"action\":\"shake_head\",\"amplitude\":0.3,"
        "\"duration_ms\":2000},{\"action\":\"wave\",\"side\":\"right\",\"amplitude\":0.6,"
        "\"duration_ms\":3600}]}. After a successful call, continue with a short natural "
        "first-person reply.",
        PropertyList({Property("sequence_json", kPropertyTypeString)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return QueueAckJson(PerformSequence(properties["sequence_json"].value<std::string>()),
                                "perform_sequence");
        });
    mcp.AddTool(
        "self.robot.stop",
        "Stop your current body movement and hold near the current position. Treat this as your "
        "own body and do not mention tools or robot control.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return QueueAckJson(Stop(), "stop"); });
    mcp.AddTool(
        "self.robot.rest",
        "Cancel your current body movement and return to your rest pose. Do not mention tools, "
        "commands, or controlling a robot.",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue { return QueueAckJson(Rest(), "rest"); });
    mcp.AddTool("self.robot.get_status",
                "Get the cached result of the most recent local robot command without blocking the "
                "voice task.",
                PropertyList(),
                [this](const PropertyList&) -> ReturnValue { return StatusJson(); });
}

void OpenArmRobotClient::StartCatalogSync() {
    xTaskCreate(CatalogTask, "openarm_catalog", 7168, this, 1, nullptr);
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

bool OpenArmRobotClient::PerformFastIntent(const std::string& action_id,
                                           const std::string& semantic_key,
                                           uint32_t lease_ms) {
    const uint32_t now = xTaskGetTickCount();
    {
        std::lock_guard<std::mutex> lock(fast_intent_mutex_);
        last_fast_intent_ = semantic_key;
        last_fast_intent_at_ = now;
    }
    interaction_lease_until_.store(now + pdMS_TO_TICKS(lease_ms));
    return Perform(action_id);
}

bool OpenArmRobotClient::HasInteractionLease() const {
    const uint32_t until = interaction_lease_until_.load();
    const uint32_t now = xTaskGetTickCount();
    return until != 0 && static_cast<int32_t>(until - now) > 0;
}

bool OpenArmRobotClient::IsRecentFastIntent(const std::string& semantic_key) {
    constexpr TickType_t kDuplicateWindow = pdMS_TO_TICKS(8000);
    std::lock_guard<std::mutex> lock(fast_intent_mutex_);
    return semantic_key == last_fast_intent_ &&
           xTaskGetTickCount() - last_fast_intent_at_ <= kDuplicateWindow;
}

bool OpenArmRobotClient::PerformEmbodied(const std::string& action) {
    if (action.empty()) {
        SaveResult(false, "embodied action is empty");
        return false;
    }

    const std::string look_direction = ResolveLookDirection(action);
    if (!look_direction.empty()) {
        const std::string semantic_key = "look:" + look_direction;
        if (IsRecentFastIntent(semantic_key)) {
            ESP_LOGI(TAG, "suppressed duplicate cloud action %s", semantic_key.c_str());
            return true;
        }
        const std::string preset = "quick-look-" + look_direction;
        ESP_LOGI(TAG, "resolved embodied action %s -> cached preset %s", action.c_str(),
                 preset.c_str());
        return Perform(preset);
    }

    const std::string resolved = ResolveEmbodiedAlias(action);
    if ((resolved == "nod" || resolved == "shake_head") && IsRecentFastIntent(resolved)) {
        ESP_LOGI(TAG, "suppressed duplicate cloud action %s", resolved.c_str());
        return true;
    }
    bool is_published = false;
    bool is_basic = false;
    {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        is_published = CatalogContains(published_catalog_, resolved);
        is_basic = CatalogContains(basic_catalog_, resolved);
        const std::string fast_preset = FastPresetForBasicAction(resolved);
        if (is_basic && !fast_preset.empty() && CatalogContains(published_catalog_, fast_preset)) {
            ESP_LOGI(TAG, "resolved embodied action %s -> cached preset %s", action.c_str(),
                     fast_preset.c_str());
            return Perform(fast_preset);
        }
    }

    if (is_basic) {
        cJSON* root = cJSON_CreateObject();
        cJSON* actions = cJSON_AddArrayToObject(root, "actions");
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "action", resolved.c_str());
        cJSON_AddItemToArray(actions, item);
        char* encoded = cJSON_PrintUnformatted(root);
        const std::string sequence(encoded == nullptr ? "" : encoded);
        cJSON_free(encoded);
        cJSON_Delete(root);
        ESP_LOGI(TAG, "resolved embodied action %s -> basic %s", action.c_str(), resolved.c_str());
        return !sequence.empty() && PerformSequence(sequence);
    }
    if (is_published) {
        ESP_LOGI(TAG, "resolved embodied action %s -> published %s", action.c_str(),
                 resolved.c_str());
        return Perform(resolved);
    }

    ESP_LOGW(TAG, "rejecting unknown embodied action: %s (resolved=%s)", action.c_str(),
             resolved.c_str());
    SaveResult(false, "unknown embodied action: " + action);
    return false;
}

bool OpenArmRobotClient::PerformSequence(const std::string& sequence_json) {
    if (sequence_json.empty() || sequence_json.size() >= sizeof(Command{}.payload)) {
        SaveResult(false, "basic action sequence JSON is empty or too large");
        return false;
    }
    cJSON* root = cJSON_Parse(sequence_json.c_str());
    cJSON* actions = root == nullptr ? nullptr : cJSON_GetObjectItem(root, "actions");
    const bool valid = cJSON_IsObject(root) && cJSON_IsArray(actions) &&
                       cJSON_GetArraySize(actions) > 0 && cJSON_GetArraySize(actions) <= 12;
    if (!valid) {
        cJSON_Delete(root);
        SaveResult(false, "basic action sequence must contain between 1 and 12 actions");
        return false;
    }

    std::string unknown_action;
    {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        cJSON* item = nullptr;
        cJSON_ArrayForEach (item, actions) {
            cJSON* action = cJSON_GetObjectItem(item, "action");
            if (!cJSON_IsString(action) || !CatalogContains(basic_catalog_, action->valuestring)) {
                unknown_action = cJSON_IsString(action) ? action->valuestring : "<missing>";
                break;
            }
        }
    }
    cJSON_Delete(root);
    if (!unknown_action.empty()) {
        ESP_LOGW(TAG, "rejecting sequence with unknown basic action: %s", unknown_action.c_str());
        SaveResult(false, "unknown basic action in sequence: " + unknown_action);
        return false;
    }

    autonomous_active_.store(false);
    return Enqueue(CommandType::kPerformSequence, "", false, false, sequence_json.c_str());
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
                                 bool interrupt_autonomous, const char* payload) {
    if (queue_ == nullptr) {
        return false;
    }
    Command command = {
        .type = type,
        .autonomous = autonomous,
        .interrupt_autonomous = interrupt_autonomous,
        .enqueued_at_us = esp_timer_get_time(),
        .action_id = {},
        .payload = {},
    };
    std::strncpy(command.action_id, action_id, sizeof(command.action_id) - 1);
    std::strncpy(command.payload, payload, sizeof(command.payload) - 1);
    const std::string command_name = type == CommandType::kPerform
                                         ? "perform"
                                         : (type == CommandType::kPerformSequence
                                                ? "perform_sequence"
                                                : (type == CommandType::kStop ? "stop" : "rest"));
    const std::string queued_result =
        command.action_id[0] == '\0'
            ? "queued " + command_name
            : "queued " + command_name + ": " + std::string(command.action_id);
    SaveResult(true, queued_result);
    if (xQueueSend(queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full; dropping type=%d", static_cast<int>(type));
        SaveResult(false, "local command queue is full");
        return false;
    }
    ESP_LOGI(TAG, "%s", queued_result.c_str());
    return true;
}

void OpenArmRobotClient::WorkerTask(void* context) {
    static_cast<OpenArmRobotClient*>(context)->WorkerLoop();
}

void OpenArmRobotClient::WorkerLoop() {
    Command command;
    while (xQueueReceive(queue_, &command, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "dispatch queue_wait_ms=%lld",
                 static_cast<long long>((esp_timer_get_time() - command.enqueued_at_us) / 1000));
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
        } else if (command.type == CommandType::kPerformSequence) {
            ok = CallTool("execute_basic_sequence", command.payload);
        } else if (command.type == CommandType::kStop) {
            ok = CallTool("cancel_motion", "{}");
        } else if (command.type == CommandType::kRest) {
            ok = CallTool("stop_and_rest", "{\"time_ms\":1400}");
        }
        ESP_LOGI(TAG, "command type=%d result=%s", static_cast<int>(command.type),
                 ok ? "ok" : "failed");
    }
}

void OpenArmRobotClient::CatalogTask(void* context) {
    static_cast<OpenArmRobotClient*>(context)->CatalogLoop();
}

void OpenArmRobotClient::CatalogLoop() {
    constexpr TickType_t kPollInterval = pdMS_TO_TICKS(5000);
    constexpr TickType_t kRefreshInterval = pdMS_TO_TICKS(15 * 60 * 1000);
    TickType_t last_sync = 0;
    bool initial_sync_complete = false;
    while (true) {
        vTaskDelay(kPollInterval);
        if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
            continue;
        }
        const TickType_t now = xTaskGetTickCount();
        if (initial_sync_complete && now - last_sync < kRefreshInterval) {
            continue;
        }
        if (queue_ != nullptr && uxQueueMessagesWaiting(queue_) != 0) {
            continue;
        }
        ESP_LOGI(TAG, "synchronizing robot action catalogs");
        if (SyncCatalogs()) {
            initial_sync_complete = true;
            last_sync = now;
        } else {
            ESP_LOGW(TAG, "robot action catalog synchronization failed; retrying while idle");
        }
    }
}

bool OpenArmRobotClient::SyncCatalogs() {
    std::string published_response;
    std::string basic_response;
    const bool published_ok = CallTool("list_action_catalog", "{}", &published_response, false);
    const bool basic_ok = CallTool("list_basic_action_catalog", "{}", &basic_response, false);
    const std::string published = published_ok ? CompactCatalog(published_response, false) : "";
    const std::string basic = basic_ok ? CompactCatalog(basic_response, true) : "";
    if (published.empty() || basic.empty()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(catalog_mutex_);
        published_catalog_ = published;
        basic_catalog_ = basic;
    }
    ESP_LOGI(TAG, "robot action catalogs synchronized");
    return true;
}

bool OpenArmRobotClient::CatalogContains(const std::string& catalog, const std::string& action_id) {
    cJSON* root = cJSON_Parse(catalog.c_str());
    cJSON* actions = root == nullptr ? nullptr : cJSON_GetObjectItem(root, "actions");
    bool found = false;
    cJSON* item = nullptr;
    cJSON_ArrayForEach (item, actions) {
        cJSON* id = cJSON_GetObjectItem(item, "id");
        if (cJSON_IsString(id) && action_id == id->valuestring) {
            found = true;
            break;
        }
    }
    cJSON_Delete(root);
    return found;
}

bool OpenArmRobotClient::CallTool(const char* tool_name, const std::string& arguments_json,
                                  std::string* response_out, bool update_status) {
    std::string token = CONFIG_OPENARM_MCP_TOKEN;
    if (token.empty()) {
        if (update_status) {
            SaveResult(false, "OPENARM_MCP_TOKEN is empty");
        }
        ESP_LOGE(TAG, "OPENARM_MCP_TOKEN is empty");
        return false;
    }

    const uint32_t id = request_id_.fetch_add(1);
    std::string body = "{\"jsonrpc\":\"2.0\",\"id\":" + std::to_string(id) +
                       ",\"method\":\"tools/call\",\"params\":{\"name\":" + JsonString(tool_name) +
                       ",\"arguments\":" + arguments_json + "}}";

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(15);
    http->SetHeader("Authorization", "Bearer " + token);
    http->SetHeader("Content-Type", "application/json");
    http->SetContent(std::move(body));
    if (!http->Open("POST", CONFIG_OPENARM_MCP_URL)) {
        if (update_status) {
            SaveResult(false, "failed to open local MCP endpoint");
        }
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

    if (response_out != nullptr) {
        *response_out = response;
    }
    std::string status_response = response;
    if (status_response.size() > 512) {
        status_response.resize(512);
    }
    if (update_status) {
        SaveResult(ok, "HTTP " + std::to_string(status_code) + ": " + status_response);
    }
    if (!ok) {
        ESP_LOGW(TAG, "tool %s failed with HTTP %d: %s", tool_name, status_code,
                 status_response.c_str());
    }
    return ok;
}

std::string OpenArmRobotClient::CachedPublishedCatalog() {
    std::lock_guard<std::mutex> lock(catalog_mutex_);
    return published_catalog_;
}

std::string OpenArmRobotClient::CachedBasicCatalog() {
    std::lock_guard<std::mutex> lock(catalog_mutex_);
    return basic_catalog_;
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
