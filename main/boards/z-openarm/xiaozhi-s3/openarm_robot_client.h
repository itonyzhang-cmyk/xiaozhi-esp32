#ifndef _OPENARM_ROBOT_CLIENT_H_
#define _OPENARM_ROBOT_CLIENT_H_

#include <atomic>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class OpenArmRobotClient {
public:
    OpenArmRobotClient();

    void RegisterMcpTools();
    void StartCatalogSync();
    bool PerformEmbodied(const std::string& action);
    bool Perform(const std::string& action_id, bool autonomous = false);
    bool PerformSequence(const std::string& sequence_json);
    bool Stop();
    bool Rest();
    bool InterruptAutonomous();
    bool GetAutonomousActive() const { return autonomous_active_.load(); }

private:
    enum class CommandType : uint8_t {
        kPerform,
        kPerformSequence,
        kStop,
        kRest,
    };

    struct Command {
        CommandType type;
        bool autonomous;
        bool interrupt_autonomous;
        char action_id[48];
        char payload[2048];
    };

    QueueHandle_t queue_ = nullptr;
    std::atomic<bool> autonomous_active_{false};
    std::atomic<uint32_t> request_id_{1};
    std::mutex status_mutex_;
    std::string last_result_ = "no robot command sent yet";
    bool last_ok_ = true;
    std::mutex catalog_mutex_;
    std::string published_catalog_ = "{\"ok\":false,\"status\":\"not_synced\",\"actions\":[]}";
    std::string basic_catalog_ = "{\"ok\":false,\"status\":\"not_synced\",\"actions\":[]}";

    static void WorkerTask(void* context);
    static void CatalogTask(void* context);
    void WorkerLoop();
    void CatalogLoop();
    bool SyncCatalogs();
    bool CatalogContains(const std::string& catalog, const std::string& action_id);
    void ClearPendingCommands();
    bool Enqueue(CommandType type, const char* action_id = "", bool autonomous = false,
                 bool interrupt_autonomous = false, const char* payload = "");
    bool CallTool(const char* tool_name, const std::string& arguments_json,
                  std::string* response_out = nullptr, bool update_status = true);
    std::string CachedPublishedCatalog();
    std::string CachedBasicCatalog();
    std::string StatusJson();
    void SaveResult(bool ok, const std::string& result);
};

#endif
