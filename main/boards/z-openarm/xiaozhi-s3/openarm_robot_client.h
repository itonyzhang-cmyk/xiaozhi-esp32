#ifndef _OPENARM_ROBOT_CLIENT_H_
#define _OPENARM_ROBOT_CLIENT_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class Http;

class OpenArmRobotClient {
public:
    enum class TriggerSource : uint8_t {
        kCloudMcp,
        kFastIntent,
        kAutonomousIdle,
        kWake,
        kSystem,
    };

    OpenArmRobotClient();
    ~OpenArmRobotClient();

    void RegisterMcpTools();
    void StartCatalogSync();
    bool PerformEmbodied(const std::string& action);
    bool Perform(const std::string& action_id, bool autonomous = false,
                 TriggerSource source = TriggerSource::kCloudMcp);
    bool PerformFastIntent(const std::string& action_id, const std::string& semantic_key,
                           uint32_t lease_ms = 15000);
    bool PerformSequence(const std::string& sequence_json,
                         TriggerSource source = TriggerSource::kCloudMcp);
    bool Stop(TriggerSource source = TriggerSource::kCloudMcp);
    bool Rest(TriggerSource source = TriggerSource::kCloudMcp);
    bool InterruptAutonomous();
    bool GetAutonomousActive() const { return autonomous_active_.load(); }
    bool HasInteractionLease() const;

private:
    enum class CommandType : uint8_t {
        kPerform,
        kPerformSequence,
        kStop,
        kRest,
    };

    struct Command {
        uint32_t id;
        CommandType type;
        TriggerSource source;
        bool autonomous;
        bool interrupt_autonomous;
        int64_t enqueued_at_us;
        char action_id[48];
        char payload[2048];
    };

    QueueHandle_t queue_ = nullptr;
    std::atomic<bool> autonomous_active_{false};
    std::atomic<uint32_t> request_id_{1};
    std::atomic<uint32_t> command_id_{1};
    std::mutex http_mutex_;
    std::unique_ptr<Http> http_;
    std::mutex status_mutex_;
    std::string last_result_ = "no robot command sent yet";
    bool last_ok_ = true;
    uint32_t trigger_counts_[5] = {};
    uint32_t cloud_duplicates_suppressed_ = 0;
    uint32_t last_queued_id_ = 0;
    std::string last_queued_source_ = "none";
    std::string last_queued_command_;
    std::string last_queued_action_;
    int64_t last_queued_uptime_ms_ = 0;
    uint32_t last_submitted_id_ = 0;
    std::string last_submitted_source_ = "none";
    std::string last_submitted_command_;
    std::string last_submitted_action_;
    bool last_submitted_ok_ = false;
    int64_t last_submitted_uptime_ms_ = 0;
    std::mutex catalog_mutex_;
    std::string published_catalog_ = "{\"ok\":false,\"status\":\"not_synced\",\"actions\":[]}";
    std::string basic_catalog_ = "{\"ok\":false,\"status\":\"not_synced\",\"actions\":[]}";
    std::atomic<uint32_t> interaction_lease_until_{0};
    std::mutex fast_intent_mutex_;
    std::string last_fast_intent_;
    uint32_t last_fast_intent_at_ = 0;

    static void WorkerTask(void* context);
    static void CatalogTask(void* context);
    void WorkerLoop();
    void CatalogLoop();
    bool SyncCatalogs();
    bool CatalogContains(const std::string& catalog, const std::string& action_id);
    bool IsRecentFastIntent(const std::string& semantic_key);
    void ClearPendingCommands();
    bool Enqueue(CommandType type, const char* action_id = "", bool autonomous = false,
                 bool interrupt_autonomous = false, const char* payload = "",
                 TriggerSource source = TriggerSource::kCloudMcp);
    bool CallTool(const char* tool_name, const std::string& arguments_json,
                  std::string* response_out = nullptr, bool update_status = true);
    std::string CachedPublishedCatalog();
    std::string CachedBasicCatalog();
    std::string StatusJson();
    void SaveResult(bool ok, const std::string& result);
    void RecordQueued(const Command& command);
    void RecordSubmitted(const Command& command, bool ok);
    void RecordSuppressedCloudDuplicate();
    static const char* CommandTypeName(CommandType type);
    static const char* TriggerSourceName(TriggerSource source);
};

#endif
