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
    bool Perform(const std::string& action_id, bool autonomous = false);
    bool Stop();
    bool Rest();
    bool InterruptAutonomous();
    bool GetAutonomousActive() const { return autonomous_active_.load(); }

private:
    enum class CommandType : uint8_t {
        kPerform,
        kStop,
        kRest,
    };

    struct Command {
        CommandType type;
        bool autonomous;
        bool interrupt_autonomous;
        char action_id[48];
    };

    QueueHandle_t queue_ = nullptr;
    std::atomic<bool> autonomous_active_{false};
    std::atomic<uint32_t> request_id_{1};
    std::mutex status_mutex_;
    std::string last_result_ = "not connected";
    bool last_ok_ = false;

    static void WorkerTask(void* context);
    void WorkerLoop();
    void ClearPendingCommands();
    bool Enqueue(CommandType type, const char* action_id = "", bool autonomous = false,
                 bool interrupt_autonomous = false);
    bool CallTool(const char* tool_name, const std::string& arguments_json);
    std::string StatusJson();
    void SaveResult(bool ok, const std::string& result);
};

#endif
