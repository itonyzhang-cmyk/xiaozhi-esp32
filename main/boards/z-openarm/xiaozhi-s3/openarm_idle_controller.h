#ifndef _OPENARM_IDLE_CONTROLLER_H_
#define _OPENARM_IDLE_CONTROLLER_H_

#include "device_state.h"
#include "openarm_robot_client.h"

class OpenArmIdleController {
public:
    explicit OpenArmIdleController(OpenArmRobotClient& robot);
    void Start();

private:
    OpenArmRobotClient& robot_;

    static void Task(void* context);
    void HandleStateChange(DeviceState old_state, DeviceState new_state);
    void Loop();
};

#endif
