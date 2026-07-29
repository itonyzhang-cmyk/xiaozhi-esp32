#ifndef _OPENARM_FAST_INTENT_ROUTER_H_
#define _OPENARM_FAST_INTENT_ROUTER_H_

#include "openarm_robot_client.h"

#include <string>

class OpenArmFastIntentRouter {
public:
    explicit OpenArmFastIntentRouter(OpenArmRobotClient& robot);
    void Start();

private:
    OpenArmRobotClient& robot_;

    void HandleStt(const std::string& text);
};

#endif
