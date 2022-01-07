#ifndef RL_CONFIG
#define RL_CONFIG


#include <string>
#include "Tools/json.h"

namespace RLConfig{
std::ifstream configFile("../rl_config.json");
json::value configData = json::parse(configFile);
std::string mode = to_string(configData["mode"]);
bool train_mode = to_bool(configData["train_mode"]);

}

#endif