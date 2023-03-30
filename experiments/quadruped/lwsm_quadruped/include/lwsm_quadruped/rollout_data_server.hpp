#pragma once

#include <ros_sockets/nlohmann/json.hpp>
#include <ros_sockets/server.hpp>

#include "lwsm_jetracer/RolloutData.h"

namespace communication
{

class RolloutDataServer : public Server
{
  public:
    RolloutDataServer(std::uint16_t port);
    void update(lwsm_jetracer::RolloutData rollout_data_msg){rollout_data_ = rollout_data_msg;}

  private:
    void processInboundJson(nlohmann::json json_data);
    std::string json_datastring();
    lwsm_jetracer::RolloutData rollout_data_;
};

} // namespace communication