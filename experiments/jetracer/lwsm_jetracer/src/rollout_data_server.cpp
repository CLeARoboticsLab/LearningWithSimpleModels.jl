#include "lwsm_jetracer/rollout_data_server.hpp"

#include <ros_sockets/nlohmann/json.hpp>

namespace communication
{

RolloutDataServer::RolloutDataServer(std::uint16_t port)
    : Server(port)
{}

void RolloutDataServer::processInboundJson(nlohmann::json json_data)
{
  if (!json_data.contains("action"))
    return;
  if (json_data["action"] != "get_rollout_data")
    return;
  scheduleWrite(json_datastring()+'\n');
}

std::string RolloutDataServer::json_datastring()
{
  nlohmann::json j;
  j["seg_idxs"] = rollout_data_.seg_idxs;
  j["ts"] = rollout_data_.ts;
  std::vector<std::vector<double>> xs;
  std::vector<std::vector<double>> us;
  for (int i = 0; i < rollout_data_.ts.size(); i++)
  {
    xs.push_back(rollout_data_.xs[i].data);
    us.push_back(rollout_data_.us[i].data);
  }
  j["xs"] = xs;
  j["us"] = us;
  return j.dump();
}

} // namespace communication