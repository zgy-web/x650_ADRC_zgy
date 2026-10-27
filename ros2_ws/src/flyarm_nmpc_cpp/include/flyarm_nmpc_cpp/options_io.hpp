#pragma once
#include "flyarm_mpc_cpp/config_io.hpp"
#include "x650_nmpc/online_solver.hpp"
#include <set>

namespace flyarm_nmpc_cpp {
namespace fm=flyarm_mpc_cpp;
struct BoundOptions {
  Json::Value manifest;
  std::string backend;
  x650_nmpc::OnlineOptions options;
};
inline BoundOptions load_options(const fm::fs::path& path,const std::string& expected,
                                const std::string& task_hash) {
  Json::Value binding;binding["path"]=path.string();binding["sha256"]=expected;
  fm::bound_path(binding);BoundOptions out;out.manifest=fm::read_json(path);const auto& root=out.manifest;
  fm::exact_keys(root,{"schema_version","kind","simulation_only","backend","task_config_sha256",
                      "worker_executable","options","source_artifacts"});
  if(fm::integer(root["schema_version"])!=1||root["kind"]!="x650_online_nmpc_options_v1"||root["simulation_only"]!=true||
     fm::string_field(root,"task_config_sha256")!=task_hash)
    throw std::invalid_argument("NMPC options identity/simulation/task binding differs");
  out.backend=fm::string_field(root,"backend");
  if(out.backend!="nmpc"&&out.backend!="mpc_callable")throw std::invalid_argument("Unknown upper controller backend");
  fm::exact_keys(root["worker_executable"],{"path","sha256"});
  const auto binary=fm::bound_path(root["worker_executable"]);
  if(binary!=fm::fs::canonical("/proc/self/exe"))
    throw std::invalid_argument("Actual NMPC worker differs from bound executable");
  const auto& o=root["options"];
  fm::exact_keys(o,{"max_proposals","absolute_decrease","relative_decrease","proposal_budget_ms","total_budget_ms"});
  out.options.nonlinear.max_proposals=fm::integer(o["max_proposals"]);
  out.options.nonlinear.absolute_decrease=fm::number(o["absolute_decrease"]);
  out.options.nonlinear.relative_decrease=fm::number(o["relative_decrease"]);
  out.options.nonlinear.evaluation_budget_ms=fm::number(o["proposal_budget_ms"]);
  out.options.total_budget_ms=fm::number(o["total_budget_ms"]);
  if(out.options.nonlinear.max_proposals!=4||out.options.nonlinear.absolute_decrease!=1e-10||
     out.options.nonlinear.relative_decrease!=1e-7||out.options.nonlinear.evaluation_budget_ms!=40.||out.options.total_budget_ms!=50.)
    throw std::invalid_argument("Explicit initial online NMPC budget/policy differs");
  if(!root["source_artifacts"].isArray()||root["source_artifacts"].empty())
    throw std::invalid_argument("Bound NMPC source files required");
  std::set<std::string> seen;
  for(const auto& source:root["source_artifacts"]){fm::exact_keys(source,{"path","sha256"});
    if(!seen.insert(fm::bound_path(source).string()).second)throw std::invalid_argument("Duplicate NMPC source binding");}
  return out;
}
}  // namespace flyarm_nmpc_cpp
