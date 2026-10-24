// Software-only continuous raw-input sequence for the actual ROS worker probe.
// Reuses the validated driver parsers, never launches ROS or an actuator.
#define main archived_cpp_core_replay_main
#include "x650_cpp_core_replay.cpp"
#undef main

int main(int argc,char** argv){
  if(argc!=5){std::cerr<<"usage: fast_sequence startup.json startup_sha seed_t0.json output_directory\n";return 2;}
  try{
    const auto startup=load_startup(argv[1],argv[2]);
    if(!startup.trajectory.is_fast())throw std::invalid_argument("Frozen fast startup required");
    const auto seed=read_json(argv[3]);
    if(seed["config"]!=startup.manifest["config"] || number(seed["input"]["t"])!=0.)
      throw std::invalid_argument("Same config and explicit t=0 raw seed required");
    const fs::path out(argv[4]);if(fs::exists(out))throw std::invalid_argument("New sequence directory required");
    fs::create_directories(out);
    void* handle=dlopen(startup.predictor_library.c_str(),RTLD_NOW|RTLD_LOCAL);if(!handle)throw std::runtime_error(dlerror());
    auto fn=reinterpret_cast<x650::Ts20BatchFn>(dlsym(handle,"ts20_prediction_batch"));if(!fn)throw std::runtime_error("Missing predictor ABI");
    const auto core_id=loaded_identity(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
    const auto optimization_id=loaded_identity(dlsym(RTLD_DEFAULT,"x650_assemble"));
    if(core_id["sha256"]!=startup.manifest["core_library"]["sha256"] ||
       optimization_id["sha256"]!=startup.manifest["optimization_library"]["sha256"])
      throw std::invalid_argument("Actually loaded core dependency differs");
    x650::Core core(startup.config,fn,x650::DiagnosticMode::Compact);
    const auto anchor=anchor_of(seed["memory"]["anchor"]);
    const auto center=numbers<3>(seed["input"]["target"]["position"]);
    const auto previous=numbers<5>(seed["memory"]["previous_command"]);
    const double origin=10.;core.reset(anchor);core.prime({{origin-.02,previous},{origin,previous}},origin);
    Json::Value start;start["origin_s"]=origin;start["anchor_position_enu"]=json_array(anchor.position);
    start["anchor_rotation_row_major"]=json_array(anchor.rotation);start["anchor_q"]=json_array(anchor.q);
    start["target_position_enu"]=json_array(center);start["history_count"]=2;
    std::array<double,64> history_times{};history_times[0]=origin-.02;history_times[1]=origin;
    std::array<double,320> history_commands{};for(int i=0;i<2;++i)std::copy(previous.begin(),previous.end(),history_commands.begin()+5*i);
    start["history_time_s"]=json_array(history_times);start["history_command"]=json_array(history_commands);start["ref_alt"]=0.;start["yaw_ned"]=0.;
    write_json(out/"live_start.json",start);
    Json::Value report;report["kind"]="cpp_fast_synthetic_sequence_v1";report["passed"]=false;report["complete"]=false;
    report["trajectory"]=startup.manifest["trajectory"];report["config"]=startup.manifest["config"];
    report["scope"]="160 continuous software inputs; translated fixed measured seed and histories, not a new physical recording or 24-second flight";
    report["physical_acceptance"]=false;report["loaded_core"]=core_id;report["loaded_optimization"]=optimization_id;
    report["seed_path"]=fs::canonical(argv[3]).string();report["seed_sha256"]=sha256(argv[3]);
    report["startup_path"]=fs::canonical(argv[1]).string();report["startup_sha256"]=sha256(argv[1]);
    report["live_start_path"]=fs::canonical(out/"live_start.json").string();report["live_start_sha256"]=sha256(out/"live_start.json");
    report["rows"]=Json::Value(Json::arrayValue);
    const auto initial=input_of(seed["input"]);
    if(initial.context.packets.empty())throw std::invalid_argument("Seed packet history is empty");
    // The numerical fixture has pre-task packets but a zero bus epoch. A live
    // source must place its bus origin before those packets. Choose the first
    // existing packet epoch; this is a phase-equivalent 5-ms grid origin, not
    // a change to any packet timestamp or the production admission contract.
    const double bus_origin=std::min(initial.context.bus_origin,initial.context.packets.front().send_time);
    const double bus_cycles=(initial.context.bus_origin-bus_origin)/startup.config.bus_period;
    if(std::abs(bus_cycles-std::round(bus_cycles))>1e-10)
      throw std::invalid_argument("Synthetic prehistory would alter the frozen bus phase");
    report["synthetic_bus_origin"]=bus_origin;
    report["original_seed_bus_origin"]=initial.context.bus_origin;
    try{for(unsigned sequence=1;sequence<=160;++sequence){
      const double t=.02*sequence;auto input=initial;input.t=t;input.dt=.02;
      input.target=startup.trajectory.sample(t,center);input.preview=startup.trajectory.preview(t,.02,startup.config.nodes,startup.config.node_dt,center);
      for(int j=0;j<3;++j){input.state.position[j]+=input.target.position[j]-center[j];input.state.velocity[j]+=input.target.velocity[j];input.state.acceleration[j]+=input.target.acceleration[j];}
      input.state.control_time=origin+t;
      input.state.stamp=origin+t-(initial.state.control_time-initial.state.stamp);
      input.state.joint_stamp=origin+t-(initial.state.control_time-initial.state.joint_stamp);
      input.context.reference_time=t-(initial.state.control_time-initial.state.joint_stamp);
      for(auto& row:input.context.commands)row.time+=t;
      for(auto& row:input.context.packets)row.send_time+=t;
      input.context.bus_origin=bus_origin;
      for(const auto& row:input.context.packets)
        if(row.send_time<bus_origin||row.send_time>t+1e-10)
          throw std::invalid_argument("Synthetic history violates the unchanged live bus boundary");
      Json::Value value=seed["input"];value["t"]=t;value["dt"]=.02;
#define S(name) value["state"][#name]=json_array(input.state.name)
      S(position);S(velocity);S(acceleration);
#undef S
      value["state"]["stamp"]=input.state.stamp;value["state"]["control_time"]=input.state.control_time;value["state"]["joint_stamp"]=input.state.joint_stamp;
      auto goal=[](const x650::Target& v){Json::Value j;j["position"]=json_array(v.position);j["velocity"]=json_array(v.velocity);j["acceleration"]=json_array(v.acceleration);return j;};
      value["target"]=goal(input.target);value["preview"]=Json::Value(Json::arrayValue);for(const auto& p:input.preview)value["preview"].append(goal(p));
      value["context"]["reference_time"]=input.context.reference_time;
      value["context"]["bus_origin"]=input.context.bus_origin;
      for(Json::ArrayIndex i=0;i<input.context.commands.size();++i)value["context"]["commands"][i]["time"]=input.context.commands[i].time;
      for(Json::ArrayIndex i=0;i<input.context.packets.size();++i)value["context"]["packets"][i]["send_time"]=input.context.packets[i].send_time;
      const auto begin=std::chrono::steady_clock::now();const auto result=core.step(input);
      const double core_wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      Json::Value row;row["sequence"]=sequence;row["input"]=value;row["command"]=json_array(result.command);
      row["position_reference"]=json_array(result.position_reference);row["velocity_feedforward"]=json_array(result.velocity_feedforward);
      row["acceleration_feedforward"]=json_array(result.acceleration_feedforward);row["diagnostics"]=diagnostic_json(result.diagnostics);
      row["core_step_wall_ms"]=core_wall_ms;
      const auto path=out/("step_"+std::to_string(sequence)+".json");write_json(path,row);
      Json::Value item;item["path"]=fs::canonical(path).string();item["sha256"]=sha256(path);item["bytes"]=Json::UInt64(fs::file_size(path));report["rows"].append(item);
    }report["passed"]=true;report["complete"]=true;
    }catch(const std::exception& error){report["error"]=error.what();}
    report["step_count"]=Json::UInt64(core.memory().step_count);write_json(out/"manifest.json",report);
    dlclose(handle);return report["passed"].asBool()?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}
}
