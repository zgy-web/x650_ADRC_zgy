// Physics-step disturbance: world force at base_link CoM plus world free couple.
// PreUpdate ECS enqueue deltas are logged, not claimed as a contact-force sensor.
#ifndef WBCLOSED_DISTURBANCE_SELF_TEST
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/plugin/Register.hh>
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace wbclosed {
struct Event { double start, end; std::array<double, 6> wrench; };
std::string trim(const std::string &s) {
  const auto a=s.find_first_not_of(" \t\r\n");
  return a==std::string::npos ? "" : s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
}
double number(const std::string &s) {
  const auto value=trim(s); std::size_t end=0; const double x=std::stod(value,&end);
  if(end!=value.size() || !std::isfinite(x)) throw std::runtime_error("Nonfinite/malformed number");
  return x;
}
Event parseEvent(const std::string &s, double duration) {
  std::istringstream stream(s); std::string item; std::vector<double> v;
  while(std::getline(stream,item,',')) v.push_back(number(item));
  if(v.size()!=8 || s.empty() || s.back()==',' || v[0]<0 || v[1]<=v[0] || v[1]>duration)
    throw std::runtime_error("event requires start,end,Fx,Fy,Fz,Tx,Ty,Tz in task duration");
  Event e{v[0],v[1],{v[2],v[3],v[4],v[5],v[6],v[7]}};
  if(std::hypot(std::hypot(v[2],v[3]),v[4])>2. || std::hypot(std::hypot(v[5],v[6]),v[7])>.15)
    throw std::runtime_error("Event exceeds 2 N force or 0.15 Nm free-couple bound");
  return e;
}
std::array<double,6> sample(const std::vector<Event> &events,double t) {
  std::array<double,6> result{};
  for(const auto &e:events) if(t>=e.start && t<e.end)
    for(std::size_t k=0;k<6;++k) result[k]+=e.wrench[k];
  return result;
}

#ifndef WBCLOSED_DISTURBANCE_SELF_TEST
namespace gs=gz::sim;
class Disturbance : public gs::System, public gs::ISystemConfigure, public gs::ISystemPreUpdate {
  std::string modelName,linkName,outputPath,startPath,stopPath;
  std::ofstream out;
  std::vector<Event> events;
  gs::Entity base=gs::kNullEntity;
  double duration=0,t0=-1,lastSim=-1,maxError=0,maxDt=0,firstTask=-1,lastTask=-1;
  std::array<double,6> impulse{},expectedImpulse{};
  bool ready=false,anchored=false,done=false;
  std::size_t taskRows=0,rows=0;
  static std::array<gz::math::Vector3d,2> wrench(const gs::EntityComponentManager &ecm,gs::Entity e) {
    const auto c=ecm.Component<gs::components::ExternalWorldWrenchCmd>(e);
    if(!c) return {gz::math::Vector3d::Zero,gz::math::Vector3d::Zero};
    const auto &w=c->Data();
    return {gz::math::Vector3d(w.force().x(),w.force().y(),w.force().z()),
            gz::math::Vector3d(w.torque().x(),w.torque().y(),w.torque().z())};
  }
  static double maximum(const gz::math::Vector3d &v) {
    return std::max({std::abs(v.X()),std::abs(v.Y()),std::abs(v.Z())});
  }
  static bool finite(const gz::math::Vector3d &v) {
    return std::isfinite(v.X())&&std::isfinite(v.Y())&&std::isfinite(v.Z());
  }
  static std::string json(const std::string &s) {
    std::ostringstream r; r<<'"';
    for(unsigned char c:s) {
      if(c=='"'||c=='\\') r<<'\\'<<c;
      else if(c<32) r<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
      else r<<c;
    }
    r<<'"';return r.str();
  }
  void status(const std::string &state,const std::string &reason="") {
    const auto target=outputPath+".status.json",temp=target+".tmp";
    std::ofstream s(temp);
    if(!s) throw std::runtime_error("Cannot write disturbance status");
    s<<std::setprecision(17)<<"{\n\"state\":"<<json(state)<<",\"reason\":"<<json(reason)
     <<",\"ready\":"<<(ready?"true":"false")<<",\"anchored\":"<<(anchored?"true":"false")
     <<",\"complete\":"<<(done?"true":"false")<<",\"success\":"<<(state=="SUCCESS"?"true":"false")
     <<",\"t0_sim_s\":"<<t0<<",\"duration_s\":"<<duration<<",\"rows\":"<<rows<<",\"task_rows\":"<<taskRows
     <<",\"first_task_sim_s\":"<<firstTask<<",\"last_task_sim_s\":"<<lastTask
     <<",\"max_step_dt_s\":"<<maxDt<<",\"max_enqueue_component_error\":"<<maxError
     <<",\"model\":"<<json(modelName)<<",\"link\":"<<json(linkName)
     <<",\"force_frame\":\"world ENU\",\"free_couple_frame\":\"world ENU\""
     <<",\"application_point\":\"base_link own center of mass, not whole-system CoM\""
     <<",\"measurement_semantics\":\"same-PreUpdate ExternalWorldWrenchCmd increment; not sensor measurement or independent proof of physics consumption\""
     <<",\"time_semantics\":\"UpdateInfo.simTime and dt; rectangular event on [start_s,end_s); absolute immutable t0\""
     <<",\"impulse_force_Ns_couple_Nms\":[";
    for(std::size_t k=0;k<6;++k)s<<(k?",":"")<<impulse[k];
    s<<"],\"expected_impulse_force_Ns_couple_Nms\":[";
    for(std::size_t k=0;k<6;++k)s<<(k?",":"")<<expectedImpulse[k];
    s<<"]}\n"; s.flush(); if(!s)throw std::runtime_error("Disturbance status write failed");
    s.close();std::filesystem::rename(temp,target);
  }
  void vector(const gz::math::Vector3d &v) {out<<','<<v.X()<<','<<v.Y()<<','<<v.Z();}
public:
  void Configure(const gs::Entity &,const std::shared_ptr<const sdf::Element> &sdf,
                 gs::EntityComponentManager &,gs::EventManager &) override {
    if(!sdf||!sdf->HasElement("config_path"))throw std::runtime_error("Disturbance requires config_path");
    std::ifstream in(sdf->Get<std::string>("config_path"));
    if(!in)throw std::runtime_error("Cannot read disturbance config");
    std::map<std::string,std::string> cfg;std::vector<std::string> rawEvents;std::string line;
    while(std::getline(in,line)) {
      line=trim(line);if(line.empty()||line[0]=='#')continue;
      const auto at=line.find('=');if(at==std::string::npos)throw std::runtime_error("Expected config key=value");
      auto key=trim(line.substr(0,at)),value=trim(line.substr(at+1));
      if(key=="event")rawEvents.push_back(value);
      else if(!cfg.emplace(key,value).second)throw std::runtime_error("Duplicate config key");
    }
    for(const auto &item:cfg)
      if(item.first!="model"&&item.first!="link"&&item.first!="output"&&item.first!="start_path"&&item.first!="stop_path"&&item.first!="duration")
        throw std::runtime_error("Unknown config key: "+item.first);
    modelName=cfg.at("model");linkName=cfg.at("link");outputPath=cfg.at("output");
    startPath=cfg.at("start_path");stopPath=cfg.at("stop_path");duration=number(cfg.at("duration"));
    if(modelName.empty()||linkName!="base_link"||duration<=0||duration>300)throw std::runtime_error("Invalid model/link/duration");
    for(const auto &p:{outputPath,startPath,stopPath})
      if(!std::filesystem::path(p).is_absolute())throw std::runtime_error("Paths must be absolute");
    for(const auto &raw:rawEvents)events.push_back(parseEvent(raw,duration));
    std::sort(events.begin(),events.end(),[](const Event&a,const Event&b){return a.start<b.start;});
    for(std::size_t i=0;i<events.size();++i) {
      if(i&&events[i].start<events[i-1].end)throw std::runtime_error("Overlapping events forbidden");
      for(std::size_t k=0;k<6;++k)expectedImpulse[k]+=(events[i].end-events[i].start)*events[i].wrench[k];
    }
    if(std::filesystem::exists(outputPath)||std::filesystem::exists(outputPath+".status.json"))
      throw std::runtime_error("Refusing to overwrite disturbance evidence");
    out.open(outputPath);if(!out)throw std::runtime_error("Cannot open disturbance log");
    out<<std::setprecision(17)<<"sim_s,t_s,step_dt_s,in_task,event_active,fx_command_N,fy_command_N,fz_command_N,tx_command_Nm,ty_command_Nm,tz_command_Nm,fx_enqueued_N,fy_enqueued_N,fz_enqueued_N,tx_origin_enqueued_Nm,ty_origin_enqueued_Nm,tz_origin_enqueued_Nm,tx_com_enqueued_Nm,ty_com_enqueued_Nm,tz_com_enqueued_Nm,r_com_world_x,r_com_world_y,r_com_world_z,valid_enqueued\n";
    out.flush();status("WAITING_FOR_FIRST_STEP");
  }
  void PreUpdate(const gs::UpdateInfo &info,gs::EntityComponentManager &ecm) override {
    if(info.paused||done)return;
    try {
      const double sim=std::chrono::duration<double>(info.simTime).count(),dt=std::chrono::duration<double>(info.dt).count();
      if(!std::isfinite(sim)||!std::isfinite(dt)||dt<=0||dt>.004+1e-9||sim<=lastSim)
        throw std::runtime_error("Invalid/nonmonotone physics time or step >4ms");
      if(lastSim>=0&&std::abs(sim-lastSim-dt)>1e-8)throw std::runtime_error("Unrecorded physics step");
      lastSim=sim;maxDt=std::max(maxDt,dt);
      if(base==gs::kNullEntity) {
        const auto model=ecm.EntityByComponents(gs::components::Model(),gs::components::Name(modelName));
        if(model==gs::kNullEntity)throw std::runtime_error("Requested model missing");
        base=gs::Model(model).LinkByName(ecm,linkName);
        if(base==gs::kNullEntity)throw std::runtime_error("Requested link missing");
      }
      if(!anchored&&std::filesystem::exists(startPath)) {
        std::ifstream s(startPath);std::string value,extra;std::getline(s,value);
        if(value.rfind("t0_sim_s=",0)==0)value=value.substr(9);
        while(std::getline(s,extra))if(!trim(extra).empty())throw std::runtime_error("Invalid extra activation data");
        t0=number(value);
        if(t0<0||sim-t0>.05||(!events.empty()&&sim>=t0+events.front().start))
          throw std::runtime_error("Activation read late; full task/pulse coverage cannot be verified");
        anchored=true;status("ARMED");
      }
      const bool stop=std::filesystem::exists(stopPath);
      const double t=anchored?sim-t0:-1.;
      const bool inTask=anchored&&t>=0&&t<duration;
      const auto requested=(!stop&&inTask)?sample(events,t):std::array<double,6>{};
      const gz::math::Vector3d f(requested[0],requested[1],requested[2]),tau(requested[3],requested[4],requested[5]);
      const gs::Link link(base);const auto pose=link.WorldPose(ecm),com=link.WorldInertialPose(ecm);
      if(!pose||!com||!finite(pose->Pos())||!finite(com->Pos()))throw std::runtime_error("Invalid pose/CoM");
      const auto offset=com->Pos()-pose->Pos();const auto before=wrench(ecm,base);
      link.AddWorldForce(ecm,f);
      link.AddWorldWrench(ecm,gz::math::Vector3d::Zero,tau);
      const auto after=wrench(ecm,base);const auto df=after[0]-before[0],dm=after[1]-before[1];
      const auto couple=dm-offset.Cross(df);
      const double error=std::max(maximum(df-f),maximum(couple-tau));
      if(!finite(df)||!finite(dm)||!finite(couple)||error>1e-9)throw std::runtime_error("ECS wrench enqueue mismatch");
      maxError=std::max(maxError,error);
      if(inTask) {
        if(firstTask<0)firstTask=sim;
        lastTask=sim;++taskRows;
        const std::array<double,6> actual{df.X(),df.Y(),df.Z(),couple.X(),couple.Y(),couple.Z()};
        for(std::size_t k=0;k<6;++k)impulse[k]+=actual[k]*dt;
      }
      out<<sim<<','<<t<<','<<dt<<','<<int(inTask)<<','<<int(maximum(f)>0||maximum(tau)>0);
      vector(f);vector(tau);vector(df);vector(dm);vector(couple);vector(offset);out<<",1\n";
      if(++rows%100==0)out.flush();if(!out)throw std::runtime_error("Disturbance log write failed");
      if(!ready) {ready=true;out.flush();status(anchored?"ARMED":"READY_WAITING_START");}
      if(stop) {done=true;out.flush();status("ABORTED","stop_path observed");return;}
      if(anchored&&t>=duration) {
        if(!taskRows||firstTask-t0>.05||sim-lastTask>maxDt+1e-8)throw std::runtime_error("Incomplete native task coverage");
        for(std::size_t k=0;k<6;++k) {
          double tolerance=1e-8;
          for(const auto &e:events)tolerance+=2*maxDt*std::abs(e.wrench[k]);
          if(std::abs(impulse[k]-expectedImpulse[k])>tolerance)throw std::runtime_error("Impulse differs from immutable schedule");
        }
        done=true;out.flush();status("SUCCESS","Full waveform enqueued with checked physics-step force and free couple");
      }
    } catch(const std::exception &e) {done=true;out.flush();status("FAILED",e.what());}
  }
  ~Disturbance()override{if(out)out.flush();}
};
#endif
}
#ifndef WBCLOSED_DISTURBANCE_SELF_TEST
GZ_ADD_PLUGIN(wbclosed::Disturbance,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(wbclosed::Disturbance,"wbclosed::Disturbance")
#else
int main() {
  using namespace wbclosed;
  try {
    const std::vector<Event> e{parseEvent("5,7,.8,0,0,0,0,0",24),parseEvent("13,15,-.8,0,0,0,0,0",24)};
    if(sample(e,4.999)[0]!=0||sample(e,5)[0]!=.8||sample(e,7)[0]!=0||sample(e,13)[0]!=-.8||sample(e,15)[0]!=0)
      throw std::runtime_error("Half-open schedule test failed");
    const std::vector<Event> p{parseEvent("5,7,0,0,0,0,.03,0",24)};
    if(sample(p,6)[4]!=.03||sample(p,6)[0]!=0)throw std::runtime_error("Couple slot test failed");
    int rejected=0;for(const auto&s:{"7,5,0,0,0,0,0,0","5,7,3,0,0,0,0,0","5,7,0,0,0,0,1,0","5,25,0,0,0,0,0,0","5,7,nan,0,0,0,0,0"}) {
      try{parseEvent(s,24);}catch(const std::exception&){++rejected;}
    }
    if(rejected!=5)throw std::runtime_error("Invalid event rejection test failed");
    std::cout<<"{\"passed\":true,\"checks\":12}\n";return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
#endif
