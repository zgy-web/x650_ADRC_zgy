// Offline parity: original selected CSV visibility prefixes, never fitted time.
#include "flyarm_state_cpp/source_core.hpp"
#include "flyarm_state_cpp/csv_source.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
using namespace flyarm_state_cpp;
template<class C>static void array(std::ostream& o,const C& v){o<<'[';bool first=true;for(auto x:v){if(!first)o<<',';first=false;o<<x;}o<<']';}
static V3 v3(const CsvTail& c,const char* name){V3 v{};for(unsigned i=0;i<3;++i)v[i]=c.get((std::string(name)+std::to_string(i)).c_str());return v;}
static std::uint8_t counter(const CsvTail& c,const std::string& name){const auto n=integer(c.get(name.c_str()));require(n<=255,"Reset counter out of range");return n;}
static Origin origin(const CsvTail& c,const std::string& p){Origin o;o.lat=c.get((p+"lat").c_str());o.lon=c.get((p+"lon").c_str());o.alt=c.get((p+"alt").c_str());
  o.ref_timestamp=integer(c.get((p+"ref_timestamp").c_str()));o.xy_global=integer(c.get((p+"xy_global").c_str()));o.z_global=integer(c.get((p+"z_global").c_str()));
  o.xy_reset=counter(c,p+"xy_reset");o.z_reset=counter(c,p+"z_reset");o.heading_reset=counter(c,p+"heading_reset");
  o.vxy_reset=counter(c,p+"vxy_reset");o.vz_reset=counter(c,p+"vz_reset");o.quat_reset=counter(c,p+"quat_reset");return o;}
int main(int argc,char** argv){try{
  require(argc==7,"Usage: replay_source_core state.csv packets.csv fixture.csv period delay output.jsonl");
  require(!std::filesystem::exists(argv[6]),"Replay output must not exist");
  CsvTail states(argv[1]),packets(argv[2]),fixtures(argv[3]);std::vector<JointRow> rows;std::vector<Packet> pkts;
  while(!states.poll([&](const CsvTail& c){rows.push_back(joint_row(c));})){}
  while(!packets.poll([&](const CsvTail& c){pkts.push_back(packet_row(c));})){}
  JointHistory history(std::stod(argv[4]),std::stod(argv[5]));std::unique_ptr<EstimatorGuard> guard;
  std::size_t ri=0,pi=0,count=0;double prior_cutoff=-1,prior_packet=-1;std::ofstream out(argv[6]);out<<std::setprecision(17);
  auto run=[&](const CsvTail& c){const double now=c.get("now"),task=c.get("task"),cutoff=c.get("state_cutoff"),packet_cutoff=c.get("packet_cutoff");
    require(cutoff>=prior_cutoff&&packet_cutoff>=prior_packet,"Original selected visibility prefix regressed");prior_cutoff=cutoff;prior_packet=packet_cutoff;
    while(ri<rows.size()&&rows[ri].sim<=cutoff)history.append(rows[ri++]);
    while(pi<pkts.size()&&pkts[pi].send<=packet_cutoff)history.append(pkts[pi++]);
    const auto joint=history.select(now,task);require(joint.row.sim==cutoff,"Original joint stamp missing in physical CSV");
    if(!guard)guard=std::make_unique<EstimatorGuard>(origin(c,"b_"));
    Position p;p.origin=origin(c,"o_");p.p=v3(c,"p");p.v=v3(c,"v");p.a=v3(c,"a");
    p.xy_valid=c.get("xy_valid");p.z_valid=c.get("z_valid");p.vxy_valid=c.get("v_xy_valid");p.vz_valid=c.get("v_z_valid");
    p.heading_good=c.get("heading_good_for_control");p.dead_reckoning=c.get("dead_reckoning");
    Q4 q{};for(unsigned i=0;i<4;++i)q[i]=c.get(("quat"+std::to_string(i)).c_str());
    Flags flags{bool(c.get("cs_ev_pos")),bool(c.get("cs_ev_vel")),bool(c.get("cs_ev_hgt")),bool(c.get("cs_gnss_pos")),bool(c.get("cs_gnss_vel")),
      bool(c.get("cs_gps_hgt")),bool(c.get("cs_baro_hgt")),bool(c.get("cs_inertial_dead_reckoning")),bool(c.get("cs_wind_dead_reckoning"))};
    std::array<Receipt,5> receipts{};for(unsigned i=0;i<5;++i){const auto prefix="rc"+std::to_string(i)+"_";
      receipts[i].sequence=integer(c.get((prefix+"seq").c_str()));receipts[i].sim_s=c.get((prefix+"sim").c_str());
      receipts[i].wall_ns=static_cast<std::uint64_t>(std::llround(c.get((prefix+"wall").c_str())*1e9));}
    const auto state=guard->read(now,static_cast<std::uint64_t>(std::llround(c.get("wall")*1e9)),p,q,v3(c,"omega"),flags,receipts);
    out<<"{\"index\":"<<count<<",\"p\":";array(out,state.p);out<<",\"v\":";array(out,state.v);out<<",\"a\":";array(out,state.a);
    out<<",\"R\":";array(out,state.R);out<<",\"omega\":";array(out,state.omega);out<<",\"state_time\":"<<state.state_time;
    out<<",\"joint_time\":"<<joint.row.sim<<",\"q\":";array(out,joint.row.q_physical);out<<",\"dq\":";array(out,joint.row.dq);
    out<<",\"r\":";array(out,joint.row.r);out<<",\"reference_time\":"<<joint.reference_time<<",\"bus_origin\":"<<joint.bus_origin<<",\"commands\":[";
    bool first=true;for(const auto& command:joint.commands){if(!first)out<<',';first=false;out<<'['<<command.when<<','<<command.u[0]<<','<<command.u[1]<<']';}
    out<<"],\"packets\":[";first=true;for(const auto& packet:joint.packets){if(!first)out<<',';first=false;out<<'['<<packet.send;
      for(const auto& v:{packet.q,packet.dq,packet.tau,packet.kp,packet.kd})for(double x:v)out<<','<<x;
      out<<']';}out<<"]}\n";++count;
  };
  while(!fixtures.poll(run)){}out.close();require(bool(out),"Replay output write failed");std::cout<<"Converted "<<count<<" archived source cases\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
