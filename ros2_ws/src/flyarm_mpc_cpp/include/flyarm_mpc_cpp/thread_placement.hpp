#pragma once
#include "flyarm_mpc_cpp/config_io.hpp"
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>
#include <map>
#include <chrono>

namespace flyarm_mpc_cpp {
inline std::uint64_t monotonic_ns(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("monotonic clock unavailable");return std::uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
inline std::uint64_t thread_cpu_ns(){timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t))throw std::runtime_error("thread CPU clock unavailable");return std::uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
inline std::set<int> affinity(pid_t tid){cpu_set_t mask;CPU_ZERO(&mask);if(sched_getaffinity(tid,sizeof(mask),&mask))throw std::runtime_error("Cannot read owned thread affinity");std::set<int> result;for(int i=0;i<CPU_SETSIZE;++i)if(CPU_ISSET(i,&mask))result.insert(i);return result;}
struct ThreadRow{std::uint64_t start{};std::set<int> cpus;};
using ThreadRows=std::map<pid_t,ThreadRow>;
inline ThreadRows owned_threads(){ThreadRows rows;for(const auto& entry:fs::directory_iterator("/proc/self/task")){
  const pid_t tid=std::stoi(entry.path().filename().string());std::ifstream input(entry.path()/"stat");std::string line;std::getline(input,line);
  const auto end=line.rfind(')');if(end==std::string::npos)throw std::runtime_error("Invalid owned /proc thread stat");
  std::istringstream fields(line.substr(end+2));std::string value;for(int i=0;i<=19;++i)if(!(fields>>value))throw std::runtime_error("Short owned /proc thread stat");
  rows.emplace(tid,ThreadRow{std::stoull(value),affinity(tid)});if(rows.size()>128)throw std::runtime_error("Owned thread count exceeds bound");}return rows;}
class ThreadPlacement {
 public:
  ThreadPlacement(std::int64_t executor,const std::vector<std::int64_t>& auxiliary):executor_(executor),main_(pid_t(syscall(SYS_gettid))){
    if(executor<0||executor>=CPU_SETSIZE||auxiliary.empty())throw std::invalid_argument("Explicit executor/auxiliary CPUs required");
    for(auto cpu:auxiliary)if(cpu<0||cpu>=CPU_SETSIZE||cpu==executor||!aux_.insert(int(cpu)).second)throw std::invalid_argument("Overlapping/duplicate/out-of-range worker roles");
    all_=aux_;all_.insert(executor);if(affinity(main_)!=all_)throw std::invalid_argument("Actual process mask differs from declared total worker mask");
  }
  void bind(){if(bound_)throw std::runtime_error("Worker placement already bound");auto before=owned_threads();
    for(const auto& [tid,row]:before){if(row.cpus!=all_)throw std::runtime_error("Unexpected startup worker thread mask");cpu_set_t mask;CPU_ZERO(&mask);
      const auto selected=tid==main_?std::set<int>{executor_}:aux_;for(int cpu:selected)CPU_SET(cpu,&mask);if(sched_setaffinity(tid,sizeof(mask),&mask))throw std::runtime_error("Cannot bind owned worker thread");}
    baseline_=owned_threads();if(before.size()!=baseline_.size())throw std::runtime_error("Worker thread created during binding");
    for(const auto& [tid,row]:baseline_)if(!before.count(tid)||before.at(tid).start!=row.start)throw std::runtime_error("Worker TID changed during binding");
    bound_=true;check("ready");
  }
  std::array<std::uint64_t,2> observe()const{const auto tid=pid_t(syscall(SYS_gettid));const int cpu=sched_getcpu();
    if(!bound_||tid!=main_||cpu!=executor_||affinity(tid)!=std::set<int>{executor_})throw std::runtime_error("Executor TID/CPU/singleton affinity changed");
    return {std::uint64_t(tid),std::uint64_t(cpu)};}
  void check(const char* stage){auto rows=owned_threads();if(!bound_||rows.size()!=baseline_.size())throw std::runtime_error("Worker thread identity set changed");
    Json::Value snapshot;snapshot["stage"]=stage;snapshot["monotonic_ns"]=Json::UInt64(monotonic_ns());snapshot["threads"]=Json::Value(Json::arrayValue);
    for(const auto& [tid,row]:rows){const auto expected=tid==main_?std::set<int>{executor_}:aux_;
      if(!baseline_.count(tid)||baseline_.at(tid).start!=row.start||row.cpus!=expected)throw std::runtime_error("Unknown/reused/misplaced worker thread");
      Json::Value value;value["tid"]=int(tid);value["starttime_ticks"]=Json::UInt64(row.start);value["cpus"]=Json::Value(Json::arrayValue);for(int cpu:row.cpus)value["cpus"].append(cpu);snapshot["threads"].append(std::move(value));}
    stages_.append(std::move(snapshot));observe();}
  Json::Value report()const{Json::Value out;out["kind"]="cpp_worker_thread_placement_v1";out["pid"]=int(getpid());out["main_tid"]=int(main_);out["executor_cpu"]=executor_;out["auxiliary_cpus"]=Json::Value(Json::arrayValue);for(int cpu:aux_)out["auxiliary_cpus"].append(cpu);out["stages"]=stages_;
    out["scope"]="strict observed stage TID/starttime/masks and callback singleton checks; not continuous observation of short-lived auxiliary threads";return out;}
 private:int executor_;pid_t main_;std::set<int> aux_,all_;bool bound_{};ThreadRows baseline_;Json::Value stages_{Json::arrayValue};
};
}
