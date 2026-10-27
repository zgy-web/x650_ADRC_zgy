// Simulation-only Gazebo clock barrier. Does not publish actuator commands.
// stdin: pause | step <iterations> | status | resume | stop. stdout: one JSON row.
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <json/json.h>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using Clock=std::chrono::steady_clock;
struct Barrier {
  gz::transport::Node node;
  std::mutex mutex; std::condition_variable changed;
  std::int64_t sim=-1; std::uint64_t callbacks=0,iterations=0;
  bool reported_paused=false, owned_pause=false;
  std::string world;
  std::int64_t tick;
  Barrier(std::string w,std::int64_t dt):world(std::move(w)),tick(dt) {
    if(world.empty()||world.find('/')!=std::string::npos||tick<=0)throw std::runtime_error("Invalid world/tick");
    if(!node.Subscribe<gz::msgs::Clock>("/world/"+world+"/clock",[this](const gz::msgs::Clock& m){
      std::lock_guard<std::mutex> guard(mutex);sim=m.sim().sec()*1000000000LL+m.sim().nsec();++callbacks;changed.notify_all();
    }))throw std::runtime_error("Clock subscription failed");
    if(!node.Subscribe<gz::msgs::WorldStatistics>("/world/"+world+"/stats",[this](const gz::msgs::WorldStatistics& m){
      std::lock_guard<std::mutex> guard(mutex);reported_paused=m.paused();iterations=m.iterations();changed.notify_all();
    }))throw std::runtime_error("Stats subscription failed");
    std::unique_lock<std::mutex> guard(mutex);
    if(!changed.wait_for(guard,std::chrono::seconds(8),[this]{return sim>=0;}))throw std::runtime_error("No world clock");
  }
  void control(bool paused,unsigned steps=0) {
    gz::msgs::WorldControl request;request.set_pause(paused);request.set_multi_step(steps);
    gz::msgs::Boolean reply;bool result=false;
    if(!node.Request("/world/"+world+"/control",request,2000u,reply,result)||!result||!reply.data())
      throw std::runtime_error("World control service rejected");
  }
  Json::Value state() {
    std::lock_guard<std::mutex> guard(mutex);Json::Value row;
    row["sim_ns"]=Json::Int64(sim);row["clock_callbacks"]=Json::UInt64(callbacks);
    row["iterations"]=Json::UInt64(iterations);row["reported_paused"]=reported_paused;row["owns_pause"]=owned_pause;
    return row;
  }
  Json::Value pause() {
    control(true);std::unique_lock<std::mutex> guard(mutex);
    if(!changed.wait_for(guard,std::chrono::seconds(3),[this]{return reported_paused;}))throw std::runtime_error("Pause not observed");
    auto before=sim;guard.unlock();std::this_thread::sleep_for(std::chrono::milliseconds(80));guard.lock();
    if(sim!=before)throw std::runtime_error("World clock advanced after pause acknowledgement");
    owned_pause=true;guard.unlock();return state();
  }
  Json::Value step(unsigned n) {
    if(!owned_pause||n<1||n>250)throw std::runtime_error("Step requires owned pause and 1..250 ticks");
    std::int64_t before;{std::lock_guard<std::mutex> guard(mutex);before=sim;}
    auto target=before+tick*n;control(true,n);std::unique_lock<std::mutex> guard(mutex);
    if(!changed.wait_for(guard,std::chrono::seconds(4),[this,target]{return sim>=target;}))throw std::runtime_error("Step clock timeout");
    if(sim!=target)throw std::runtime_error("Step overshot exact requested clock boundary");
    guard.unlock();auto row=state();row["before_sim_ns"]=Json::Int64(before);row["requested_ticks"]=n;return row;
  }
};
int main(int argc,char** argv) {
  Json::StreamWriterBuilder writer;writer["indentation"]="";
  try {
    if(argc!=3)throw std::runtime_error("Usage: simulation_stepper world physics_tick_ns");
    Barrier barrier(argv[1],std::stoll(argv[2]));std::string line;
    while(std::getline(std::cin,line)) {
      auto begin=Clock::now();Json::Value result;std::istringstream input(line);std::string op,extra;input>>op;
      if(op=="step") {unsigned n=0;if(!(input>>n)||(input>>extra))throw std::runtime_error("Malformed step");result=barrier.step(n);}
      else {
        if(input>>extra)throw std::runtime_error("Unexpected command argument");
        if(op=="pause")result=barrier.pause();
        else if(op=="status")result=barrier.state();
        else if(op=="resume") {barrier.control(false);barrier.owned_pause=false;result=barrier.state();}
        else if(op=="stop") {result=barrier.state();result["stopped"]=true;}
        else throw std::runtime_error("Unknown command");
      }
      result["op"]=op;result["ok"]=true;
      result["wall_ns"]=Json::UInt64(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-begin).count());
      std::cout<<Json::writeString(writer,result)<<std::endl;
      if(op=="stop")return 0;
    }
    // An unexpected owner disconnect never silently resumes the world.
    return 2;
  }catch(const std::exception& e){Json::Value error;error["ok"]=false;error["error"]=e.what();std::cout<<Json::writeString(writer,error)<<std::endl;return 1;}
}
