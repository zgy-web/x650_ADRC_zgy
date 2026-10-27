#include "flyarm_nmpc_cpp/fast_json.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
Json::Value parse(const std::string& text){Json::CharReaderBuilder b;Json::Value v;std::string error;std::istringstream in(text);require(Json::parseFromStream(b,in,&v,&error),"Encoded JSON failed parse");return v;}
int main(){try{
  Json::Value root;root["signed_min"]=Json::Int64(std::numeric_limits<std::int64_t>::min());root["unsigned_max"]=Json::UInt64(std::numeric_limits<std::uint64_t>::max());
  root["boolean"]=true;root["null"]=Json::Value();root["escaped"]=std::string("quoted\"\\\n\t\b\f\r\0tail",18);root["utf8"]=u8"关节诊断";
  root["empty_array"]=Json::Value(Json::arrayValue);root["empty_object"]=Json::Value(Json::objectValue);
  auto& real=root["real"];real=Json::Value(Json::arrayValue);
  for(double x:{0.,-0.,1.,-1.,std::numeric_limits<double>::denorm_min(),std::numeric_limits<double>::min(),std::numeric_limits<double>::max(),std::nextafter(1.,2.)})real.append(x);
  std::mt19937_64 rng(78234);for(int i=0;i<20000;++i){const std::uint64_t bits=rng();double v;std::memcpy(&v,&bits,sizeof(v));if(std::isfinite(v))real.append(v);}
  const auto result=parse(flyarm_nmpc_cpp::encode_json(root));require(result==root,"JSON value/type changed");
  for(Json::ArrayIndex i=0;i<real.size();++i){const double x=real[i].asDouble(),y=result["real"][i].asDouble();require(std::memcmp(&x,&y,sizeof(x))==0,"IEEE double bits changed");}
  bool rejected=false;try{flyarm_nmpc_cpp::encode_json(Json::Value(std::numeric_limits<double>::quiet_NaN()));}catch(const std::exception&){rejected=true;}require(rejected,"Nonfinite real accepted");
  std::cout<<"PASS 20k finite bit-pattern round trips, exact UInt64/Int64, signed zero, controls, embedded NUL, UTF-8, nested/empty values and nonfinite rejection\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
