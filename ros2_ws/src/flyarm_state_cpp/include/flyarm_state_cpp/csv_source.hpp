#pragma once
#include "flyarm_state_cpp/source_core.hpp"
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <unordered_map>

namespace flyarm_state_cpp {
// Actual plugin logs use an unquoted ASCII header and finite numeric rows.
// Read at most 1 MiB per poll; retain incomplete final line, never infer it.
class CsvTail {
 public:
  explicit CsvTail(const std::string& path):stream_(path,std::ios::binary) {
    require(bool(stream_),"Cannot open actual TS20 CSV");partial_.reserve(8192);values_.reserve(128);
  }
  template<class F> bool poll(F&& consume) {
    stream_.clear();stream_.read(buffer_.data(),buffer_.size());const auto count=stream_.gcount();
    require(!stream_.bad(),"Actual TS20 CSV read failed");
    append(buffer_.data(),static_cast<std::size_t>(count),std::forward<F>(consume));
    return count<static_cast<std::streamsize>(buffer_.size());
  }
  template<class F> void append(const char* data,std::size_t size,F&& consume) {
    for(std::size_t begin=0;begin<size;) {
      const auto newline=std::find(data+begin,data+size,'\n');const std::size_t end=static_cast<std::size_t>(newline-data);
      partial_.append(data+begin,end-begin);
      require(partial_.size()<=65536,"Actual TS20 CSV line capacity exceeded");
      if(end==size)break;
      if(!partial_.empty()&&partial_.back()=='\r')partial_.pop_back();
      if(!partial_.empty()) {
        if(header_.empty())parse_header();else {parse_values();consume(*this);++rows_;}
      }
      partial_.clear();begin=end+1;
    }
  }
  double get(const char* key)const{const auto it=header_.find(key);require(it!=header_.end(),"Actual TS20 CSV lacks required column");return values_.at(it->second);}
  std::uint64_t rows()const{return rows_;}
  bool has_header()const{return !header_.empty();}
 private:
  void parse_header(){std::size_t start=0,index=0;
    while(true){const auto end=partial_.find(',',start);const auto key=partial_.substr(start,end==std::string::npos?end:end-start);
      require(!key.empty()&&key.find('"')==std::string::npos,"Invalid TS20 CSV header");
      require(header_.emplace(key,index++).second,"Duplicate TS20 CSV header");
      if(end==std::string::npos)break;
      start=end+1;}
    require(index<=256,"TS20 CSV header capacity exceeded");values_.resize(index);
  }
  void parse_values(){const char* current=partial_.c_str();
    for(std::size_t i=0;i<values_.size();++i){char* end=nullptr;errno=0;const double v=std::strtod(current,&end);
      require(end!=current&&errno!=ERANGE&&finite(v),"Nonfinite/malformed actual TS20 numeric row");
      require((i+1==values_.size()&&*end=='\0')||(i+1<values_.size()&&*end==','),"TS20 numeric column count differs");
      values_[i]=v;current=end+1;}
  }
  std::ifstream stream_;std::array<char,1024*1024> buffer_{};std::string partial_;
  std::unordered_map<std::string,std::size_t>header_;std::vector<double>values_;std::uint64_t rows_{};
};
inline JointRow joint_row(const CsvTail& c){JointRow r;
  r.sim=c.get("sim_s");r.begin=c.get("interval_begin_s");
  r.upper={integer(c.get("upper_sequence1")),integer(c.get("upper_sequence2"))};
  r.u={c.get("u1"),c.get("u2")};r.receipt={c.get("receipt1_s"),c.get("receipt2_s")};
  r.q_physical={c.get("q1_physical"),c.get("q2_physical")};r.q_internal={c.get("q1_internal"),c.get("q2_internal")};
  r.dq={c.get("dq1_internal"),c.get("dq2_internal")};r.r={c.get("r1"),c.get("r2")};
  r.last_sequence=integer(c.get("last_send_sequence"));r.active_sequence=integer(c.get("active_sequence"));
  r.last_send=c.get("last_send_s");r.last_apply=c.get("last_send_apply_s");r.active_send=c.get("active_send_s");r.active_apply=c.get("active_apply_s");
  r.last_q={c.get("last_qref1"),c.get("last_qref2")};r.last_dq={c.get("last_dqref1"),c.get("last_dqref2")};
  r.active_q={c.get("active_qref1"),c.get("active_qref2")};r.active_dq={c.get("active_dqref1"),c.get("active_dqref2")};return r;
}
inline Packet packet_row(const CsvTail& c){Packet p;p.sequence=integer(c.get("sequence"));p.send=c.get("send_s");p.apply=c.get("apply_s");
  p.q={c.get("qref1"),c.get("qref2")};p.dq={c.get("dqref1"),c.get("dqref2")};p.tau={c.get("tau1"),c.get("tau2")};
  p.kp={c.get("kp1"),c.get("kp2")};p.kd={c.get("kd1"),c.get("kd2")};return p;}
} // namespace flyarm_state_cpp
