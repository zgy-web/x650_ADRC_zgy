#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace flyarm_control_gateway {
struct ConsumptionRow {
  double sim{};
  std::array<std::uint64_t,2> counter{};
  std::array<double,2> velocity{},consumed{};
};
// Independent append-only reader. Only a newline-terminated complete physics
// row may acknowledge consumption. It never infers it from a sent command.
class ConsumptionCsv {
 public:
  std::optional<ConsumptionRow> append(const std::string &chunk) {
    partial_+=chunk;
    if(partial_.size()>2*1024*1024) throw std::runtime_error("Consumption CSV buffer limit");
    std::optional<ConsumptionRow> result;
    std::size_t offset=0,end;
    while((end=partial_.find('\n',offset))!=std::string::npos) {
      auto line=partial_.substr(offset,end-offset);offset=end+1;
      if(!line.empty() && line.back()=='\r')line.pop_back();
      if(line.empty())continue;
      auto fields=split(line);
      if(header_.empty()) {
        header_=fields;
        sim_=index("sim_s");
        for(int j=0;j<2;++j) {
          const auto n=std::to_string(j+1);
          counter_[j]=index("upper_sequence"+n);velocity_[j]=index("u"+n);consumed_[j]=index("receipt"+n+"_s");
        }
        continue;
      }
      if(fields.size()!=header_.size())throw std::runtime_error("Incomplete/malformed complete consumption row");
      std::vector<double> values;
      for(const auto &field:fields) {
        std::size_t pos=0;double v=std::stod(field,&pos);
        if(pos!=field.size() || !std::isfinite(v))throw std::runtime_error("Nonfinite/invalid consumption value");
        values.push_back(v);
      }
      ConsumptionRow r;r.sim=values[sim_];
      if(r.sim<0 || (last_ && (r.sim<=last_->sim || r.sim-last_->sim>.02000001)))
        throw std::runtime_error("Consumption physics chronology/gap");
      for(int j=0;j<2;++j) {
        auto count=values[counter_[j]];
        if(count<0 || count>9007199254740991. || count!=std::floor(count))throw std::runtime_error("Invalid consumption counter");
        r.counter[j]=static_cast<std::uint64_t>(count);
        r.velocity[j]=values[velocity_[j]];r.consumed[j]=values[consumed_[j]];
        if(r.consumed[j]>r.sim+1e-9 || (last_ && (r.counter[j]<last_->counter[j] ||
            (r.counter[j]==last_->counter[j] && (r.velocity[j]!=last_->velocity[j] || r.consumed[j]!=last_->consumed[j])))))
          throw std::runtime_error("Consumption counter/value/time changed inconsistently");
      }
      last_=r;result=r;
    }
    partial_.erase(0,offset);
    return result;
  }
 private:
  static std::vector<std::string> split(const std::string &line) {
    std::vector<std::string> v;std::size_t start=0,end;
    while((end=line.find(',',start))!=std::string::npos) {v.push_back(line.substr(start,end-start));start=end+1;}
    v.push_back(line.substr(start));return v;
  }
  std::size_t index(const std::string &name) const {
    std::size_t found=header_.size();
    for(std::size_t i=0;i<header_.size();++i)if(header_[i]==name) {
      if(found!=header_.size())throw std::runtime_error("Duplicate consumption field");
      found=i;
    }
    if(found==header_.size())throw std::runtime_error("Missing consumption field "+name);
    return found;
  }
  std::string partial_;std::vector<std::string> header_;std::optional<ConsumptionRow> last_;
  std::size_t sim_{};std::array<std::size_t,2> counter_{},velocity_{},consumed_{};
};
}
