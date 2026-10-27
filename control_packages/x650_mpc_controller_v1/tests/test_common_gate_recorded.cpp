// Fixed historical Memory/input provenance supplied as a SHA-bound fixture.
#include "x650_mpc/common_gate.hpp"
#include "benchmark_io.hpp"
#include <iostream>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
namespace b=x650_benchmark;
int main(int argc,char** argv){try{
 b::require(argc==3,"usage: test_x650_mpc_common_gate_recorded fixture.json sha256");
 b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"historical fixture hash differs");
 std::ifstream stream(argv[1],std::ios::binary);const std::string bytes((std::istreambuf_iterator<char>(stream)),{});
 const auto data=b::parse_json(bytes);const auto config=b::config_of(data["config"]);
 b::require(data["kind"].asString()=="legacy_mpc_common_gate_fixture_v1"&&data["cases"].size()==2,"two bound real cases required");
 Json::Value report;report["kind"]="legacy_mpc_common_gate_recorded_test_v1";
 report["fixture_sha256"]=argv[2];report["physical_experiment"]=false;
 report["rows"]=Json::Value(Json::arrayValue);unsigned passed_cases=0,rejected_cases=0;
 for(const auto& row:data["cases"]){
  const auto memory=b::memory_of(row["memory"]);const auto input=b::input_of(row["input"]);
  x650::Core direct(config,ts20_prediction_batch,x650::DiagnosticMode::Full);direct.restore(memory);
  const auto legacy=direct.step(input);b::require(legacy.diagnostics.accepted,"original native candidate not accepted");
  const auto historical_plan=b::vector(row["plan"],300);
  b::require(historical_plan.size()==legacy.diagnostics.plan.size(),"historical adopted plan shape differs");
  for(std::size_t i=0;i<historical_plan.size();++i)
   b::require(std::abs(historical_plan[i]-legacy.diagnostics.plan[i])<=1e-12,"new solve differs from successor-bound historical plan");
  x650_mpc::CommonGateController gate(config,ts20_prediction_batch);gate.restore_for_replay(memory);
  const auto result=gate.prepare(input,gate.monotonic_ns());
  b::require(result.validation_called&&result.output.has_value(),"fixture expired before actual common check");
  b::require(result.output->command==legacy.command&&result.output->diagnostics.plan==legacy.diagnostics.plan,"unchanged native plan differs");
  b::require(b::same_memory(memory,gate.committed_memory()),"prepare mutated committed history");
  const bool expected=row["expected_common_pass"].asBool();b::require(result.accepted==expected,"common admission differs from fixed expectation");
  b::require(std::abs(result.validation.stopping_violation-row["expected_stopping_violation"].asDouble())<1e-12,"actual historical stopping discrepancy changed");
  if(expected){
   ++passed_cases;b::require(gate.has_pending()&&result.token,"safe fixture lacks pending");
   gate.commit_published(result.token,result.output->command);
   b::require(b::same_memory(direct.memory(),gate.committed_memory()),"matching publication changed native Memory");
  }else{
   ++rejected_cases;b::require(result.status=="common_plan_rejected"&&!result.token&&!gate.has_pending()
    &&result.validation.native_bounds_passed&&result.validation.hold_passed&&result.validation.pending_guard_passed
    &&!result.validation.stopping_passed,"historical stopping-only rejection differs");
   bool rejected=false;try{gate.commit_published(result.token,result.output->command);}catch(const std::logic_error&){rejected=true;}
   b::require(rejected&&b::same_memory(memory,gate.committed_memory()),"rejected candidate committed");
  }
  Json::Value r;r["id"]=row["id"];r["original_core_accepted"]=true;r["common_accepted"]=result.accepted;
  r["status"]=result.status;r["stopping_violation_rad"]=result.validation.stopping_violation;
  r["core_ms"]=result.timing.core_ms;r["validation_ms"]=result.timing.validation_ms;
  r["total_issue_ms"]=result.timing.issue_age_ms;r["passed"]=true;report["rows"].append(r);
 }
 b::require(passed_cases==1&&rejected_cases==1,"safe and unsafe real fixtures both required");
 report["passed"]=true;Json::StreamWriterBuilder w;w["indentation"]="  ";std::cout<<Json::writeString(w,report)<<'\n';return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
