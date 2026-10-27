#include "x650_nmpc/offline_solver.hpp"
#include "benchmark_io.hpp"
#include <dlfcn.h>
#include <iostream>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
extern "C" unsigned x650_optimization_abi_version();
namespace b=x650_benchmark;
Json::Value objective(const x650_nmpc::Objective& o){Json::Value j;j["total"]=o.total;j["components"]=b::array_json(o.components);return j;}
Json::Value loaded(void* symbol){Dl_info info{};b::require(dladdr(symbol,&info)&&info.dli_fname,"Cannot identify library");auto p=b::fs::canonical(info.dli_fname);Json::Value v;v["path"]=p.string();v["sha256"]=b::sha256_file(p);return v;}
int main(int argc,char**argv){Json::Value report;report["kind"]="x650_upper_nmpc_nonlinear_iteration_offline_v1";report["physical_validation"]=false;report["online_approved"]=false;report["complete"]=false;
try{
 b::require(argc==3,"usage: x650_nmpc_offline_replay fixture.json sha256");
 b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"Fixture SHA mismatch");
 std::ifstream file(argv[1]);std::string text((std::istreambuf_iterator<char>(file)),{});auto data=b::dataset_of(b::parse_json(text));
 report["input_sha256"]=argv[2];report["loaded_core"]=loaded(reinterpret_cast<void*>(&x650_mpc_core_abi_version));report["loaded_optimizer"]=loaded(reinterpret_cast<void*>(&x650_optimization_abi_version));report["loaded_predictor"]=loaded(reinterpret_cast<void*>(&ts20_prediction_batch));
 x650_nmpc::Options options;options.evaluation_budget_ms=200.;report["options"]["max_proposals"]=options.max_proposals;
 report["options"]["offline_evaluation_budget_ms"]=options.evaluation_budget_ms;report["options"]["absolute_decrease"]=options.absolute_decrease;report["options"]["relative_decrease"]=options.relative_decrease;
 report["rows"]=Json::Value(Json::arrayValue);int successes=0,improved=0,over50=0,timing_unknown=0;double max_ms=0;
 for(const auto& row:data.cases){Json::Value item;item["id"]=row.id;item["task_time_s"]=row.input.t;item["first_dt_s"]=row.input.dt;
   try{
    const auto memory=row.memory;const auto result=x650_nmpc::solve_offline(data.config,ts20_prediction_batch,row.memory,row.input,options);
    item["first_objective"]=objective(result.first_objective);item["selected_objective"]=objective(result.selected_objective);
    item["selected_iteration"]=result.selected_iteration;item["stop_reason"]=result.stop_reason;
    item["total_ms"]=result.elapsed_ms;item["within_50ms"]=result.elapsed_ms<=50.;item["within_offline_budget"]=result.within_evaluation_budget;
    item["input_memory_unchanged"]=b::same_memory(memory,row.memory);
    item["single_step_increment"]=result.proposed_memory.step_count==row.memory.step_count+1;
    auto expected_history=row.memory.joint_history;
    expected_history.push_back({row.input.t,result.output.joint_velocity});
    expected_history.erase(std::remove_if(expected_history.begin(),expected_history.end(),[&](const x650::Command& x){return x.time<row.input.t-.75;}),expected_history.end());
    bool history_matches=result.proposed_memory.joint_history.size()==expected_history.size();
    if(history_matches)for(std::size_t i=0;i<expected_history.size();++i)
      history_matches=history_matches&&expected_history[i].time==result.proposed_memory.joint_history[i].time&&expected_history[i].velocity==result.proposed_memory.joint_history[i].velocity;
    item["single_history_increment"]=history_matches;
    item["selected_plan_memory_matches"]=result.proposed_memory.plan==result.output.diagnostics.plan&&result.proposed_memory.previous_command==result.output.command;
    item["online_approved"]=result.approved_for_online_publication;
    item["selected_command"]=b::array_json(result.output.command);item["selected_plan"]=b::array_json(result.output.diagnostics.plan);
    item["native_envelope"]=b::array_json(result.output.diagnostics.candidate_envelope);item["hold_envelope"]=b::array_json(result.output.diagnostics.hold_guard.envelope);
    item["iterations"]=Json::Value(Json::arrayValue);double previous=-1.;bool monotone=true,hard=true;
    for(const auto& it:result.iterations){Json::Value a;a["index"]=it.index;a["accepted"]=it.accepted;a["reason"]=it.reason;a["objective"]=objective(it.objective);a["native_passed"]=it.native_passed;a["stopping_passed"]=it.stopping_passed;a["stopping_violation"]=it.stopping_violation;a["native_ms"]=it.native_compute_ms;a["elapsed_ms"]=it.elapsed_ms;a["normalized_step_inf"]=it.normalized_step_inf;item["iterations"].append(a);
      if(it.accepted){if(previous>=0)monotone=monotone&&it.objective.total<previous;previous=it.objective.total;hard=hard&&it.native_passed&&it.stopping_passed;}
    }
    item["accepted_costs_strictly_decrease"]=monotone;item["accepted_hard_checks_pass"]=hard;
    item["passed"]=monotone&&hard&&item["input_memory_unchanged"].asBool()&&item["single_step_increment"].asBool()&&item["single_history_increment"].asBool()&&item["selected_plan_memory_matches"].asBool()&&!result.approved_for_online_publication;
    if(item["passed"].asBool())++successes;
    if(result.selected_iteration>0)++improved;
   }catch(const x650_nmpc::OfflineError&e){item["passed"]=false;item["error"]=e.what();item["stop_reason"]=e.diagnostics.stop_reason;item["total_ms"]=e.diagnostics.elapsed_ms;item["iterations"]=Json::Value(Json::arrayValue);
     for(const auto& it:e.diagnostics.iterations){Json::Value a;a["index"]=it.index;a["accepted"]=it.accepted;a["reason"]=it.reason;a["objective"]=objective(it.objective);a["stopping_violation"]=it.stopping_violation;item["iterations"].append(a);}
   }catch(const std::exception&e){item["passed"]=false;item["error"]=e.what();}
   if(item.isMember("total_ms")){const double value=item["total_ms"].asDouble();if(value>50)++over50;max_ms=std::max(max_ms,value);}else ++timing_unknown;
   report["rows"].append(item);
 }
 report["complete"]=true;report["case_count"]=Json::UInt64(data.cases.size());report["passed_cases"]=successes;report["improved_cases"]=improved;report["above_50ms_cases"]=over50;report["timing_unknown_cases"]=timing_unknown;report["max_total_ms"]=max_ms;report["passed"]=successes==int(data.cases.size());
 report["scope"]="Independent fixed snapshots only; no physical rollout, flight performance, nonlinear KKT/global convergence, or real-time scheduling qualification. Same original event model and input interface; bounded QP proposals accepted with true nonlinear cost. Actual actuator history is immutable within each solve.";
}catch(const std::exception&e){report["passed"]=false;report["error"]=e.what();}
Json::StreamWriterBuilder w;w["indentation"]="  ";std::cout<<Json::writeString(w,report)<<'\n';return report["passed"].asBool()?0:1;
}
