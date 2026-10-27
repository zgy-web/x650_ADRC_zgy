#include "flyarm_nmpc_cpp/diagnostics_json.hpp"
#include "flyarm_nmpc_cpp/options_io.hpp"
#include <iostream>
#include <limits>
#include <unistd.h>
namespace fm=flyarm_mpc_cpp;
namespace fn=flyarm_nmpc_cpp;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
template<class F> void rejects(F call){bool rejected=false;try{call();}catch(const std::exception&){rejected=true;}require(rejected,"Expected rejection");}
int main(){fm::fs::path output;try{
  x650_nmpc::OnlineResult result;result.accepted=false;result.status="candidate_issue_deadline_exceeded";
  result.issue_age_ms=51.;result.call_elapsed_ms=30.;result.completed_monotonic_ns=90000000;
  result.solve.selected_iteration=1;result.solve.elapsed_ms=29.;result.solve.stop_reason="no_strict_nonlinear_decrease";
  result.solve.first_objective.total=12.;result.solve.first_objective.components[0]=12.;
  result.solve.selected_objective.total=8.;result.solve.selected_objective.components[0]=8.;
  x650_nmpc::Iteration first;first.index=0;first.accepted=first.native_passed=first.stopping_passed=true;first.objective=result.solve.first_objective;first.native_compute_ms=9.;first.elapsed_ms=10.;
  auto second=first;second.index=1;second.objective=result.solve.selected_objective;second.elapsed_ms=29.;second.normalized_step_inf=.002;
  result.solve.iterations={first,second};auto json=fn::diagnostics_json(result);
  require(!json["accepted_by_solver"].asBool()&&json["issue_age_ms"].asDouble()==51.&&json["call_elapsed_ms"].asDouble()==30.,"Stale rejection/timing evidence lost");
  require(json["first_objective"]["total"].asDouble()==12.&&json["selected_objective"]["total"].asDouble()==8.&&json["iterations"].size()==2,"Nonlinear objective/iterations conflated with native QP certificate");
  require(!json["offline_publication_flag"].asBool(),"Offline solver publication flag relabeled");
  result.issue_age_ms=std::numeric_limits<double>::infinity();
  const auto invalid_clock=fn::diagnostics_json(result);
  require(invalid_clock["issue_age_ms"].isNull()&&!invalid_clock["issue_age_finite"].asBool(),"Invalid original issue rejection lost to JSON nonfinite conversion");
  result.issue_age_ms=51.;
  result.actual_stopping_violation=std::numeric_limits<double>::quiet_NaN();rejects([&]{fn::diagnostics_json(result);});
  const auto executable=fm::fs::canonical("/proc/self/exe");const auto task=std::string(64,'a');
  Json::Value manifest;manifest["schema_version"]=1;manifest["kind"]="x650_online_nmpc_options_v1";manifest["simulation_only"]=true;manifest["backend"]="nmpc";manifest["task_config_sha256"]=task;
  manifest["worker_executable"]["path"]=executable.string();manifest["worker_executable"]["sha256"]=fm::sha256(executable);
  manifest["source_artifacts"]=Json::Value(Json::arrayValue);manifest["source_artifacts"].append(manifest["worker_executable"]);
  auto& options=manifest["options"];options["max_proposals"]=4;options["absolute_decrease"]=1e-10;options["relative_decrease"]=1e-7;options["proposal_budget_ms"]=40.;options["total_budget_ms"]=50.;
  output=fm::fs::temp_directory_path()/("flyarm_nmpc_options_test_"+std::to_string(getpid()));
  require(!fm::fs::exists(output),"Test output preexists");fm::fs::create_directory(output);
  const auto good=output/"good.json";fm::write_json(good,manifest);
  const auto loaded=fn::load_options(good,fm::sha256(good),task);
  require(loaded.options.total_budget_ms==50.&&loaded.options.nonlinear.evaluation_budget_ms==40.,"Options budget mapping differs");
  rejects([&]{fn::load_options(good,fm::sha256(good),std::string(64,'b'));});
  manifest["options"]["total_budget_ms"]=51.;const auto bad=output/"bad.json";fm::write_json(bad,manifest);
  rejects([&]{fn::load_options(bad,fm::sha256(bad),task);});
  fm::fs::remove(good);fm::fs::remove(bad);fm::fs::remove(output);
  std::cout<<"PASS distinct nonlinear cost, source deadline rejection, finite evidence, executable/task/options bindings\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
