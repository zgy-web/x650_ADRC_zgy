#include "flyarm_mpc_cpp/config_io.hpp"
#include "flyarm_mpc_cpp/diagnostics_json.hpp"
#include "flyarm_mpc_cpp/bounded_record_queue.hpp"
#include "flyarm_mpc_cpp/thread_placement.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <thread>

namespace fm=flyarm_mpc_cpp;
void require(bool condition,const char* text){if(!condition)throw std::runtime_error(text);}
template<class F>void rejects(F action){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}require(rejected,"Invalid configuration/evidence was accepted");}
Json::Value config_json(){x650::Config c;Json::Value j;
#define FIELD(name) j[#name]=c.name
 FIELD(nodes);FIELD(max_iterations);FIELD(node_dt);FIELD(max_dt);FIELD(feasibility_tolerance);FIELD(kkt_tolerance);FIELD(arm_share);FIELD(joint_position_gain);FIELD(bus_period);FIELD(integration_step);FIELD(trust_radius);
#undef FIELD
#define FIELD(name) j[#name]=fm::numeric_json(c.name)
 FIELD(weights);FIELD(velocity_limit);FIELD(acceleration_limit);FIELD(q_lower);FIELD(q_upper);FIELD(physical_lower);FIELD(physical_upper);FIELD(geometry);FIELD(response);FIELD(base_position_gain);FIELD(motion);FIELD(kp);FIELD(kd);
#undef FIELD
 return j;
}
int main(){fm::fs::path folder=fm::fs::temp_directory_path()/std::string("flyarm_cpp_config_test_").append(std::to_string(getpid()));
 try{
  rejects([]{fm::ThreadPlacement p(std::int64_t(1)<<32,{1,2});});
  rejects([]{fm::ThreadPlacement p(1,{1,2});});
  rejects([]{fm::ThreadPlacement p(1,{2,2});});
  rejects([]{fm::ThreadPlacement p(1,{-1,2});});
  {fm::BoundedRecordQueue<std::unique_ptr<int>,2> queue;auto a=std::make_unique<int>(1),b=std::make_unique<int>(2),c=std::make_unique<int>(3);std::unique_ptr<int> received;
   require(queue.push(std::move(a))&&queue.push(std::move(b))&&!a&&!b,"Queue did not transfer ownership");
   require(!queue.push(std::move(c))&&c&&*c==3,"Full queue moved or lost the rejected record");
   require(queue.pop(received)&&*received==1&&queue.push(std::move(c))&&queue.pop(received)&&*received==2&&queue.pop(received)&&*received==3&&!queue.pop(received),"Queue order/full ownership differs");}
  {fm::BoundedRecordQueue<std::unique_ptr<unsigned>,4> queue;bool ordered=true;std::thread consumer([&]{for(unsigned i=0;i<10000;++i){std::unique_ptr<unsigned> received;while(!queue.pop(received))std::this_thread::yield();if(!received||*received!=i)ordered=false;}});
   for(unsigned i=0;i<10000;++i){auto record=std::make_unique<unsigned>(i);while(!queue.push(std::move(record)))std::this_thread::yield();}
   consumer.join();require(ordered,"Concurrent single-producer/consumer records changed");}
  const auto original=config_json();auto c=fm::config_from_json(original);require(c.node_dt==.04&&c.nodes==15&&c.velocity_limit[4]==.35,"Explicit config values changed");
  for(const auto& mutation:std::vector<std::function<void(Json::Value&)>>{
   [](auto& j){j.removeMember("motion");},[](auto& j){j["implicit_default"]=1;},[](auto& j){j["nodes"]=true;},
   [](auto& j){j["kp"].append(6.);},[](auto& j){j["max_dt"]=.051;},[](auto& j){j["feasibility_tolerance"]=2.1e-7;},
   [](auto& j){j["kkt_tolerance"]=.003;},[](auto& j){j["motion"][0]=std::numeric_limits<double>::quiet_NaN();}}){auto j=original;mutation(j);rejects([&]{fm::config_from_json(j);});}
  require(fm::fs::create_directory(folder),"Fresh test directory required");
  const auto source=folder/"source.txt";{std::ofstream f(source);f<<"bound artifact";}
  Json::Value bound;bound["path"]=source.string();bound["sha256"]=fm::sha256(source);require(fm::bound_path(bound)==fm::fs::canonical(source),"Artifact binding changed");
  auto wrong=bound;wrong["sha256"]=std::string(64,'0');rejects([&]{fm::bound_path(wrong);});
  const auto duplicate=folder/"duplicate.json";{std::ofstream f(duplicate);f<<"{\"x\":1,\"x\":2}";}rejects([&]{fm::read_json(duplicate);});
  Json::Value startup;startup["schema_version"]=1;startup["kind"]="cpp_full_pipeline_v1";startup["simulation_only"]=true;
  startup["task_config"]=startup["core_library"]=startup["optimization_library"]=startup["predictor_library"]=bound;startup["predictor_library"]["symbol"]="ts20_prediction_batch";
  startup["source_artifacts"].append(bound);startup["config"]=original;startup["trajectory"]["kind"]="hold";
  const auto good=folder/"startup.json";fm::write_json(good,startup);auto loaded=fm::load_startup(good,fm::sha256(good));require(loaded.config.nodes==15,"Startup config changed");
  unsigned index=0;for(const auto& mutation:std::vector<std::function<void(Json::Value&)>>{
   [](auto& j){j["simulation_only"]=1;},[](auto& j){j.removeMember("optimization_library");},[](auto& j){j["trajectory"]["kind"]="figure8";},
   [](auto& j){j["predictor_library"]["symbol"]="replacement";},[](auto& j){j["source_artifacts"].append(j["source_artifacts"][0]);}}){
    auto j=startup;mutation(j);auto p=folder/("reject_"+std::to_string(++index)+".json");fm::write_json(p,j);rejects([&]{fm::load_startup(p,fm::sha256(p));});}
  x650::StepResult result;result.diagnostics.candidate_packets.push_back({});result.diagnostics.candidate_packets[0].q_ref={.123,-.234};
  result.diagnostics.candidate_states={1,2,3};result.diagnostics.linearized_states={.1,.2,.3};result.diagnostics.C={4,5};result.diagnostics.attempts.push_back({true,true,.01,.02,.3});
  result.diagnostics.recovery_triggered=true;result.diagnostics.recovery_reason="QP infeasible";
  result.diagnostics.recovery_trust_radius=.02;result.diagnostics.recovery_stopping_violation=1e-9;
  x650::RecoveryAttempt recovery;recovery.centre_kind="original_constraints_brake_projection";recovery.rejection="";
  recovery.nonlinear_passed=recovery.original_constraints_passed=recovery.trusted=recovery.projection_attempted=recovery.projection_solved=true;
  recovery.original_primal_violation=2e-9;recovery.stopping_violation=1e-9;recovery.elapsed_ms=12.;recovery.substeps=600;
  recovery.centre_plan.assign(5*c.nodes,.01);recovery.node_times.assign(c.nodes,.02);recovery.node_states.assign(4*c.nodes,.03);recovery.node_references.assign(2*c.nodes,.04);recovery.envelope={-.1,-.2,.1,.2,-.1,-.1,.1,.1};
  result.diagnostics.recovery_attempts.push_back(recovery);result.diagnostics.subproblems=2;
  require(fm::recovery_evidence_valid(result.diagnostics,c),"Valid recovery evidence was rejected");
  const auto evidence=fm::result_json(result);result.diagnostics.candidate_packets[0].q_ref[0]=9.;
  require(evidence["diagnostics"]["candidate_packets"][0]["q_ref"][0].asDouble()==.123&&evidence["diagnostics"]["C"][1].asDouble()==5.,"Actual native evidence missing or aliased");
  result.diagnostics.linearized_states[0]=99.;require(evidence["diagnostics"]["linearized_states"][0].asDouble()==.1,"Original pre-replacement linearized prediction missing or aliased");
  result.diagnostics.recovery_attempts[0].centre_plan[0]=99.;
  require(evidence["diagnostics"]["recovery_attempts"][0]["centre_plan"][0].asDouble()==.01
      &&evidence["diagnostics"]["recovery_attempts"][0]["projection_solved"].asBool()
      &&evidence["diagnostics"]["recovery_attempts"][0]["node_states"].size()==unsigned(4*c.nodes)
      &&evidence["diagnostics"]["recovery_trust_radius"].asDouble()==.02,"Owning recovery diagnostic values missing or aliased");
  for(const auto& mutation:std::vector<std::function<void(x650::Diagnostics&)>>{
   [](auto& d){d.recovery_trust_radius=.03;},[](auto& d){d.recovery_triggered=false;},
   [](auto& d){d.recovery_attempts[0].nonlinear_passed=false;},[](auto& d){d.recovery_attempts[0].original_constraints_passed=false;},
   [](auto& d){d.recovery_attempts[0].node_states.clear();},[](auto& d){d.recovery_attempts[0].projection_attempted=false;},
   [](auto& d){d.recovery_attempts[0].original_primal_violation=1e-6;},[](auto& d){d.recovery_stopping_violation=1e-6;},
   [](auto& d){d.recovery_attempts.push_back(d.recovery_attempts.front());}}){auto altered=result.diagnostics;mutation(altered);require(!fm::recovery_evidence_valid(altered,c),"Invalid recovery evidence reached worker acceptance");}
  for(const auto& mutation:std::vector<std::function<void(x650::Diagnostics&)>>{
   [](auto& d){d.recovery_trust_radius=std::numeric_limits<double>::quiet_NaN();},
   [](auto& d){d.recovery_attempts[0].elapsed_ms=std::numeric_limits<double>::infinity();},
   [](auto& d){d.recovery_attempts[0].node_states[0]=std::numeric_limits<double>::quiet_NaN();},
   [](auto& d){d.linearized_states[0]=std::numeric_limits<double>::quiet_NaN();},
   [](auto& d){d.recovery_attempts[0].original_primal_violation=std::numeric_limits<double>::infinity();}}){auto altered=result.diagnostics;mutation(altered);rejects([&]{fm::diagnostics_json(altered);});}
  {x650::Diagnostics d;require(fm::interval_evidence_valid(d,c),"Normal empty interval metadata was rejected");
   d.subproblems=2;d.qp_columns=5*c.nodes;d.qp_rows=231;d.interval_rows=1;d.compute_ms=14.;d.interval_canonical_substeps=600;d.interval_canonical_envelope={0,0,0,0,0,0,.423,0};
   x650::IntervalPoint p;p.time=.0525;p.joint=0;p.component=1;p.source_kind=3;p.value=.423;
   p.lower=-c.velocity_limit[3]+x650::joint_safety::dq_padding;p.upper=c.velocity_limit[3]-x650::joint_safety::dq_padding;
   x650::IntervalConstraint row;row.point=p;row.affine_offset=.423;row.qp_row=230;row.gradient.assign(5*c.nodes,0.);row.gradient[3]=1.;
   row.encoding_kernel_l1=.5;row.encoding_effort_half_lsb=.5*(c.kp[0]*(2.*std::acos(-1.)/std::pow(2.,24))+c.kd[0]/512.);row.encoding_anchor_bias=.00001;
   row.encoding_lower_remainder=-row.encoding_kernel_l1*row.encoding_effort_half_lsb-row.encoding_anchor_bias;row.encoding_upper_remainder=row.encoding_kernel_l1*row.encoding_effort_half_lsb-row.encoding_anchor_bias;d.interval_constraints.push_back(row);
   x650::IntervalRepair repair;repair.control_subproblem=1;repair.added_points=repair.total_points=1;repair.canonical_substeps=600;repair.elapsed_ms=10.;repair.envelope=d.interval_canonical_envelope;repair.trigger_points={p};repair.points={p};
   repair.centre_plan.assign(5*c.nodes,0.);repair.risk_radius=.02;repair.risk_margin={.02};repair.risk_encoding_lower={row.encoding_lower_remainder};repair.risk_encoding_upper={row.encoding_upper_remainder};d.interval_repairs.push_back(repair);
   require(fm::interval_evidence_valid(d,c),"Valid compact interval metadata was rejected");
   auto full=d;full.interval_centre_plan.assign(5*c.nodes,0.);full.interval_canonical_boundaries={.02,.04};full.interval_canonical_states.assign(8,0.);
   require(fm::interval_evidence_valid(full,c),"Valid owning full interval metadata was rejected");
   auto encoded=fm::diagnostics_json(full);full.interval_constraints[0].gradient[3]=99.;full.interval_repairs[0].points[0].value=99.;full.interval_canonical_states[0]=99.;
   full.interval_repairs[0].trigger_points[0].value=99.;full.interval_repairs[0].centre_plan[0]=99.;full.interval_repairs[0].risk_margin[0]=99.;full.interval_repairs[0].risk_encoding_lower[0]=99.;
   require(encoded["interval_constraints"][0]["gradient"][3].asDouble()==1.
       &&encoded["interval_repairs"][0]["points"][0]["value"].asDouble()==.423
       &&encoded["interval_canonical_states"][0].asDouble()==0.
       &&encoded["interval_repairs"][0]["trigger_points"][0]["value"].asDouble()==.423
       &&encoded["interval_repairs"][0]["centre_plan"][0].asDouble()==0.
       &&encoded["interval_repairs"][0]["risk_margin"][0].asDouble()==.02
       &&encoded["interval_repairs"][0]["risk_radius"].asDouble()==.02
       &&encoded["interval_repairs"][0]["risk_encoding_lower"][0].asDouble()==row.encoding_lower_remainder
       &&encoded["interval_constraints"][0]["encoding_kernel_l1"].asDouble()==.5
       &&encoded["interval_constraints"][0]["encoding_anchor_bias"].asDouble()==.00001
       &&encoded["interval_rows"].asInt()==1&&encoded["interval_constraints"][0]["qp_row"].asInt()==230,
       "Copied actual interval diagnostic values were missing or aliased");
   for(const auto& mutation:std::vector<std::function<void(x650::Diagnostics&)>>{
    [](auto& v){v.interval_rows=0;},[](auto& v){v.interval_repairs.clear();},[](auto& v){v.subproblems=1;},
    [](auto& v){v.interval_constraints[0].gradient.clear();},[](auto& v){v.interval_constraints[0].gradient.assign(v.qp_columns,0.);},
    [](auto& v){v.interval_constraints[0].qp_row++;},[](auto& v){v.interval_constraints[0].point.upper=.43;},
    [](auto& v){v.interval_constraints[0].point.joint=2;},[](auto& v){v.interval_constraints[0].point.component=2;},
    [](auto& v){v.interval_constraints[0].point.source_kind=4;},[](auto& v){v.interval_constraints.push_back(v.interval_constraints[0]);v.interval_rows=2;v.interval_repairs[0].added_points=v.interval_repairs[0].total_points=2;},
    [](auto& v){v.interval_repairs[0].rejection="canonical mismatch";},[](auto& v){v.interval_repairs[0].total_points=2;},
    [](auto& v){v.interval_repairs[0].elapsed_ms=51.;},[](auto& v){v.interval_repairs[0].control_subproblem=2;},
    [](auto& v){v.interval_repairs[0].risk_radius=.03;},[](auto& v){v.interval_repairs[0].risk_margin.clear();},
    [](auto& v){v.interval_repairs[0].risk_margin[0]=-.1;},[](auto& v){v.interval_repairs[0].trigger_points.clear();},
    [](auto& v){v.interval_repairs[0].risk_encoding_lower.clear();},[](auto& v){v.interval_repairs[0].risk_encoding_upper[0]=-1.;},
    [](auto& v){v.interval_repairs[0].trigger_points[0].value=0.;},[](auto& v){v.interval_repairs[0].centre_plan.clear();},
    [](auto& v){v.interval_repairs[0].centre_plan[3]=1.;},[](auto& v){v.interval_repairs[0].points[0].value=-.2;},
    [](auto& v){v.interval_rows=513;},
    [](auto& v){v.interval_constraints[0].encoding_kernel_l1=-1.;},[](auto& v){v.interval_constraints[0].encoding_effort_half_lsb=-1.;},
    [](auto& v){v.interval_constraints[0].encoding_effort_half_lsb=.01;},
    [](auto& v){v.interval_constraints[0].encoding_lower_remainder-=.0001;},[](auto& v){v.interval_constraints[0].encoding_upper_remainder+=.0001;},
    [](auto& v){v.interval_constraints[0].encoding_anchor_bias+=.0001;},
    [](auto& v){v.interval_canonical_substeps=0;},[](auto& v){v.interval_canonical_envelope[0]=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_centre_plan.assign(5,0.);}}){auto altered=d;mutation(altered);require(!fm::interval_evidence_valid(altered,c),"Malformed interval metadata reached worker acceptance");}
   for(const auto& mutation:std::vector<std::function<void(x650::Diagnostics&)>>{
    [](auto& v){v.interval_constraints[0].affine_offset=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_constraints[0].gradient[0]=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_constraints[0].point.time=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_repairs[0].elapsed_ms=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_repairs[0].points[0].value=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_repairs[0].trigger_points[0].value=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_repairs[0].risk_radius=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_repairs[0].risk_margin[0]=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_repairs[0].centre_plan[0]=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_repairs[0].risk_encoding_lower[0]=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_repairs[0].risk_encoding_upper[0]=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_constraints[0].encoding_kernel_l1=std::numeric_limits<double>::infinity();},
    [](auto& v){v.interval_constraints[0].encoding_anchor_bias=std::numeric_limits<double>::quiet_NaN();},
    [](auto& v){v.interval_canonical_envelope[0]=std::numeric_limits<double>::infinity();}}){auto altered=d;mutation(altered);rejects([&]{fm::diagnostics_json(altered);});}
  }
  result.diagnostics.compute_ms=std::numeric_limits<double>::infinity();rejects([&]{fm::result_json(result);});
  fm::fs::remove_all(folder);std::cout<<"Strict startup/source binding and actual native diagnostics contracts passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';if(fm::fs::exists(folder))fm::fs::remove_all(folder);return 1;}}
