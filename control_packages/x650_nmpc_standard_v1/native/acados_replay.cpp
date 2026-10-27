// C++ call of the generated nonlinear OCP solver. No Python or ROS in solve().
#include "acados_solver_x650_standard.h"
#include "x650_model_identity.hpp"
#include "benchmark_io.hpp"
#include <chrono>
#include <iostream>
#include <memory>
namespace b=x650_benchmark;
using Clock=std::chrono::steady_clock;
std::vector<double> row(const Json::Value& matrix,int index,int width){
  b::require(matrix.isArray()&&index>=0&&index<int(matrix.size()),"Missing matrix row");
  auto v=b::vector(matrix[index],100000);b::require(int(v.size())==width,"Matrix width mismatch");return v;
}
int main(int argc,char** argv){try{
  b::require(argc==5,"usage: x650_standard_native_replay case.json SHA256 output.json repetitions");
  b::require(!b::fs::exists(argv[3]),"Refusing to replace numerical evidence");
  const int repetitions=std::stoi(argv[4]);b::require(repetitions>=1&&repetitions<=50,"Repetitions 1..50 required");
  std::ifstream input(argv[1],std::ios::binary);b::require(bool(input),"Cannot open bound case");
  const std::string bytes((std::istreambuf_iterator<char>(input)),{});
  b::require(b::is_sha256(argv[2])&&b::sha256_bytes(bytes)==argv[2],"Input SHA256 differs");
  const auto d=b::parse_json(bytes);
  b::require(d["kind"].asString()=="x650_standard_nmpc_numeric_case_v1","Unexpected numeric case schema");
  b::require(d["graph_sha256"].asString()==x650_standard_generated::graph_sha256&&
    d["config_sha256"].asString()==x650_standard_generated::physical_config_sha256,
    "Numeric case does not match the compiled nonlinear model identity");
  b::require(d["N"].asInt()==X650_STANDARD_N&&d["nx"].asInt()==X650_STANDARD_NX&&
    d["nu"].asInt()==X650_STANDARD_NU&&d["np"].asInt()==X650_STANDARD_NP&&
    d["nh"].asInt()==X650_STANDARD_NH&&X650_STANDARD_NH==X650_STANDARD_NH0,
    "Case dimensions differ from generated solver");
  auto* capsule=x650_standard_acados_create_capsule();b::require(capsule,"Capsule allocation failed");
  struct Cleanup{ x650_standard_solver_capsule* c; bool created=false;
    ~Cleanup(){if(created)x650_standard_acados_free(c);x650_standard_acados_free_capsule(c);} } cleanup{capsule};
  b::require(x650_standard_acados_create(capsule)==0,"Generated solver create failed");cleanup.created=true;
  auto* cfg=x650_standard_acados_get_nlp_config(capsule);
  auto* dims=x650_standard_acados_get_nlp_dims(capsule);
  auto* in=x650_standard_acados_get_nlp_in(capsule);
  auto* out=x650_standard_acados_get_nlp_out(capsule);
  auto* solver=x650_standard_acados_get_nlp_solver(capsule);
  const int n=X650_STANDARD_N,nx=X650_STANDARD_NX,nu=X650_STANDARD_NU,np=X650_STANDARD_NP;
  b::require(d["p"].size()==unsigned(n+1)&&d["guess_x"].size()==unsigned(n+1)&&d["guess_u"].size()==unsigned(n),"Generated horizon differs");
  b::require(d["time_steps"].size()==unsigned(n)&&d["cost_scaling"].size()==unsigned(n+1),"Incomplete horizon/scaling");
  for(int k=0;k<n;++k){
    const double h=b::number(d["time_steps"][k]);
    b::require(h>0.&&h<=0.1&&std::abs(h-b::number(d["p"][k][0]))<1e-12,
      "DISCRETE model duration differs from exported horizon");
  }
  for(int k=0;k<=n;++k)b::require(b::number(d["cost_scaling"][k])==1.,"Stage h is already in symbolic cost");
  auto x0=b::vector(d["x0"],nx);b::require(int(x0.size())==nx,"Initial state width differs");
  Json::Value report;report["kind"]="x650_standard_acados_cpp_fixed_case_v1";
  report["case_sha256"]=argv[2];report["model_name"]="x650_standard";
  report["compiled_solver_mode"]=x650_standard_generated::solver_mode;
  report["compiled_graph_sha256"]=x650_standard_generated::graph_sha256;
  report["compiled_config_sha256"]=x650_standard_generated::physical_config_sha256;
  report["nx"]=nx;report["nu"]=nu;report["nodes"]=n;report["np"]=np;
  report["scope"]="Generated C nonlinear solver called by C++; fixed state/parameter replay, not online feedback or actuator qualification";
  report["samples"]=Json::Value(Json::arrayValue);
  for(int rep=0;rep<repetitions;++rep){
    const auto setup_begin=Clock::now();b::require(x650_standard_acados_reset(capsule,1,0,0,0)==0,"Reset failed");
    for(int k=0;k<=n;++k){
      auto p=row(d["p"],k,np);b::require(x650_standard_acados_update_params(capsule,k,p.data(),np)==0,"Parameter update failed");
      auto x=row(d["guess_x"],k,nx);ocp_nlp_out_set(cfg,dims,out,in,k,"x",x.data());
      if(k<n){
        auto u=row(d["guess_u"],k,nu);ocp_nlp_out_set(cfg,dims,out,in,k,"u",u.data());
        auto lo=row(d["lbu"],k,nu),hi=row(d["ubu"],k,nu);
        b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,k,"lbu",lo.data())==0,"Input lower bounds failed");
        b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,k,"ubu",hi.data())==0,"Input upper bounds failed");
        const int nh=X650_STANDARD_NH;auto lh=row(d["lh"],k,nh),uh=row(d["uh"],k,nh);
        if(nh){
          b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,k,"lh",lh.data())==0,"Constraint lower bounds failed");
          b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,k,"uh",uh.data())==0,"Constraint upper bounds failed");
        }
        double scaling=b::number(d["cost_scaling"][k]);
        b::require(ocp_nlp_cost_model_set(cfg,dims,in,k,"scaling",&scaling)==0,"Cost scaling update failed");
      }
    }
    b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"lbx",x0.data())==0,"x0 lower failed");
    b::require(ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"ubx",x0.data())==0,"x0 upper failed");
    const auto begin=Clock::now();const int status=x650_standard_acados_solve(capsule);const auto end=Clock::now();
    Json::Value sample;sample["raw_status"]=status;sample["solver_success"]=status==0;
    sample["setup_ms"]=std::chrono::duration<double,std::milli>(begin-setup_begin).count();
    sample["solve_ms"]=std::chrono::duration<double,std::milli>(end-begin).count();
    sample["plan"]=Json::Value(Json::arrayValue);sample["states"]=Json::Value(Json::arrayValue);
    for(int k=0;k<=n;++k){
      std::vector<double>x(nx);ocp_nlp_out_get(cfg,dims,out,k,"x",x.data());
      for(double v:x)b::require(std::isfinite(v),"Nonfinite solution state");sample["states"].append(b::array_json(x));
      if(k<n){std::vector<double>u(nu);ocp_nlp_out_get(cfg,dims,out,k,"u",u.data());
        for(double v:u)b::require(std::isfinite(v),"Nonfinite solution command");sample["plan"].append(b::array_json(u));}
    }
    // Residual evaluation is separately timed and is not hidden in solve_ms.
    const auto residual_begin=Clock::now();ocp_nlp_eval_residuals(solver,in,out);
    for(const auto* field:{"res_stat","res_eq","res_ineq","res_comp"}){
      double value=0.;ocp_nlp_get(solver,field,&value);
      if(std::isfinite(value))sample["acados_residuals"][field]=value;
      else sample["acados_residuals"][field]=Json::Value();
    }
    sample["residual_evaluation_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-residual_begin).count();
    sample["online_publication_approved"]=false;report["samples"].append(std::move(sample));
  }
  std::ofstream output(argv[3],std::ios::binary);b::require(bool(output),"Cannot create output");
  Json::StreamWriterBuilder writer;writer["indentation"]="  ";writer["precision"]=17;
  output<<Json::writeString(writer,report)<<'\n';output.close();b::require(bool(output),"Evidence write failed");
  std::cout<<"generated_C_calls="<<repetitions<<" output="<<argv[3]<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
