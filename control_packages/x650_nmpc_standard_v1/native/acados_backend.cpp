#include "acados_backend.hpp"
#include "acados_solver_x650_standard.h"
#include "x650_standard_model/x650_standard_model.h"
#include "x650_model_identity.hpp"
#include "benchmark_io.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace x650_standard {
namespace {
using Clock=std::chrono::steady_clock;
void require(bool v,const char* message){if(!v)throw std::invalid_argument(message);}
template<class T>bool finite(const T& x){return std::all_of(x.begin(),x.end(),[](double v){return std::isfinite(v);});}
double elapsed(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
void validate(const NumericalCase& c,const std::vector<double>& plan){
  require(c.parameters.size()==(horizon_nodes+1)*parameter_dim&&
          c.horizon.size()==horizon_nodes&&plan.size()==horizon_nodes*input_dim,
          "Generated OCP input dimensions differ");
  require(finite(c.x0)&&finite(c.parameters)&&finite(c.horizon)&&finite(plan),"Nonfinite OCP data");
  for(int i=0;i<horizon_nodes;++i)
    require(c.horizon[i]>0.&&c.horizon[i]<=.1&&std::abs(c.horizon[i]-c.parameters[i*parameter_dim])<1e-12,
            "DISCRETE model step must equal original horizon duration");
}
void bind_configuration(const x650::Config& actual){
  const auto expected=x650_benchmark::config_of(x650_benchmark::parse_json(x650_standard_generated::physical_config_json));
#define SAME(field) require(actual.field==expected.field,"Config differs from compiled nonlinear model: " #field)
  SAME(nodes);SAME(max_iterations);SAME(node_dt);SAME(max_dt);SAME(command_hold_s);
  SAME(feasibility_tolerance);SAME(kkt_tolerance);SAME(arm_share);SAME(joint_position_gain);
  SAME(weights);SAME(velocity_limit);SAME(acceleration_limit);SAME(q_lower);SAME(q_upper);
  SAME(physical_lower);SAME(physical_upper);SAME(geometry);SAME(response);SAME(base_position_gain);
  SAME(motion);SAME(kp);SAME(kd);SAME(bus_period);SAME(integration_step);SAME(trust_radius);
#undef SAME
  const auto s=x650_benchmark::parse_json(x650_standard_generated::surrogate_json);
  require(s["velocity_smoothing_rad_s"].asDouble()==.05&&s["rk4_substeps_per_bus"].asInt()==10&&
          s["history_rk4_substeps_per_bus"].asInt()==20&&s["event_slots"].asInt()==22&&
          s["gravity_m_s2"].asDouble()==9.8&&s["attitude_decay_s"].asDouble()==.15&&
          s["q_padding_rad"].asDouble()==.0003&&s["dq_padding_rad_s"].asDouble()==.03&&
          s["rotation_norm_regularizer"].asDouble()==1e-12,"Compiled surrogate differs from C++ state mapping");
}
}
struct AcadosBackend::Impl {
  x650_standard_solver_capsule* capsule=nullptr;
  bool created=false;
  std::vector<const double*> args;
  std::vector<double*> results;
  std::vector<int> integer_work;
  std::vector<double> real_work;
  explicit Impl(const x650::Config& config){
    static_assert(state_dim==X650_STANDARD_NX&&input_dim==X650_STANDARD_NU&&
                  parameter_dim==X650_STANDARD_NP&&horizon_nodes==X650_STANDARD_N,
                  "Rebuild matching builder and generated solver");
    bind_configuration(config);
    require(x650_standard_dyn_disc_phi_fun_n_in()==3&&x650_standard_dyn_disc_phi_fun_n_out()==1,
            "Generated dynamics argument count differs");
    for(int i=0;i<3;++i){const int* shape=x650_standard_dyn_disc_phi_fun_sparsity_in(i);
      const int expected=i==0?state_dim:(i==1?input_dim:parameter_dim);
      require(shape&&shape[0]==expected&&shape[1]==1&&shape[2]==1,"Generated dynamics input is not the bound dense vector");}
    const int* shape=x650_standard_dyn_disc_phi_fun_sparsity_out(0);
    require(shape&&shape[0]==state_dim&&shape[1]==1&&shape[2]==1,"Generated dynamics output must be dense state vector");
    int na=0,nr=0,ni=0,nw=0;
    require(x650_standard_dyn_disc_phi_fun_work(&na,&nr,&ni,&nw)==0&&na>=3&&nr>=1&&ni>=0&&nw>=0,
            "Generated dynamics workspace mismatch");
    args.resize(na,nullptr);results.resize(nr,nullptr);integer_work.resize(ni);real_work.resize(nw);
    capsule=x650_standard_acados_create_capsule();require(capsule,"Capsule allocation failed");
    const int status=x650_standard_acados_create(capsule);
    if(status){x650_standard_acados_free_capsule(capsule);capsule=nullptr;throw std::runtime_error("Generated OCP create failed");}
    created=true;
  }
  ~Impl(){if(capsule){if(created)x650_standard_acados_free(capsule);x650_standard_acados_free_capsule(capsule);}}
  void step(const double* x,const double* u,const double* p,double* next){
    args[0]=x;args[1]=u;args[2]=p;results[0]=next;
    require(x650_standard_dyn_disc_phi_fun(args.data(),results.data(),integer_work.data(),real_work.data(),nullptr)==0,
            "Generated nonlinear transition failed");
  }
};
AcadosBackend::AcadosBackend(const x650::Config& config):impl_(std::make_unique<Impl>(config)){}
AcadosBackend::~AcadosBackend()=default;

std::vector<double> AcadosBackend::rollout(const NumericalCase& c,const std::vector<double>& plan){
  validate(c,plan);std::vector<double> states((horizon_nodes+1)*state_dim);
  std::copy(c.x0.begin(),c.x0.end(),states.begin());
  for(int i=0;i<horizon_nodes;++i)
    impl_->step(states.data()+i*state_dim,plan.data()+i*input_dim,c.parameters.data()+i*parameter_dim,
                states.data()+(i+1)*state_dim);
  require(finite(states),"Nonfinite generated nonlinear rollout");return states;
}

NativeSolveResult AcadosBackend::solve(const NumericalCase& c){
  const auto setup=Clock::now();validate(c,c.initial_plan);NativeSolveResult result;
  auto states=rollout(c,c.initial_plan);
  auto* cap=impl_->capsule;auto* cfg=x650_standard_acados_get_nlp_config(cap);
  auto* dims=x650_standard_acados_get_nlp_dims(cap);auto* in=x650_standard_acados_get_nlp_in(cap);
  auto* out=x650_standard_acados_get_nlp_out(cap);auto* solver=x650_standard_acados_get_nlp_solver(cap);
  // Same reset policy as the pinned Python reference: all primal/dual/QP
  // state is reset; fixed OCP options/bounds remain. All x/u/p are rebound below.
  require(x650_standard_acados_reset(cap,1,0,0,0)==0,"Generated solver reset failed");
  for(int i=0;i<=horizon_nodes;++i){
    // The acados setters copy data and do not mutate the passed numeric values.
    require(x650_standard_acados_update_params(cap,i,const_cast<double*>(c.parameters.data()+i*parameter_dim),parameter_dim)==0,
            "Generated parameter update failed");
    ocp_nlp_out_set(cfg,dims,out,in,i,"x",states.data()+i*state_dim);
    double scale=1.;require(ocp_nlp_cost_model_set(cfg,dims,in,i,"scaling",&scale)==0,"Cost scaling failed");
    if(i<horizon_nodes)ocp_nlp_out_set(cfg,dims,out,in,i,"u",const_cast<double*>(c.initial_plan.data()+i*input_dim));
  }
  require(ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"lbx",const_cast<double*>(c.x0.data()))==0&&
          ocp_nlp_constraints_model_set(cfg,dims,in,out,0,"ubx",const_cast<double*>(c.x0.data()))==0,"Initial state binding failed");
  const auto start=Clock::now();result.raw_status=x650_standard_acados_solve(cap);const auto done=Clock::now();
  result.setup_ms=elapsed(setup,start);result.solve_ms=elapsed(start,done);
  result.solver_success=result.raw_status==0;
  result.states.resize((horizon_nodes+1)*state_dim);result.plan.resize(horizon_nodes*input_dim);
  for(int i=0;i<=horizon_nodes;++i){
    ocp_nlp_out_get(cfg,dims,out,i,"x",result.states.data()+i*state_dim);
    if(i<horizon_nodes)ocp_nlp_out_get(cfg,dims,out,i,"u",result.plan.data()+i*input_dim);
  }
  require(finite(result.states)&&finite(result.plan),"Generated solver returned nonfinite values");
  ocp_nlp_eval_residuals(solver,in,out);
  const char* names[]={"res_stat","res_eq","res_ineq","res_comp"};
  for(int j=0;j<4;++j)ocp_nlp_get(solver,names[j],&result.residuals[j]);
  require(finite(result.residuals),"Generated solver residual nonfinite");
  result.rollout_states=rollout(c,result.plan);
  std::array<double,state_dim> next{};
  for(int i=0;i<=horizon_nodes;++i){
    for(int j=0;j<state_dim;++j){
      const auto index=i*state_dim+j;
      result.rollout_disagreement=std::max(result.rollout_disagreement,std::abs(result.states[index]-result.rollout_states[index]));
    }
    if(i==0){for(int j=0;j<state_dim;++j)result.local_dynamics_residual=std::max(result.local_dynamics_residual,std::abs(result.states[j]-c.x0[j]));}
    else {
      impl_->step(result.states.data()+(i-1)*state_dim,result.plan.data()+(i-1)*input_dim,c.parameters.data()+(i-1)*parameter_dim,next.data());
      for(int j=0;j<state_dim;++j)result.local_dynamics_residual=std::max(result.local_dynamics_residual,std::abs(result.states[i*state_dim+j]-next[j]));
    }
  }
  result.postprocess_ms=elapsed(done,Clock::now());return result;
}
} // namespace x650_standard
