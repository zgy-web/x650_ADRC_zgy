// Optional numerical accelerator for ts20_prediction.py, no hardware I/O.
// Own exponential-midpoint implementation; normalized TS20 interface values
// remain parameters. It contains no firmware, transport, or physical inertias.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

namespace {
struct Transition {double a00,a01,a10,a11,b0,b1;};
struct State {double q,v;};
struct Core {
  double J,b,Fc,offset,delay,Fs,delta,vs,kp,kd,maxstep;
  Transition transition(double h) const {
    const double k=kp/J, d=(b+kd)/J, half=-d/2, disc=half*half-k;
    double c,s;
    if(std::abs(disc)*h*h<1e-14){c=1;s=h;}
    else if(disc>0){const auto w=std::sqrt(disc);c=std::cosh(w*h);s=std::sinh(w*h)/w;}
    else{const auto w=std::sqrt(-disc);c=std::cos(w*h);s=std::sin(w*h)/w;}
    const double e=std::exp(half*h);
    Transition A{e*(c-half*s),e*s,e*(-k*s),e*(c+half*s),0,0};
    if(kp>0){A.b0=(1-A.a00)/kp;A.b1=-A.a10/kp;}
    else if(d>0){const auto gain=-std::expm1(-d*h)/d;A.b0=(h-gain)/(b+kd);A.b1=gain/J;}
    else{A.b0=h*h/(2*J);A.b1=h/J;}
    return A;
  }
  static State step(State x,Transition a,double effort){
    return {a.a00*x.q+a.a01*x.v+a.b0*effort,a.a10*x.q+a.a11*x.v+a.b1*effort};
  }
  double friction(double v)const{
    const auto ratio=std::min(28.,std::abs(v)/vs);return Fc+delta*std::exp(-ratio*ratio);
  }
};
}

extern "C" int ts20_prediction_abi(){return 1;}

extern "C" int ts20_prediction_batch(const double *p,const double *g,double maxstep,double start,
    const double *initial,const double *knots,const double *forcing,int packets,
    const double *boundaries,int count,int track,double *states,int *modes,int *zeros,
    double *envelope,double *event_output,int event_capacity,int *event_count,
    int *substep_count,char *error,int error_size){
  try {
    if(!p||!g||!initial||!knots||!forcing||!boundaries||!states||!modes||!zeros||!envelope||!event_output||!event_count||!substep_count||!error||packets<1||count<1)
      throw std::runtime_error("native TS20 invalid array contract");
    std::array<State,2> x{{{initial[0],initial[2]},{initial[1],initial[3]}}};
    for(int j=0;j<2;++j){envelope[j]=envelope[2+j]=x[j].q;envelope[4+j]=envelope[6+j]=x[j].v;}
    *event_count=0;*substep_count=0;
    auto event=[&](double time,int joint,int kind,State value){
      if(*event_count>=event_capacity)throw std::runtime_error("native TS20 extrema capacity exhausted");
      auto *out=event_output+(*event_count)++*5;out[0]=time;out[1]=joint;out[2]=kind;out[3]=value.q;out[4]=value.v;
    };
    int active=0;while(active+1<packets&&knots[active+1]<=start+1e-11)++active;
    double cursor=start;
    for(int index=0;index<count;++index){
      const double end=boundaries[index],duration=end-cursor;
      if(duration < -1e-12)throw std::runtime_error("native TS20 regressed time");
      const int steps=std::max(1,int(std::ceil(duration/maxstep-1e-12)));
      const double h=duration/steps;
      int zero_count=0;
      if(duration>0)*substep_count+=steps;
      for(int j=0;j<2;++j){
        Core c{p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],g[j],g[2+j],maxstep};
        const auto full=c.transition(h),half=c.transition(h/2);
        const double force=forcing[active*2+j],damping=c.b+c.kd;
        int mode=x[j].v>0?1:x[j].v<0?-1:0;
        for(int sub=0;sub<steps&&h>0;++sub){
          double remaining=h;int restarts=0;
          while(remaining>1e-14){
            if(++restarts>12)throw std::runtime_error("native TS20 zero-event budget exhausted");
            const double drive=force-c.kp*x[j].q-c.offset;
            if(x[j].v==0&&std::abs(drive)<=c.Fs){mode=0;break;}
            const double sign=std::copysign(1.,x[j].v!=0?x[j].v:drive);
            double effort=force-c.offset-sign*c.friction(x[j].v);
            const auto midpoint=Core::step(x[j],remaining==h?half:c.transition(remaining/2),effort);
            effort=force-c.offset-sign*c.friction(midpoint.v);
            State next=Core::step(x[j],remaining==h?full:c.transition(remaining),effort);
            double accepted=remaining;bool zero=false;mode=int(sign);
            if(sign*next.v<0){
              double lo=0,hi=remaining;
              for(int i=0;i<36;++i){const auto mid=(lo+hi)/2;if(sign*Core::step(x[j],c.transition(mid),effort).v>0)lo=mid;else hi=mid;}
              next=Core::step(x[j],c.transition(hi),effort);next.v=0;accepted=hi;zero=true;++zero_count;
            }
            if(track){
              envelope[j]=std::min(envelope[j],next.q);envelope[2+j]=std::max(envelope[2+j],next.q);
              envelope[4+j]=std::min(envelope[4+j],next.v);envelope[6+j]=std::max(envelope[6+j],next.v);
              const double acc0=(effort-c.kp*x[j].q-damping*x[j].v)/c.J;
              const double acc1=(effort-c.kp*next.q-damping*next.v)/c.J;
              if(acc0*acc1<0){
                double lo=0,hi=accepted,mid=0;State at{};
                for(int i=0;i<30;++i){mid=(lo+hi)/2;at=Core::step(x[j],c.transition(mid),effort);const auto a=(effort-c.kp*at.q-damping*at.v)/c.J;if(a*acc0>0)lo=mid;else hi=mid;}
                envelope[4+j]=std::min(envelope[4+j],at.v);envelope[6+j]=std::max(envelope[6+j],at.v);
                event(cursor+sub*h+h-remaining+mid,j,2,at);
              }
              if(zero)event(cursor+sub*h+h-remaining+accepted,j,1,next);
            }
            x[j]=next;remaining-=accepted;
          }
        }
        if(!std::isfinite(x[j].q)||!std::isfinite(x[j].v)||std::abs(x[j].q)>1e6||std::abs(x[j].v)>1e6)
          throw std::runtime_error("native TS20 motion diverged");
        states[index*4+j]=x[j].q;states[index*4+2+j]=x[j].v;modes[index*2+j]=mode;
      }
      zeros[index]=zero_count;cursor=end;
      while(active+1<packets&&knots[active+1]<=end+1e-11)++active;
    }
    return 0;
  } catch(const std::exception &e){std::snprintf(error,error_size,"%s",e.what());return 1;}
}
