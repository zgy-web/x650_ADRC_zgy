// Native numerical core for the existing conditional TS20 upper MPC.
// This library performs no ROS, file, actuator or controller-memory I/O.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "vendor/quadprogpp/QuadProg++.hh"
// Array.hh defines an unscoped compatibility macro. Do not leak it to Eigen.
#ifdef solve
#undef solve
#endif
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>
#include <limits>

using Mat = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using Vec = Eigen::VectorXd;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using V2 = Eigen::Vector2d;
static void require(bool ok,const char* what){if(!ok)throw std::runtime_error(what);}
struct Reader {
 const double* p; int left;
 Mat matrix(int r,int c){require(r>=0&&c>=0&&r*c<=left,"packed input truncated");
  Mat out=Eigen::Map<const Mat>(p,r,c);p+=r*c;left-=r*c;require(out.allFinite(),"nonfinite input");return out;}
 Vec vec(int n){return matrix(n,1);}
 double scalar(){return vec(1)[0];}
};
struct Writer {
 double* p; int left,used=0;
 template<class T> void put(const Eigen::MatrixBase<T>& x){Mat a=x;int n=a.size();require(n<=left,"output capacity");
  std::copy(a.data(),a.data()+n,p);p+=n;left-=n;used+=n;}
};
static M3 skew(V3 v){M3 m;m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m;}
static M3 rotvec(V3 v){double a=v.norm();if(a<1e-8){M3 K=skew(v);return M3::Identity()+K+.5*K*K;}
 return Eigen::AngleAxisd(a,v/a).toRotationMatrix();}
struct Geometry {V3 p,v;M3 R;Eigen::Matrix<double,3,2> J,vq;};
struct Context {double yaw,yr,gravity;M3 E,Ed;};
static std::pair<M3,M3> thrust(V3 a,V3 jerk,double yaw,double yr,double gravity){
 V3 f=a+V3(0,0,gravity);double norm=f.norm();require(norm>=gravity*.25&&f.z()>gravity*.25,"upright thrust domain");
 V3 z=f/norm,zd=(jerk-z*z.dot(jerk))/norm,xc(std::cos(yaw),std::sin(yaw),0);
 V3 xcd=V3(-std::sin(yaw),std::cos(yaw),0)*yr,c=z.cross(xc);double cn=c.norm();require(cn>=.25,"heading singularity");
 V3 y=c/cn,cd=zd.cross(xc)+z.cross(xcd),yd=(cd-y*y.dot(cd))/cn;
 M3 R,Rd;R.col(0)=y.cross(z);R.col(1)=y;R.col(2)=z;
 Rd.col(0)=yd.cross(z)+y.cross(zd);Rd.col(1)=yd;Rd.col(2)=zd;return {R,Rd};
}
static Geometry geometry(V2 q,V2 dq,V3 a,V3 jerk,const Context& c,const Vec& g){
 auto rr=thrust(a,jerk,c.yaw,c.yr,c.gravity);M3 R=rr.first*c.E,Rd=rr.second*c.E+rr.first*c.Ed;
 double t1=g[2]+q[0],t2=g[3]+q.sum();V3 d1(std::cos(t1),0,-std::sin(t1)),d2(std::cos(t2),0,-std::sin(t2));
 V3 t_1(-std::sin(t1),0,-std::cos(t1)),t_2(-std::sin(t2),0,-std::cos(t2));
 V3 r=g.segment<3>(4)+g.segment<3>(7)+g[0]*d1+g[1]*d2;
 Eigen::Matrix<double,3,2> J,K;J.col(0)=g[0]*t_1+g[1]*t_2;J.col(1)=g[1]*t_2;
 K.col(0)=-g[0]*d1*dq[0]-g[1]*d2*dq.sum();K.col(1)=-g[1]*d2*dq.sum();
 return {R*r,R*J*dq+Rd*r,R,R*J,R*K+Rd*J};
}
struct Output {V3 p,v,a;M3 R;Mat px,pu,vx,vu;};
static Output endpoint(const Vec& x,const Vec& u,const Context& c,const Vec& geo,const Vec& response,bool jac){
 double tau=response[4];V3 a(x[10],x[11],(u[2]-x[5])/tau),jerk;
 Mat ax=Mat::Zero(3,12),au=Mat::Zero(3,5),jx=ax,ju=au;
 for(int i=0;i<2;i++){double wn=response[2*i],z=response[2*i+1];jerk[i]=wn*wn*(u[i]-x[3+i])-2*z*wn*a[i];
  ax(i,10+i)=1;jx(i,3+i)=-wn*wn;jx(i,10+i)=-2*z*wn;ju(i,i)=wn*wn;}
 jerk[2]=-a[2]/tau;ax(2,5)=-1/tau;au(2,2)=1/tau;jx(2,5)=1/(tau*tau);ju(2,2)=-1/(tau*tau);
 auto g=geometry(x.segment<2>(6),x.segment<2>(8),a,jerk,c,geo);
 Output out{x.head<3>()+g.p,x.segment<3>(3)+g.v,a,g.R,Mat(),Mat(),Mat(),Mat()};
 if(jac){M3 pa,va;for(int i=0;i<3;i++){V3 d=V3::Zero();d[i]=1e-5;
  auto plus=geometry(x.segment<2>(6),x.segment<2>(8),a+d,jerk,c,geo);
  auto minus=geometry(x.segment<2>(6),x.segment<2>(8),a-d,jerk,c,geo);
  pa.col(i)=(plus.p-minus.p)/(2e-5);va.col(i)=(plus.v-minus.v)/(2e-5);}
  out.px=pa*ax;out.pu=pa*au;out.vx=va*ax+pa*jx;out.vu=va*au+pa*ju;
  out.px.leftCols(3)+=M3::Identity();out.px.middleCols(6,2)+=g.J;
  out.vx.middleCols(3,3)+=M3::Identity();out.vx.middleCols(6,2)+=g.vq;out.vx.middleCols(8,2)+=g.J;}
 return out;
}
static void transition(double h,const Vec& r,Mat& A,Mat& B){
 A=Mat::Identity(12,12);B=Mat::Zero(12,5);
 for(int i=0;i<2;i++){double wn=r[2*i],z=r[2*i+1];Eigen::Matrix4d G;
  G<<0,1,0,0,0,0,1,0,0,-wn*wn,-2*z*wn,wn*wn,0,0,0,0;
  Eigen::Matrix4d T=(h*G).exp();int ix[3]={i,3+i,10+i};
  for(int j=0;j<3;j++){for(int k=0;k<3;k++)A(ix[j],ix[k])=T(j,k);B(ix[j],i)=T(j,3);}}
 double alpha=std::exp(-h/r[4]),integrated=r[4]*(1-alpha);A(2,5)=integrated;A(5,5)=alpha;B(2,2)=h-integrated;B(5,2)=1-alpha;
 A.block<2,2>(8,8).setZero();B.block<2,2>(6,3)=h*Eigen::Matrix2d::Identity();B.block<2,2>(8,3).setIdentity();
}
static std::pair<Vec,Vec> bounds(double dt,const V2& q,const Vec& vl,const Vec& al,const Vec& previous,const V2& qlo,const V2& qhi,double maxdt){
 require((q.array()>=qlo.array()-1e-8).all()&&(q.array()<=qhi.array()+1e-8).all(),"joint soft envelope");
 Vec lo=(-vl).cwiseMax(previous-al*dt),hi=vl.cwiseMin(previous+al*dt);
 for(int j=0;j<2;j++){double a=al[3+j],reserve=a*maxdt;
  lo[3+j]=std::max(lo[3+j],-(std::sqrt(reserve*reserve+2*a*std::max(q[j]-qlo[j],0.))-reserve));
  hi[3+j]=std::min(hi[3+j],std::sqrt(reserve*reserve+2*a*std::max(qhi[j]-q[j],0.))-reserve);}
 require((lo.array()<=hi.array()+1e-10).all(),"inconsistent stopping bounds");
 for(int j=0;j<5;j++)if(lo[j]>hi[j])lo[j]=hi[j]=.5*(lo[j]+hi[j]);return {lo,hi};
}

extern "C" int x650_optimization_abi(){return 1;}
// Frozen sliding-branch sensitivity recurrence. The reference predictor supplies
// its exact cached A/B transitions and already validated packet sensitivities.
// Layout: transition[boundary][joint][state][A0,A1,B], S[joint][state][column].
extern "C" int x650_joint_sensitivity(int n,int nb,int nq,const double* transitions,
 const double* forcing,const int* query_index,double* output,char* error,int es){
 try {
  require(n>0&&n<=300&&nb>0&&nb<=5000&&nq>0&&nq<=nb,
          "joint sensitivity shape");
  require(transitions&&forcing&&query_index&&output,"joint sensitivity pointers");
  Mat S=Mat::Zero(4,n),next(4,n);int query=0;
  for(int b=0;b<nb;b++){
   for(int j=0;j<2;j++){
    const double* a=transitions+b*12+j*6;
    Eigen::Map<const Vec> force(forcing+(b*2+j)*n,n);
    require(force.allFinite(),"nonfinite joint forcing sensitivity");
    for(int k=0;k<6;k++)require(std::isfinite(a[k]),"nonfinite joint transition");
    for(int s=0;s<2;s++)
     next.row(2*j+s)=a[3*s]*S.row(2*j)+a[3*s+1]*S.row(2*j+1)+a[3*s+2]*force.transpose();
   }
   S.swap(next);
   if(query<nq&&query_index[query]==b){
    std::copy(S.data(),S.data()+4*n,output+query*4*n);query++;
   }
  }
  require(query==nq,"joint query coverage/order");return 0;
 }catch(const std::exception& e){std::snprintf(error,es,"%s",e.what());return 1;}
}
extern "C" int x650_assemble(int N,const double* data,int length,double* output,int capacity,int* shape,char* error,int es){
 try {require(N>=2&&N<=60&&data&&output&&shape,"assembly contract");int n=5*N;Reader r{data,length};
  double dt=r.scalar(),node_dt=r.scalar(),maxdt=r.scalar(),age=r.scalar(),trust=r.scalar();
  require(dt>0&&dt<=maxdt+1e-10&&node_dt>0&&age>=0&&age<=.25&&trust>0&&trust<=.02,"time/trust contract");
  Vec w=r.vec(6),vl=r.vec(5),al=r.vec(5),previous=r.vec(5);V2 qlo=r.vec(2),qhi=r.vec(2);
  Vec x0=r.vec(12);M3 R0=r.matrix(3,3);V3 omega=r.vec(3),acceleration=r.vec(3);Vec response=r.vec(5),geo=r.vec(10);
  V3 p_anchor=r.vec(3);M3 R_anchor=r.matrix(3,3);V2 q_anchor=r.vec(2);double share=r.scalar();
  V2 physical_lo=r.vec(2),physical_hi=r.vec(2),measured_dq=r.vec(2),current_q=r.vec(2),current_dq=r.vec(2);
  int nh=int(r.scalar());require(nh>=0&&nh<=64,"history capacity");Mat recent=r.matrix(nh,2);
  Mat warm=r.matrix(N,5),targets=r.matrix(N,6),joints=r.matrix(N,4+4*n),references=r.matrix(N,2+2*n);
  require(r.left==0&&(vl.array()>0).all()&&(al.array()>0).all()&&(response.array()>0).all()&&(w.array()>=0).all(),"assembly input");
  Vec h=Vec::Constant(N,node_dt);h[0]=dt;Vec scale(n);for(int i=0;i<N;i++)scale.segment(i*5,5)=vl;
  Vec zwarm=Eigen::Map<Vec>(warm.data(),n).cwiseQuotient(scale);
  double yaw=std::atan2(R0(1,0),R0(0,0)),roll=std::atan2(R0(2,1),R0(2,2)),pitch=std::asin(std::clamp(-R0(2,0),-1.,1.));
  double yr=(std::sin(roll)*omega[1]+std::cos(roll)*omega[2])/std::cos(pitch),tau=.15,gravity=9.8;
  M3 nominal_R=thrust(acceleration,V3::Zero(),yaw,0,gravity).first;
  Eigen::AngleAxisd aa(nominal_R.transpose()*R0);V3 residual=aa.axis()*aa.angle();
  Vec offset=x0;Mat S=Mat::Zero(12,n),offsets(N,12),jac(N*12,n),lin(N,12),rot(N,9),contexts(N,20);
  Mat Amax=Mat::Zero(N*21,n),Cmax=Mat::Zero(N*15+5,n);Vec bmax(N*21),lo_max(N*15+5),hi_max(N*15+5);int ar=0,cr=0;
  auto cost=[&](const Mat& A,const Vec& b,double weight,double step){if(weight){int k=A.rows();double f=std::sqrt(weight*step);Amax.middleRows(ar,k)=f*A;bmax.segment(ar,k)=f*b;ar+=k;}};
  auto constrain=[&](const Mat& C,const Vec& lo,const Vec& hi){for(int j=0;j<C.rows();j++){
   if((C.row(j).array()==0).all()){require(lo[j]<=0&&hi[j]>=0,"constant joint constraint violated");continue;}
   Cmax.row(cr)=C.row(j);lo_max[cr]=lo[j];hi_max[cr]=hi[j];cr++;}};
  auto body_tip=[&](V2 q){return V3(geo.segment<3>(4)+geo[0]*V3(std::cos(geo[2]+q[0]),0,-std::sin(geo[2]+q[0]))+geo.segment<3>(7)+geo[1]*V3(std::cos(geo[3]+q.sum()),0,-std::sin(geo[3]+q.sum())));};
  V3 tip0=body_tip(q_anchor),tipw0=p_anchor+R_anchor*tip0;double elapsed=0;
  Mat A0,B0,Astep,Bstep;transition(dt,response,A0,B0);transition(node_dt,response,Astep,Bstep);
  for(int i=0;i<N;i++){double step=h[i];elapsed+=step;Mat At=i?Astep:A0,Bt=i?Bstep:B0;
   offset=At*offset;S=At*S;S.middleCols(i*5,5)+=Bt*vl.asDiagonal();
   offset.segment(6,4)=joints.row(i).head(4).transpose();
   S.middleRows(6,4)=Eigen::Map<const Mat>(joints.row(i).data()+4,4,n);
   Vec xlin=offset+S*zwarm;offsets.row(i)=offset.transpose();jac.middleRows(12*i,12)=S;lin.row(i)=xlin.transpose();
   Mat U=Mat::Zero(5,n);U.middleCols(i*5,5)=vl.asDiagonal();Mat D=U;if(i)D.middleCols((i-1)*5,5)-=vl.asDiagonal().toDenseMatrix();
   double decay=std::exp(-elapsed/tau);M3 E=rotvec(decay*residual),Ed=E*skew(-decay*residual/tau);
   Context ctx{yaw+tau*(1-decay)*yr,decay*yr,gravity,E,Ed};
   contexts(i,0)=ctx.yaw;contexts(i,1)=ctx.yr;Eigen::Map<Mat>(contexts.row(i).data()+2,3,3)=E;Eigen::Map<Mat>(contexts.row(i).data()+11,3,3)=Ed;
   auto eo=endpoint(xlin,warm.row(i).transpose(),ctx,geo,response,true);
   Eigen::Map<Mat>(rot.row(i).data(),3,3)=eo.R;Mat P=eo.px*S+eo.pu*U,V=eo.vx*S+eo.vu*U;
   V3 po=eo.p+eo.px*(offset-xlin)-eo.pu*warm.row(i).transpose(),vo=eo.v+eo.vx*(offset-xlin)-eo.vu*warm.row(i).transpose();
   V3 tp=targets.row(i).head(3).transpose(),tv=targets.row(i).tail(3).transpose(),delta=R_anchor.transpose()*(tp-tipw0);delta[1]=0;
   V3 goal=tip0+share*delta-geo.segment<3>(4)-geo.segment<3>(7);require(std::abs(goal[1])<=1e-8,"nominal outside arm plane");
   double cosine=(goal.x()*goal.x()+goal.z()*goal.z()-geo[0]*geo[0]-geo[1]*geo[1])/(2*geo[0]*geo[1]);
   require(std::abs(cosine)<=1+1e-10,"nominal unreachable");V2 qn;double best=std::numeric_limits<double>::infinity();
   for(int sign:{-1,1}){double beta=sign*std::acos(std::clamp(cosine,-1.,1.));double theta=std::atan2(-goal.z(),goal.x())-std::atan2(geo[1]*std::sin(beta),geo[0]+geo[1]*std::cos(beta));
    V2 q(theta-geo[2],beta+geo[2]-geo[3]);for(int j=0;j<2;j++){double v=std::fmod(q[j]-q_anchor[j]+M_PI,2*M_PI);if(v<0)v+=2*M_PI;q[j]=q_anchor[j]+v-M_PI;}
    double dist=(q-q_anchor).norm();if(dist<best){best=dist;qn=q;}}
   require((qn.array()>=physical_lo.array()).all()&&(qn.array()<=physical_hi.array()).all(),"nominal physical joint limit");
   Eigen::Matrix<double,3,2> J;V3 tangent1(-std::sin(geo[2]+qn[0]),0,-std::cos(geo[2]+qn[0])),tangent2(-std::sin(geo[3]+qn.sum()),0,-std::cos(geo[3]+qn.sum()));J.col(0)=geo[0]*tangent1+geo[1]*tangent2;J.col(1)=geo[1]*tangent2;
   Eigen::Matrix2d Jxz;Jxz.row(0)=J.row(0);Jxz.row(1)=J.row(2);require(std::abs(Jxz.determinant())>1e-14,"nominal IK singular");V3 vb=share*R_anchor.transpose()*tv;V2 dqn=Jxz.partialPivLu().solve(V2(vb.x(),vb.z()));
   V3 pn=tp-R_anchor*body_tip(qn),vn=tv-R_anchor*J*dqn;Vec un(5);un<<vn,dqn;
   cost(P,tp-po,w[0],step);cost(V,tv-vo,w[1],step);cost(S.topRows(3),pn-offset.head(3),w[2],step);cost(S.middleRows(6,2),qn-offset.segment(6,2),w[3],step);
   cost(vl.cwiseInverse().asDiagonal()*U,un.cwiseQuotient(vl),w[4],step);Vec prev=i?Vec::Zero(5):previous;
   constrain(D,prev-al*step,prev+al*step);Vec denom=al*step;cost(denom.cwiseInverse().asDiagonal()*D,prev.cwiseQuotient(denom),w[5],step);
  }
  V2 slew=al.tail(2)*dt,vmin=measured_dq.cwiseMin(current_dq).cwiseMin(previous.tail(2)-slew),vmax=measured_dq.cwiseMax(current_dq).cwiseMax(previous.tail(2)+slew);
  for(int j=0;j<nh;j++){vmin=vmin.cwiseMin(recent.row(j).transpose());vmax=vmax.cwiseMax(recent.row(j).transpose());}
  V2 qgl=current_q+(.05+age)*vmin.cwiseMin(V2::Zero()),qgh=current_q+(.05+age)*vmax.cwiseMax(V2::Zero());
  Vec firstlo=bounds(dt,qgl,vl,al,previous,qlo,qhi,maxdt).first,firsthi=bounds(dt,qgh,vl,al,previous,qlo,qhi,maxdt).second;
  require((firstlo.array()<=firsthi.array()+1e-10).all(),"pending guard infeasible");V2 reserve=vl.tail(2).cwiseQuotient(2*al.tail(2))+V2::Constant(maxdt+.05+age);
  for(int i=0;i<N;i++){V2 q=joints.row(i).head(2).transpose(),dq=joints.row(i).segment(2,2).transpose();Mat Q=Eigen::Map<const Mat>(joints.row(i).data()+4,2,n),D=Eigen::Map<const Mat>(joints.row(i).data()+4+2*n,2,n);
   constrain(Q,qlo-q,qhi-q);constrain(D,-vl.tail(2)-dq,vl.tail(2)-dq);constrain(Q+reserve.asDiagonal()*D,qlo-q-reserve.cwiseProduct(dq),qhi-q-reserve.cwiseProduct(dq));}
  for(int i=0;i<N;i++)constrain(Eigen::Map<const Mat>(references.row(i).data()+2,2,n),qlo-references.row(i).head(2).transpose(),qhi-references.row(i).head(2).transpose());
  for(int i=0;i<N;i++)for(int j=3;j<5;j++){Mat I=Mat::Zero(1,n);I(0,i*5+j)=1;constrain(I,Vec::Constant(1,zwarm[i*5+j]-trust),Vec::Constant(1,zwarm[i*5+j]+trust));}
  Mat first=Mat::Zero(5,n);first.leftCols(5)=vl.asDiagonal();constrain(first,firstlo,firsthi);
  Mat A=Amax.topRows(ar),C=Cmax.topRows(cr);Vec b=bmax.head(ar),lo=lo_max.head(cr),hi=hi_max.head(cr);
  Writer out{output,capacity};out.put(A);out.put(b);out.put(2*A.transpose()*A);out.put(-2*A.transpose()*b);out.put(C);out.put(lo);out.put(hi);out.put(offsets);out.put(jac);out.put(lin);out.put(rot);out.put(contexts);out.put(firstlo);out.put(firsthi);out.put(qgl);out.put(qgh);out.put(residual);
  shape[0]=ar;shape[1]=cr;shape[2]=out.used;return 0;
 }catch(const std::exception& e){std::snprintf(error,es,"%s",e.what());return 1;}
}

// Lawson-Hanson passive-set NNLS for the same full-row KKT check used by Python.
static Vec nnls(const Mat& A,const Vec& b){int m=A.cols(),steps=0;Vec x=Vec::Zero(m);std::vector<int> passive;std::vector<bool> selected(m,false);
 double tol=10*std::max(A.rows(),A.cols())*std::numeric_limits<double>::epsilon();
 while(true){Vec w=A.transpose()*(b-A*x);int add=-1;double best=tol;for(int j=0;j<m;j++)if(!selected[j]&&w[j]>best){best=w[j];add=j;}if(add<0)break;
  passive.push_back(add);selected[add]=true;
  while(true){require(++steps<=std::max(10*m,1),"NNLS iteration limit");Mat P(A.rows(),passive.size());for(int j=0;j<int(passive.size());j++)P.col(j)=A.col(passive[j]);Vec s=P.completeOrthogonalDecomposition().solve(b),z=Vec::Zero(m);bool positive=true;
   for(int j=0;j<int(passive.size());j++){z[passive[j]]=s[j];if(s[j]<=0)positive=false;}if(positive){x=z;break;}
   double alpha=1;for(int j:passive)if(z[j]<=0)alpha=std::min(alpha,x[j]/(x[j]-z[j]));x+=alpha*(z-x);
   std::vector<int> keep;for(int j:passive)if(x[j]>tol)keep.push_back(j);else {x[j]=0;selected[j]=false;}passive.swap(keep);
  }}return x;}
static Vec quality(const Mat& H,const Vec& g,const Mat& C,const Vec& lo,const Vec& hi,const Vec& z){
 int n=z.size(),m=C.rows();Vec value=C*z;double primal=std::max({0.,(lo-value).maxCoeff(),(value-hi).maxCoeff(),(z.cwiseAbs().array()-1).maxCoeff()});
 Mat normals(2*m+2*n,n);normals<<C,-C,Mat::Identity(n,n),-Mat::Identity(n,n);Vec limits(2*m+2*n);limits<<hi,-lo,Vec::Ones(2*n);Vec slack=limits-normals*z,grad=H*z+g;
 std::vector<int> active;for(int i=0;i<slack.size();i++)if(slack[i]<1e-6)active.push_back(i);double comp=0;
 if(!active.empty()){Mat K(n,active.size());Vec s(active.size());for(int i=0;i<int(active.size());i++){K.col(i)=normals.row(active[i]).transpose();s[i]=slack[active[i]];}Vec lambda=nnls(K,-grad);grad+=K*lambda;comp=lambda.cwiseProduct(s).cwiseAbs().maxCoeff();}
 Vec q(4);q<<primal,grad.cwiseAbs().maxCoeff(),comp,double(active.size());return q;
}
extern "C" int x650_solve(int n,int m,const double* hp,const double* gp,const double* cp,const double* lp,const double* up,int max_iterations,double* zp,double* quality_out,int* mode,char* error,int es){
 try {require(n>0&&n<=300&&m>0&&m<=5000,"QP shape");Mat H=Eigen::Map<const Mat>(hp,n,n),C=Eigen::Map<const Mat>(cp,m,n);Vec g=Eigen::Map<const Vec>(gp,n),lo=Eigen::Map<const Vec>(lp,m),hi=Eigen::Map<const Vec>(up,m);
  require(H.allFinite()&&C.allFinite()&&g.allFinite()&&lo.allFinite()&&hi.allFinite()&&(lo.array()<=hi.array()).all(),"QP finite/bounds");
  require((H-H.transpose()).cwiseAbs().maxCoeff()<1e-8,"Hessian symmetry");Eigen::LLT<Eigen::MatrixXd> factor(H);require(factor.info()==Eigen::Success,"Hessian not positive definite");Vec center=-factor.solve(g),value=C*center,z=center;*mode=0;
  bool interior=(center.cwiseAbs().array()<=1.-1e-10).all()&&(value.array()>=lo.array()+1e-10).all()&&(value.array()<=hi.array()-1e-10).all();
  if(!interior){*mode=1;Mat T=factor.matrixU().solve(Mat::Identity(n,n)),M(2*m+2*n,n);Vec b(2*m+2*n);
   M<<C*T,-C*T,T,-T;b<<hi-value,value-lo,Vec::Ones(n)-center,Vec::Ones(n)+center;
   // Exact duplicate inequalities are harmless; retain all for quality.
   std::vector<int> keep;for(int i=0;i<M.rows();i++){double norm=M.row(i).norm();if(norm<1e-14){require(b[i]>=0,"constant QP infeasible");continue;}M.row(i)/=norm;b[i]/=norm;keep.push_back(i);}
   quadprogpp::Matrix<double> G(n,n),CE(n,0),CI(n,keep.size());quadprogpp::Vector<double> g0(n),ce0(0),ci0(keep.size()),y(n);
   for(int i=0;i<n;i++){g0[i]=0;for(int j=0;j<n;j++)G[i][j]=i==j?1:0;}
   for(int j=0;j<int(keep.size());j++){ci0[j]=b[keep[j]];for(int i=0;i<n;i++)CI[i][j]=-M(keep[j],i);}
   double objective=quadprogpp::solve_quadprog(G,g0,CE,ce0,CI,ci0,y,max_iterations);require(std::isfinite(objective),"QP infeasible");Vec solution(n);for(int i=0;i<n;i++)solution[i]=y[i];z=center+T*solution;
  }
  require(z.allFinite(),"nonfinite solution");Vec q=quality(H,g,C,lo,hi,z);Eigen::Map<Vec>(zp,n)=z;Eigen::Map<Vec>(quality_out,4)=q;return 0;
 }catch(const std::exception& e){std::snprintf(error,es,"%s",e.what());return 1;}
}
extern "C" int x650_endpoint(int N,const double* xp,const double* up,const double* ctx,const double* geo,const double* response,double* output,char* error,int es){
 try {require(N>0&&N<=60,"endpoint nodes");Mat x=Eigen::Map<const Mat>(xp,N,12),u=Eigen::Map<const Mat>(up,N,5);Vec g=Eigen::Map<const Vec>(geo,10),r=Eigen::Map<const Vec>(response,5);Writer out{output,N*15};
  for(int i=0;i<N;i++){Context c{ctx[20*i],ctx[20*i+1],9.8,Eigen::Map<const Mat>(ctx+20*i+2,3,3),Eigen::Map<const Mat>(ctx+20*i+11,3,3)};auto y=endpoint(x.row(i).transpose(),u.row(i).transpose(),c,g,r,false);out.put(y.p);out.put(y.v);out.put(y.R);}return 0;
 }catch(const std::exception& e){std::snprintf(error,es,"%s",e.what());return 1;}
}
extern "C" int x650_candidate(int N,int nr,const double* predicted,const double* actual_q,const double* actual_dq,const double* refs,const double* envelope,const double* lower,const double* upper,const double* limits,double* result,char* error,int es){
 try {require(N>0&&N<=60&&nr==N,"candidate shape");Mat p=Eigen::Map<const Mat>(predicted,N,12),q=Eigen::Map<const Mat>(actual_q,N,2),dq=Eigen::Map<const Mat>(actual_dq,N,2),reference=Eigen::Map<const Mat>(refs,N,2);
  require(p.allFinite()&&q.allFinite()&&dq.allFinite()&&reference.allFinite(),"candidate finite values");bool bounds=true;for(int j=0;j<2;j++){
   require(std::isfinite(envelope[j])&&std::isfinite(envelope[j+2])&&std::isfinite(envelope[j+4])&&std::isfinite(envelope[j+6]),"candidate finite envelope");
   bounds=bounds&&envelope[j]-.0003>=lower[j]-2e-7&&envelope[j+2]+.0003<=upper[j]+2e-7&&envelope[j+4]-.03>=-limits[j]-2e-7&&envelope[j+6]+.03<=limits[j]+2e-7;
   bounds=bounds&&(reference.col(j).array()>=lower[j]-2e-7).all()&&(reference.col(j).array()<=upper[j]+2e-7).all();}
  double qe=(q-p.middleCols(6,2)).cwiseAbs().maxCoeff(),ve=(dq-p.middleCols(8,2)).cwiseAbs().maxCoeff();result[0]=qe;result[1]=ve;result[2]=bounds?1:0;result[3]=bounds&&qe<=.006&&ve<=.10?1:0;return 0;
 }catch(const std::exception& e){std::snprintf(error,es,"%s",e.what());return 1;}
}
