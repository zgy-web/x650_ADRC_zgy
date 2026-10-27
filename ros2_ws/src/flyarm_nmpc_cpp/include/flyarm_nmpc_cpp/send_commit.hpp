#pragma once
// Actual writer publication commits the plan. A consumer receipt is deliberately
// absent from this interface: consumers are monitored asynchronously downstream.
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

namespace flyarm_nmpc_cpp {
struct ExpectedSend {
  std::uint64_t sequence{}, published_begin_ns{};
  std::array<double,5> command{};
  std::array<double,3> position{}, velocity{};
};
struct SendReceipt {
  std::uint64_t session{}, epoch{}, sequence{}, dispatch{}, sent_ns{};
  double sim_s{};
  bool stop_zero{};
  std::array<std::uint64_t,2> counter{};
  std::array<double,2> joint_velocity{};
  std::uint8_t coordinate_frame{};
  std::uint16_t type_mask{};
  std::array<double,3> position_ned{}, velocity_ned{}, acceleration_ned{};
  double yaw_ned{};
};
template<class Snapshot> class SendCommit {
 public:
  SendCommit(std::uint64_t session,std::uint64_t epoch):session_(session),epoch_(epoch){}
  void begin(double ref_alt,double yaw) {
    if(begun_||!std::isfinite(ref_alt)||!std::isfinite(yaw))throw std::invalid_argument("Invalid send handoff");
    ref_alt_=ref_alt;yaw_=yaw;begun_=true;
  }
  void stage(ExpectedSend candidate) {
    if(!begun_||pending_||candidate.sequence!=committed_+1||candidate.published_begin_ns==0)
      throw std::logic_error("Invalid pending send sequence");
    pending_=std::move(candidate);
  }
  bool pending()const{return pending_.has_value();}
  std::uint64_t pending_sequence()const{return pending_?pending_->sequence:0;}
  std::uint64_t committed()const{return committed_;}
  std::uint64_t buffered_sequence()const{return buffered_?buffered_->sequence:0;}
  void defer(const Snapshot& snapshot) {
    if(!pending_||buffered_||snapshot.sequence!=pending_->sequence+1)
      throw std::logic_error("Only one next admitted snapshot may precede writer notification");
    buffered_=snapshot;
  }
  bool matches(const SendReceipt& send,std::uint64_t now)const {
    if(send.session!=session_||send.epoch!=epoch_||send.stop_zero||send.sequence<=committed_)return false;
    if(!pending_||send.sequence!=pending_->sequence)throw std::invalid_argument("Writer sequence has no matching pending candidate");
    const auto& p=*pending_;
    if(send.dispatch==0||send.sent_ns<p.published_begin_ns||send.sent_ns>now||now-send.sent_ns>25000000ULL||
       !std::isfinite(send.sim_s)||send.counter[0]==0||send.counter[1]==0||send.coordinate_frame!=1||send.type_mask!=2048||
       send.joint_velocity!=std::array<double,2>{p.command[3],p.command[4]}||
       send.position_ned!=std::array<double,3>{p.position[1],p.position[0],ref_alt_-p.position[2]}||
       send.velocity_ned!=std::array<double,3>{p.velocity[1],p.velocity[0],-p.velocity[2]}||
       send.acceleration_ned!=std::array<double,3>{0.,0.,0.}||send.yaw_ned!=yaw_)
      throw std::invalid_argument("Writer notification is stale or differs from selected PX4/TS20 output");
    return true;
  }
  // Call after the backend transaction commits successfully. Duplicates become
  // harmless only after that commit; an exception must not advance the ledger.
  std::optional<Snapshot> complete() {
    if(!pending_)throw std::logic_error("No pending send to commit");
    committed_=pending_->sequence;pending_.reset();auto next=std::move(buffered_);buffered_.reset();return next;
  }
  void discard() {pending_.reset();buffered_.reset();}
 private:
  std::uint64_t session_{},epoch_{},committed_{};double ref_alt_{},yaw_{};bool begun_{};
  std::optional<ExpectedSend> pending_;std::optional<Snapshot> buffered_;
};
}  // namespace flyarm_nmpc_cpp
