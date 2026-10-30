// Power-disconnected attitude-only bench. MPC PVA encoder is tested offline;
// this executable cannot transmit PVA, arming, or mode-changing commands.
#include "flyarm_mavlink/contract.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace flyarm_mavlink;
volatile sig_atomic_t interrupted=0;
void signal_stop(int){interrupted=1;}
int64_t now_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// These are checked against the JSON profile by the PC packaging tool.
constexpr std::array<std::pair<int,int>,8> streams{{{31,50},{65,20},{436,10},{230,10},{245,2},{1,2},{147,2},{83,20}}};
struct Port {
  int fd=-1;termios original{};bool have_original=false,exclusive=false,restored=false;
  explicit Port(int baud){
    try {
      const char* device="/dev/ttyTHS1";
#ifdef FLYARM_UART_TEST_DEVICE
      device=std::getenv("FLYARM_UART_TEST_PTY");
      if(!device||std::string(device).rfind("/dev/pts/",0)!=0)throw std::runtime_error("PTY test device required");
#endif
      fd=open(device,O_RDWR|O_NOCTTY|O_NONBLOCK);
      if(fd<0)throw std::runtime_error("UART open failed");
      if(flock(fd,LOCK_EX|LOCK_NB)||ioctl(fd,TIOCEXCL))throw std::runtime_error("UART ownership failed");
      exclusive=true;
      if(tcgetattr(fd,&original))throw std::runtime_error("UART attributes unavailable");
      have_original=true;auto a=original;cfmakeraw(&a);
      a.c_cflag=(a.c_cflag&~(CSIZE|PARENB|CSTOPB|CRTSCTS))|CS8|CLOCAL|CREAD;
      const auto speed=baud==921600?B921600:B57600;cfsetispeed(&a,speed);cfsetospeed(&a,speed);
      if(tcsetattr(fd,TCSANOW,&a))throw std::runtime_error("UART attributes rejected");
    }catch(...){close_port();throw;}
  }
  void close_port(){
    if(fd<0)return;
    if(have_original)restored=tcsetattr(fd,TCSANOW,&original)==0;
    if(exclusive)ioctl(fd,TIOCNXCL);
    close(fd);fd=-1;
  }
  ~Port(){close_port();}
};
struct Record {int64_t ns;bool tx;uint16_t length;std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> packet;};
struct Link {
  Port port;Receiver receiver;mavlink_message_t parse_buffer{};mavlink_status_t parse_status{};
  std::vector<Record> records;uint64_t bad_crc_before=0,bad_crc_after=0,rx_bytes=0,tx_bytes=0;
  bool synchronized=false;int ack=-1;int64_t ack_at=0;uint64_t sent_setpoints=0;
  explicit Link(int baud):port(baud){records.reserve(20000);}
  void record(const mavlink_message_t& m,bool tx,int64_t ns){
    if(records.size()>=20000)throw std::runtime_error("bounded capture full");
    Record r{};r.ns=ns;r.tx=tx;r.length=mavlink_msg_to_send_buffer(r.packet.data(),&m);records.push_back(r);
  }
  void receive(int wait_ms){
    pollfd p{port.fd,POLLIN,0};const int r=poll(&p,1,wait_ms);
    if(r<0){if(errno==EINTR)return;throw std::runtime_error("UART poll failed");}
    if(!r)return;
    if(p.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("UART disconnected");
    uint8_t bytes[4096];auto count=read(port.fd,bytes,sizeof(bytes));
    if(count<0){if(errno==EAGAIN||errno==EINTR)return;throw std::runtime_error("UART read failed");}
    rx_bytes+=count;const auto at=now_ns();
    for(ssize_t i=0;i<count;++i){
      mavlink_message_t m{};mavlink_status_t status{};
      const auto framing=mavlink_frame_char_buffer(&parse_buffer,&parse_status,bytes[i],&m,&status);
      if(framing==MAVLINK_FRAMING_BAD_CRC||framing==MAVLINK_FRAMING_BAD_SIGNATURE){
        if(synchronized)++bad_crc_after;else ++bad_crc_before;
      }else if(framing==MAVLINK_FRAMING_OK){
        synchronized=true;record(m,false,at);receiver.receive(m,at);
        if(m.sysid==1&&m.compid==1&&m.msgid==MAVLINK_MSG_ID_COMMAND_ACK){
          mavlink_command_ack_t a{};mavlink_msg_command_ack_decode(&m,&a);
          if(a.command==MAV_CMD_SET_MESSAGE_INTERVAL){ack=a.result;ack_at=at;}
        }
      }
    }
  }
  void send(const mavlink_message_t& m,bool allow_setpoint){
    bool allowed=m.msgid==MAVLINK_MSG_ID_TIMESYNC||m.msgid==MAVLINK_MSG_ID_HEARTBEAT;
    if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG){
      mavlink_command_long_t c{};mavlink_msg_command_long_decode(&m,&c);
      allowed=c.command==MAV_CMD_SET_MESSAGE_INTERVAL&&c.target_system==1&&c.target_component==1&&
        c.param3==0&&c.param4==0&&c.param5==0&&c.param6==0&&c.param7==0;
      bool listed=false;for(auto [id,hz]:streams)if(c.param1==id&&c.param2==1000000.f/hz)listed=true;
      allowed=allowed&&listed;
    }
    if(m.msgid==MAVLINK_MSG_ID_SET_ATTITUDE_TARGET){
      mavlink_set_attitude_target_t a{};mavlink_msg_set_attitude_target_decode(&m,&a);
      allowed=allow_setpoint&&a.target_system==1&&a.target_component==1&&a.type_mask==7&&a.thrust==0;
    }
    if(!allowed)throw std::runtime_error("TX whitelist rejected message");
    int queued=0;if(ioctl(port.fd,TIOCOUTQ,&queued)||queued>128)throw std::runtime_error("UART output backlog");
    std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};const auto length=mavlink_msg_to_send_buffer(bytes.data(),&m);
    const auto at=now_ns();const auto n=write(port.fd,bytes.data(),length);
    if(n!=length)throw std::runtime_error("partial or failed UART write");
    tx_bytes+=n;if(m.msgid==82)++sent_setpoints;record(m,true,at);
  }
  void configure(){
    for(auto [id,hz]:streams){
      mavlink_command_long_t c{};c.target_system=1;c.target_component=1;c.command=MAV_CMD_SET_MESSAGE_INTERVAL;
      c.param1=id;c.param2=1000000.f/hz;mavlink_message_t m{};
      mavlink_msg_command_long_encode(companion_system,companion_component,&m,&c);
      ack=-1;const auto sent=now_ns();send(m,false);
      while(!interrupted&&now_ns()-sent<1500000000LL&&(ack<0||ack_at<sent))receive(10);
      if(ack!=MAV_RESULT_ACCEPTED||ack_at<sent)throw std::runtime_error("stream request not accepted");
    }
  }
  void housekeeping(int64_t now,int64_t& heartbeat,int64_t& sync){
    mavlink_message_t m{};
    if(now>=heartbeat){
      mavlink_heartbeat_t h{};h.type=MAV_TYPE_ONBOARD_CONTROLLER;h.autopilot=MAV_AUTOPILOT_INVALID;
      h.system_status=MAV_STATE_STANDBY;mavlink_msg_heartbeat_encode(companion_system,companion_component,&m,&h);
      send(m,false);heartbeat=now+1000000000LL;
    }
    if(now>=sync){
      mavlink_timesync_t s{};s.ts1=now;s.target_system=1;s.target_component=1;
      mavlink_msg_timesync_encode(companion_system,companion_component,&m,&s);send(m,false);sync=now+100000000;
    }
  }
};
int main(int argc,char** argv){
  int baud=57600,duration=30;bool config=false,power_disconnected=false;Case test=Case::Observe;
  try{
    for(int i=1;i<argc;++i){const std::string a=argv[i];
      if(a=="--power-disconnected")power_disconnected=true;
      else if(a=="--configure-bench-streams")config=true;
      else if(a=="--baud"&&i+1<argc)baud=std::stoi(argv[++i]);
      else if(a=="--seconds"&&i+1<argc)duration=std::stoi(argv[++i]);
      else if(a=="--case"&&i+1<argc){const std::string v=argv[++i];
        if(v=="observe")test=Case::Observe;else if(v=="readback")test=Case::Readback;
        else if(v=="manual_exit")test=Case::ManualExit;else if(v=="stream_stop")test=Case::StreamStop;
        else throw std::invalid_argument("unknown case");
      }else throw std::invalid_argument("unknown argument");
    }
    if((baud!=57600&&baud!=921600)||duration<5||duration>75)throw std::invalid_argument("bounded UART options required");
    if((test!=Case::Observe||config)&&(!power_disconnected||baud!=921600))throw std::invalid_argument("approved disconnected-power profile required");
    signal(SIGINT,signal_stop);signal(SIGTERM,signal_stop);
    Link link(baud);BenchGate gate;gate.test=test;
    std::string error;const auto start=now_ns();bool notified=false;
    int64_t next=start,heartbeat=start,sync=start,last_send=0,max_send_gap=0,late_ticks=0;
    try{
      // Observe means zero TX unless explicit stream-configuration is selected.
      if(config){
        const auto wait_end=now_ns()+4000000000LL;
        while(!link.receiver.px4_identity&&now_ns()<wait_end&&!interrupted)link.receive(20);
        if(!link.receiver.px4_identity||link.receiver.armed)throw std::runtime_error("disarmed PX4 identity required before stream setup");
        link.configure();
      }
      while(!interrupted&&now_ns()-start<int64_t(duration)*1000000000LL){
        link.receive(2);const auto now=now_ns();
        if(now<next)continue;
        if(gate.started&&now-next>50000000)gate.stop("scheduler_stall");
        if(now-next>20000000)++late_ticks;
        next+=((now-next)/20000000+1)*20000000; // Fixed phase, no catch-up bursts.
        if(test!=Case::Observe){
          if(link.bad_crc_after>0)gate.stop("post_sync_crc_error");
          if(gate.tick(link.receiver,now)){
            if(!notified){std::cout<<"{\"event\":\"ready\",\"zero_thrust_stream\":true,\"limit_s\":60}"<<std::endl;notified=true;}
            link.send(zero_thrust_attitude(gate.anchor,link.receiver.attitude_clock.stamp),true);
            if(last_send)max_send_gap=std::max(max_send_gap,now-last_send);
            last_send=now;
          }
          link.housekeeping(now,heartbeat,sync); // Heartbeats never keep Offboard alive.
        }
      }
      if(interrupted)gate.stop("signal_stop");else gate.stop("capture_complete");
    }catch(const std::exception& e){error=e.what();gate.stop("io_or_protocol_error");}
    link.port.close_port();
    std::cout<<"{\"event\":\"result\",\"complete\":true,\"error\":"<<std::quoted(error)
      <<",\"elapsed_s\":"<<double(now_ns()-start)/1e9<<",\"settings_restored\":"<<(link.port.restored?"true":"false")
      <<",\"fd_closed\":true,\"nx_log_files\":[],\"setpoints\":"<<link.sent_setpoints
      <<",\"mpc_called\":false,\"position_setpoints\":0,\"arm_commands\":0,\"mode_commands\":0"
      <<",\"started\":"<<(gate.started?"true":"false")<<",\"seen_offboard\":"<<(gate.seen_offboard?"true":"false")
      <<",\"stop_reason\":"<<std::quoted(gate.reason)<<",\"late_ticks\":"<<late_ticks
      <<",\"max_send_gap_ms\":"<<double(max_send_gap)/1e6<<",\"rx_bytes\":"<<link.rx_bytes
      <<",\"tx_bytes\":"<<link.tx_bytes<<",\"crc_errors_before_sync\":"<<link.bad_crc_before
      <<",\"crc_errors_after_sync\":"<<link.bad_crc_after<<",\"records\":[";
    bool first=true;constexpr char hex[]="0123456789abcdef";
    for(const auto& r:link.records){if(!first)std::cout<<',';first=false;
      std::cout<<"{\"ns\":"<<r.ns<<",\"tx\":"<<(r.tx?"true":"false")<<",\"hex\":\"";
      for(unsigned i=0;i<r.length;++i){const auto b=r.packet[i];std::cout<<hex[b>>4]<<hex[b&15];}std::cout<<"\"}";
    }
    std::cout<<"]}\n";
    return error.empty()&&link.port.restored?0:2;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
