#include "control_service.hpp"
#include "persistent_bus.hpp"
#include "motion_control.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sched.h>
#include <sys/mman.h>
#include <net/if.h>
#include <set>
#include <sstream>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
namespace robot_maint {
namespace {
volatile std::sig_atomic_t service_stop=0;
volatile std::sig_atomic_t client_stop=0;
void stop_client(int){client_stop=1;}
void stop_service(int){service_stop=1;}
bool valid_iface(const std::string& iface) {
    if(iface.empty()||iface.size()>15)return false;
    for(unsigned char c:iface)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.'||c==':'))return false;
    return true;
}
std::string directory(){return "/tmp/arm_maint-control-"+std::to_string(::geteuid());}
std::string socket_path(const std::string& iface){return directory()+"/"+iface+".sock";}
int connect_service(const std::string& iface) {
    if(!valid_iface(iface)){errno=EINVAL;return -1;}
    const int fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(fd<0)return -1;
    sockaddr_un addr{};addr.sun_family=AF_UNIX;const auto path=socket_path(iface);
    std::memcpy(addr.sun_path,path.c_str(),path.size()+1);
    if(::connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))!=0){int err=errno;::close(fd);errno=err;return -1;}
    return fd;
}
std::vector<std::string> words(const std::string& line){std::istringstream in(line);std::vector<std::string> out;std::string s;while(in>>s)out.push_back(s);return out;}
std::string encode(const ControlOptions& o) {
    std::ostringstream out;out<<o.command;
    if(o.all)out<<" --all";
    for(const auto& axis:o.axes)out<<" --axis "<<axis;
    if(o.command=="jog-enabled") {
        out<<(o.degrees?" --delta_deg ":" --delta_count ")<<std::setprecision(17);
        for(std::size_t i=0;i<o.deltas.size();++i){if(i)out<<',';out<<o.deltas[i];}
        out<<" --speed_rpm "<<o.rpm;
    }
    if(o.command=="move-zero")out<<" --speed_rpm "<<std::setprecision(17)<<o.rpm;
    return out.str();
}
void reply(int fd,bool ok,const std::string& message) {
    const std::string payload=(ok?"OK\n":"ERR\n")+message+"\n";
    (void)::send(fd,payload.data(),payload.size(),MSG_NOSIGNAL|MSG_DONTWAIT);
}
bool runtime_directory(std::string& error) {
    const auto path=directory();
    if(::mkdir(path.c_str(),0700)!=0&&errno!=EEXIST){error="无法创建 PDO 服务目录";return false;}
    struct stat st{};
    if(::lstat(path.c_str(),&st)||!S_ISDIR(st.st_mode)||st.st_uid!=::geteuid()||(st.st_mode&077)!=0){error="PDO 服务目录所有者或权限异常";return false;}
    return true;
}
struct Client {
    int fd{-1};std::string input;bool pending{false},shutdown{false};MotionTicket ticket;
    std::chrono::steady_clock::time_point connected{std::chrono::steady_clock::now()};
};
int service(const std::string& iface,int lock_fd,int ready_fd,const std::vector<std::string>& initial_axes) {
    (void)lock_fd; // Kept open for the lifetime of this sole bus owner.
    service_stop=0;std::signal(SIGINT,stop_service);std::signal(SIGTERM,stop_service);std::signal(SIGHUP,SIG_IGN);
    std::signal(SIGPIPE,SIG_IGN);::prctl(PR_SET_NAME,"arm_maint_pdo",0,0,0);
    auto startup_error=[&](const std::string& message){
        std::cerr<<"[启动失败] "<<message<<'\n';
        const auto s="ERR "+message+"\n";
        if(::write(ready_fd,s.data(),s.size())!=static_cast<ssize_t>(s.size()))
            std::cerr<<"无法发送启动错误通知\n";
    };
    PersistentPdoBus bus(iface);std::string error;
    if(!bus.prepare(initial_axes,error)){startup_error(error);return 1;}
    MotionControl control(bus.info());std::vector<MotionFeedback> feedback;
    if(!bus.cycle(control.outputs(),feedback)){startup_error("初始 PDO 反馈无效");return 1;}
    control.update(feedback,true);
    std::cerr<<"[启动] 在失能状态下连续验证 PDO 通信 1 秒\n";
    auto warmup_next=std::chrono::steady_clock::now();
    for(int n=0;n<500;++n) {
        if(service_stop){startup_error("启动验证被中断");return 1;}
        if(!bus.cycle(control.outputs(),feedback)) {
            startup_error("失能状态 PDO 验证失败，WKC="+std::to_string(bus.last_wkc())
                          +" expected="+std::to_string(bus.expected_wkc())+"；"+bus.diagnostics());return 1;
        }
        control.update(feedback,true);
        warmup_next+=std::chrono::milliseconds(2);std::this_thread::sleep_until(warmup_next);
    }
    const int listener=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);
    if(listener<0){startup_error("创建控制套接字失败");return 1;}
    sockaddr_un address{};address.sun_family=AF_UNIX;const auto path=socket_path(iface);
    std::memcpy(address.sun_path,path.c_str(),path.size()+1);
    struct stat st{};
    if(::lstat(path.c_str(),&st)==0){if(!S_ISSOCK(st.st_mode)||st.st_uid!=::geteuid()){startup_error("套接字路径异常");::close(listener);return 1;}::unlink(path.c_str());}
    if(::bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))||::listen(listener,8)) {
        startup_error("绑定控制套接字失败");::close(listener);return 1;
    }
    ::chmod(path.c_str(),0600);
    // Keep cyclic traffic responsive to normal desktop workloads. Failure to
    // obtain these privileges is visible; communication protections stay active.
    if(::mlockall(MCL_CURRENT)!=0)
        std::cerr<<"[调度] 锁定当前内存失败: "<<std::strerror(errno)<<'\n';
    else std::cerr<<"[调度] 已锁定当前内存\n";
    sched_param scheduling{};scheduling.sched_priority=10;
    if(::sched_setscheduler(0,SCHED_FIFO,&scheduling)!=0)
        std::cerr<<"[调度] 设置 SCHED_FIFO/10 失败: "<<std::strerror(errno)<<'\n';
    else std::cerr<<"[调度] 使用 SCHED_FIFO/10\n";
    const std::string ready="READY "+std::to_string(::getpid())+"\n";
    const bool notified=::write(ready_fd,ready.data(),ready.size())==static_cast<ssize_t>(ready.size());
    ::close(ready_fd);
    if(!notified){::close(listener);::unlink(path.c_str());return 1;}
    std::vector<Client> clients;clients.reserve(16);auto next=std::chrono::steady_clock::now();bool finish=false;
    int final_reply_fd=-1;std::string final_reply;
    const auto running_since=std::chrono::steady_clock::now();
    std::uint64_t cycles=0;std::int64_t max_cycle_us=0,max_late_us=0;
    while(!service_stop&&!finish) {
        const auto now=std::chrono::steady_clock::now();
        const auto late_us=std::chrono::duration_cast<std::chrono::microseconds>(now-next).count();
        const bool late=late_us>20000&&control.any_active();
        const bool healthy=bus.cycle(control.outputs(),feedback);
        ++cycles;max_cycle_us=std::max(max_cycle_us,bus.last_cycle_us());
        max_late_us=std::max(max_late_us,static_cast<std::int64_t>(late_us));
        if(!healthy||late) {
            const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(now-running_since).count();
            std::cerr<<"[异常停止] PID="<<::getpid()<<" elapsed_ms="<<elapsed<<" cycle="<<cycles
                     <<" "<<bus.diagnostics()
                     <<" exchange_us="<<bus.last_cycle_us()<<" late_us="<<late_us
                     <<" max_exchange_us="<<max_cycle_us<<" max_late_us="<<max_late_us
                     <<" reason="<<(!healthy?"WKC不足":"周期延迟超过20ms")<<'\n';
        }
        control.update(feedback,healthy);
        if(late)control.stop_all("PDO 周期延迟超过 20ms，停止全部目标");
        for(unsigned n=0;n<8;++n) {
            int fd=::accept4(listener,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);if(fd<0)break;
            if(clients.size()>=16) {
                control.stop_all("控制连接数超限，停止全部目标");
                reply(fd,false,"控制连接数超限，服务正在失能退出");::close(fd);finish=true;break;
            }
            Client client;client.fd=fd;clients.push_back(std::move(client));
        }
        for(auto& client:clients) {
            if(finish){reply(client.fd,false,"PDO 服务正在失能关闭");::close(client.fd);client.fd=-1;continue;}
            char data[512];const auto got=::recv(client.fd,data,sizeof(data),MSG_DONTWAIT);
            if(got==0||(got<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)) {
                if(client.pending)control.abort(client.ticket,"请求客户端断开，已取消并失能目标");
                ::close(client.fd);client.fd=-1;continue;
            }
            if(got>0) {
                if(client.pending){reply(client.fd,false,"一个连接只允许一条命令");control.abort(client.ticket,"客户端协议异常");::close(client.fd);client.fd=-1;continue;}
                client.input.append(data,static_cast<std::size_t>(got));
                if(client.input.size()>4096) {
                    reply(client.fd,false,"控制请求超过 4096 字节");
                    ::close(client.fd);client.fd=-1;continue;
                }
            }
            if(!client.pending&&client.input.find('\n')!=std::string::npos) {
                const auto line=client.input.substr(0,client.input.find('\n'));
                if(line=="status"||line=="read-pos") {reply(client.fd,true,"PDO 服务 PID="+std::to_string(::getpid())+"\n"+bus.diagnostics()+"\n"+control.status());::close(client.fd);client.fd=-1;continue;}
                if(line.rfind("read-pos ",0)==0) {
                    std::vector<std::string> labels;std::string label;
                    std::istringstream list(line.substr(9));
                    while(std::getline(list,label,','))labels.push_back(label);
                    std::vector<int> ids;
                    const bool ok=resolve_motion_axes(labels,control.size(),ids,error);
                    reply(client.fd,ok,ok?control.status(ids):error);
                    ::close(client.fd);client.fd=-1;continue;
                }
                ControlOptions options;std::vector<int> ids;
                bool accepted=parse_control_options(words(line),options,error);
                if(accepted&&options.command=="start-pdo") {
                    int available=0;
                    for(const auto& axis:control.info())if(axis.unavailable.empty())++available;
                    reply(client.fd,true,"[PDO 通信已建立] 可控制伺服="+std::to_string(available)
                          +"；本命令不执行使能\n"+bus.diagnostics()+"\n"+control.status());
                    ::close(client.fd);client.fd=-1;continue;
                }
                if(accepted&&options.command=="stop-pdo") {
                    for(int i=0;i<control.size();++i)if(control.info()[i].unavailable.empty())ids.push_back(i);
                } else if(accepted&&options.all) {
                    for(int i=0;i<control.size();++i)if(control.info()[i].unavailable.empty())ids.push_back(i);
                } else if(accepted)accepted=resolve_motion_axes(options.axes,control.size(),ids,error);
                if(accepted) {
                    if(options.command=="enable")accepted=control.enable(ids,client.ticket,error);
                    else if(options.command=="jog-enabled")accepted=control.jog(ids,options.degrees,options.deltas,options.rpm,client.ticket,error);
                    else if(options.command=="move-zero")accepted=control.move_zero(ids,options.rpm,client.ticket,error);
                    else if(options.command=="disable"||options.command=="stop-pdo")accepted=control.disable(ids,client.ticket,error);
                    else {accepted=false;error="不支持的控制命令";}
                }
                if(!accepted){reply(client.fd,false,error);::close(client.fd);client.fd=-1;continue;}
                client.pending=true;client.shutdown=options.command=="stop-pdo"||(options.command=="disable"&&options.all);
            }
            if(!client.pending&&now-client.connected>std::chrono::seconds(2)) {
                reply(client.fd,false,"请求过长或不完整");::close(client.fd);client.fd=-1;continue;
            }
            if(client.pending) {
                const int result=control.result(client.ticket,error);
                if(result!=0) {
                    if(client.shutdown&&result>0) {
                        // Successful --all replies only after PDO shutdown and
                        // interface-lock release, allowing immediate maintenance.
                        final_reply_fd=client.fd;final_reply=error+(client.shutdown?"；[PDO 通信已停止] 网口已释放":"");finish=true;
                    } else {reply(client.fd,result>0,error);::close(client.fd);}
                    client.fd=-1;
                }
            }
        }
        clients.erase(std::remove_if(clients.begin(),clients.end(),[](const Client& c){return c.fd<0;}),clients.end());
        if(!healthy||late) {control.stop_all("PDO 通信/周期异常，服务正在失能退出");finish=true;}
        next+=std::chrono::milliseconds(2);
        if(std::chrono::steady_clock::now()>next+std::chrono::milliseconds(20))next=std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next);
    }
    std::cerr<<"[服务退出] PID="<<::getpid()<<" cycles="<<cycles<<" signal="<<service_stop
             <<" max_exchange_us="<<max_cycle_us<<" max_late_us="<<max_late_us<<'\n';
    control.stop_all("PDO 服务停止");
    for(auto& client:clients){reply(client.fd,false,"PDO 服务停止，目标轴正在失能");::close(client.fd);}
    ::close(listener);::unlink(path.c_str());
    bus.shutdown();
    ::close(lock_fd);
    if(final_reply_fd>=0){reply(final_reply_fd,true,final_reply);::close(final_reply_fd);}
    return 0;
}
bool launch(const ControlOptions& initial,InterfaceLock& lock,std::string& error) {
    const auto& iface=initial.iface;
    if(!runtime_directory(error)||!lock.acquire(iface,error))return false;
    int pipefd[2];if(::pipe2(pipefd,O_CLOEXEC)){error="创建启动通知管道失败";return false;}
    const auto pid=::fork();
    if(pid<0){::close(pipefd[0]);::close(pipefd[1]);error="创建后台 PDO 进程失败";return false;}
    if(pid==0) {
        ::close(pipefd[0]);::setsid();
        const int nullfd=::open("/dev/null",O_RDONLY);if(nullfd>=0){::dup2(nullfd,STDIN_FILENO);::close(nullfd);}
        const auto log=directory()+"/"+iface+".log";
        const int fd=::open(log.c_str(),O_WRONLY|O_CREAT|O_APPEND|O_NOFOLLOW,0600);
        if(fd>=0){::dup2(fd,STDOUT_FILENO);::dup2(fd,STDERR_FILENO);::close(fd);}
        // _exit bypasses iostream destruction; flush every startup diagnostic.
        std::cout<<std::unitbuf;std::cerr<<std::unitbuf;
        std::cerr<<"[启动] PDO 服务 PID="<<::getpid()<<" 网口="<<iface<<'\n';
        int result=1;
        try { result=service(iface,lock.fd(),pipefd[1],initial.axes); }
        catch(const std::exception& e){
            const std::string message=std::string("ERR PDO 服务异常: ")+e.what()+"\n";
            std::cerr<<message;
            if(::write(pipefd[1],message.data(),message.size())!=static_cast<ssize_t>(message.size()))
                std::cerr<<"无法发送启动异常通知\n";
        }
        ::_exit(result);
    }
    ::close(pipefd[1]);
    // Large buses perform many sequential mailbox reads before the listener exists.
    // Use a deadline and retry EINTR rather than treating an interrupted poll as timeout.
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(120);
    std::string notification;bool timed_out=false;bool read_failed=false;
    while(notification.find('\n')==std::string::npos&&notification.size()<4096) {
        const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
        if(remaining<=0){timed_out=true;break;}
        pollfd p{pipefd[0],POLLIN,0};
        const int ready=::poll(&p,1,static_cast<int>(remaining));
        if(ready<0){if(errno==EINTR)continue;read_failed=true;break;}
        if(ready==0){timed_out=true;break;}
        char buffer[512];const auto n=::read(pipefd[0],buffer,sizeof(buffer));
        if(n<0){if(errno==EINTR)continue;read_failed=true;break;}
        if(n==0)break;
        notification.append(buffer,static_cast<std::size_t>(n));
    }
    ::close(pipefd[0]);
    if(notification.rfind("READY ",0)!=0||notification.find('\n')==std::string::npos) {
        if(notification.rfind("ERR ",0)==0)error=notification.substr(4);
        else if(timed_out)error="PDO 服务启动超过 120 秒";
        else if(read_failed)error="读取 PDO 服务启动通知失败";
        else error="PDO 服务在就绪前退出，未收到完整启动通知";
        // EOF can precede the child becoming waitable by a few instructions.
        // Reap before signalling, preserving the actual crash/exit status.
        bool reaped=false;
        for(int attempt=0;attempt<20;++attempt) {
            int status=0;const auto waited=::waitpid(pid,&status,WNOHANG);
            if(waited==pid) {
                reaped=true;
                if(WIFSIGNALED(status))
                    error+="；子进程被信号 "+std::to_string(WTERMSIG(status))+" ("+::strsignal(WTERMSIG(status))+") 终止";
                else if(WIFEXITED(status))error+="；子进程退出码="+std::to_string(WEXITSTATUS(status));
                break;
            }
            if(waited<0){if(errno==EINTR)continue;if(errno==ECHILD)reaped=true;break;}
            if(timed_out||read_failed)break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        error+="；后台日志: "+directory()+"/"+iface+".log";
        // An ERR notification means the child is unwinding and performing its
        // Shutdown sequence. Let it release the bus instead of interrupting it.
        if(!reaped&&notification.rfind("ERR ",0)!=0) {
            ::kill(pid,SIGTERM);(void)::waitpid(pid,nullptr,WNOHANG);
        }
        return false;
    }
    return true;
}
}
InterfaceLock::~InterfaceLock(){if(fd_>=0)::close(fd_);}
bool InterfaceLock::acquire(const std::string& iface,std::string& error) {
    if(!valid_iface(iface)){error="无效网卡名称";return false;}
    const unsigned index=::if_nametoindex(iface.c_str());
    if(index==0){error="网卡不存在: "+iface;return false;}
    const auto path="/tmp/arm_maint-ifindex-"+std::to_string(index)+".lock";
    fd_=::open(path.c_str(),O_CREAT|O_RDONLY|O_CLOEXEC|O_NOFOLLOW,0644);
    struct stat st{};
    if(fd_<0||::fstat(fd_,&st)||!S_ISREG(st.st_mode)||::flock(fd_,LOCK_EX|LOCK_NB)) {
        if(fd_>=0){::close(fd_);fd_=-1;}
        error="网口 "+iface+" 正被其他维护/PDO 进程占用；先执行 disable --all 释放服务";return false;
    }
    return true;
}
int control_request(const std::string& iface,const std::string& request,std::string& response,int timeout_ms) {
    const int fd=connect_service(iface);
    if(fd<0){if(errno==ENOENT||errno==ECONNREFUSED)return 0;response="无法连接 PDO 服务: "+std::string(std::strerror(errno));return -1;}
    const auto message=request+"\n";
    if(::send(fd,message.data(),message.size(),MSG_NOSIGNAL)!=static_cast<ssize_t>(message.size())) {::close(fd);response="发送控制命令失败";return -1;}
    response.clear();const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    while(!client_stop&&std::chrono::steady_clock::now()<end) {
        pollfd p{fd,POLLIN,0};const auto ready=::poll(&p,1,100);
        if(ready<0){if(errno==EINTR)continue;break;}if(!ready)continue;
        char bytes[1024];const auto n=::recv(fd,bytes,sizeof(bytes),0);
        if(n==0) {::close(fd);const bool ok=response.rfind("OK\n",0)==0;
            if(response.rfind("ERR\n",0)==0)response.erase(0,4);else if(ok)response.erase(0,3);
            else response="PDO 服务连接中断，命令结果未确认";
            return ok?1:-1;}
        if(n<0){if(errno==EINTR)continue;break;}
        response.append(bytes,static_cast<std::size_t>(n));if(response.size()>16384)break;
    }
    ::close(fd);response=client_stop ? "用户中断请求，后台将取消并失能本次目标轴" : "等待 PDO 服务响应超时或连接失败，命令未确认";return -1;
}
bool parse_control_options(const std::vector<std::string>& args,ControlOptions& o,std::string& error) {
    o={};bool speed=false;
    try {
        for(std::size_t i=0;i<args.size();++i) {
            const auto& arg=args[i];auto value=[&](){if(++i>=args.size())throw std::runtime_error(arg+" 缺少参数");return args[i];};
            if(arg=="enable"||arg=="disable"||arg=="jog-enabled"||arg=="move-zero"||arg=="start-pdo"||arg=="stop-pdo") {if(!o.command.empty())throw std::runtime_error("重复命令");o.command=arg;}
            else if(arg=="-i"||arg=="--iface")o.iface=value();
            else if(arg=="--axis") {
                const auto text=value();
                if(text.empty())throw std::runtime_error("空目标轴");
                std::istringstream list(text);std::string axis;
                while(std::getline(list,axis,',')){if(axis.empty())throw std::runtime_error("空目标轴");o.axes.push_back(axis);}
                if(o.axes.empty()||args[i].back()==',')throw std::runtime_error("空目标轴");
            } else if(arg=="--all") {if(o.all)throw std::runtime_error("重复 --all");o.all=true;}
            else if(arg=="--delta_deg"||arg=="--delta_count"||arg=="--speed_rpm") {
                const auto text=value();
                if(arg=="--speed_rpm") {
                    if(speed)throw std::runtime_error("重复转速");
                    std::size_t used=0;o.rpm=std::stod(text,&used);speed=true;
                    if(used!=text.size())throw std::runtime_error("无效转速: "+text);
                } else {
                    if(o.displacement)throw std::runtime_error("只能指定一种位移且不能重复");
                    if(text.empty()||text.back()==',')throw std::runtime_error("位移列表包含空值");
                    o.degrees=arg=="--delta_deg";o.displacement=true;
                    std::istringstream list(text);std::string item;
                    while(std::getline(list,item,',')) {
                        if(item.empty())throw std::runtime_error("位移列表包含空值");
                        std::size_t used=0;
                        const double delta=o.degrees?std::stod(item,&used):static_cast<double>(std::stoll(item,&used));
                        if(used!=item.size()||!std::isfinite(delta)
                           ||(!o.degrees&&(delta<std::numeric_limits<std::int32_t>::min()||delta>std::numeric_limits<std::int32_t>::max())))
                            throw std::runtime_error("无效位移: "+item);
                        o.deltas.push_back(delta);
                        if(o.deltas.size()>64)throw std::runtime_error("位移列表最多支持 64 个数值");
                    }
                }
            } else throw std::runtime_error("未知参数: "+arg+"；多轴使用 --axis D18,D19 或重复 --axis");
        }
        if(o.command.empty()||!valid_iface(o.iface))throw std::runtime_error("命令或网卡名称无效");
        if(o.command=="start-pdo"||o.command=="stop-pdo") {
            if(!o.axes.empty()||o.all)throw std::runtime_error("start-pdo/stop-pdo 操作整个 PDO 服务，不接受 --axis/--all");
        } else if((o.axes.empty()&&!o.all)||(!o.axes.empty()&&o.all))throw std::runtime_error("必须指定 --axis，且不能与 --all 混用");
        if(o.all&&o.command!="disable")throw std::runtime_error("enable/jog-enabled/move-zero 请显式列出目标轴");
        if(o.command=="jog-enabled") {
            if(!o.displacement||o.deltas.empty()||!std::isfinite(o.rpm)||o.rpm<.05||o.rpm>2
               ||!std::any_of(o.deltas.begin(),o.deltas.end(),[](double value){return value!=0;}))
                throw std::runtime_error("点动要求至少一个非零位移和 0.05..2 RPM 的转速");
            if(o.deltas.size()!=1&&o.deltas.size()!=o.axes.size())
                throw std::runtime_error("位移数量 "+std::to_string(o.deltas.size())+" 与目标轴数量 "+std::to_string(o.axes.size())+" 不一致；指定 1 个值表示所有轴相同，或为每个轴依次指定 1 个值");
        } else if(o.command=="move-zero") {
            if(o.displacement)throw std::runtime_error("move-zero 不接受位移参数，目标固定为编码器绝对位置 0");
            if(!std::isfinite(o.rpm)||o.rpm<.05||o.rpm>2)
                throw std::runtime_error("转速范围 0.05..2 RPM");
        } else if(o.displacement||speed)throw std::runtime_error("此命令不接受运动参数");
        for(const auto& axis:o.axes) {
            std::size_t n=(axis[0]=='D'||axis[0]=='d'||axis[0]=='S'||axis[0]=='s')?1:0;
            if(n==axis.size())throw std::runtime_error("无效目标轴: "+axis);
            const auto start=n;
            for(;n<axis.size();++n)if(axis[n]<'0'||axis[n]>'9')throw std::runtime_error("无效目标轴: "+axis);
            const auto number=std::stoull(axis.substr(start));
            if(number<1||number>64)throw std::runtime_error("目标轴必须在 1..64: "+axis);
        }
    } catch(const std::exception& e){error=e.what();return false;}
    return true;
}
bool handle_control_cli(int argc,char** argv,int& exit_code) {
    std::vector<std::string> args(argv+1,argv+argc);bool relevant=false;
    for(const auto& a:args){if(a=="--help"||a=="-h")return false;relevant|=a=="enable"||a=="disable"||a=="jog-enabled"||a=="move-zero"||a=="start-pdo"||a=="stop-pdo";}
    if(!relevant)return false;
    ControlOptions options;std::string response;
    if(!parse_control_options(args,options,response)){std::cerr<<"[错误] "<<response<<'\n';exit_code=1;return true;}
    client_stop=0;std::signal(SIGINT,stop_client);std::signal(SIGTERM,stop_client);
    int result=control_request(options.iface,encode(options),response);
    if(result==0&&options.command=="start-pdo") {
        {
            InterfaceLock lock;
            if(!launch(options,lock,response)){std::cerr<<"[错误] "<<response<<'\n';exit_code=1;return true;}
        } // Child now owns the only copy of the interface lock.
        result=control_request(options.iface,encode(options),response);
    }
    if(result==0&&options.command=="disable") {
        InterfaceLock lock;MaintEngine engine(options.iface);std::vector<int> ids;
        bool ok=lock.acquire(options.iface,response)&&engine.init();
        if(ok&&options.all)for(int i=0;i<engine.slave_count();++i)ids.push_back(i);
        else if(ok)ok=resolve_motion_axes(options.axes,engine.slave_count(),ids,response);
        std::string combined;
        if(ok)for(int i:ids){std::string detail;const bool disabled=engine.disable_axis(i+1,detail);ok=disabled&&ok;combined+=slave_to_axis_name(i+1,engine.slave_count())+": "+detail+"\n";}
        if(!combined.empty())response=combined;
        result=ok?1:-1;
    }
    if(result==0&&options.command=="start-pdo") {
        response="PDO 服务在启动后退出，请检查后台日志: "+directory()+"/"+options.iface+".log";result=-1;
    }
    if(result==0&&options.command=="stop-pdo") {
        std::cout<<"[成功] [PDO 通信未运行] 无需停止\n";exit_code=0;return true;
    }
    if(result==0&&options.command=="enable") {
        response="PDO 通信未运行，请先执行 start-pdo -i "+options.iface+"；未下发使能";result=-1;
    }
    if(result==0) {
        response.clear();for(const auto& axis:options.axes){if(!response.empty())response+="；";response+=axis+" 未使能（无运行中的 PDO 使能服务）";}
        response+="；未下发任何运动";result=-1;
    }
    std::cout<<(result>0?"[成功] ":"[失败] ")<<response<<'\n';exit_code=result>0?0:1;return true;
}
}
