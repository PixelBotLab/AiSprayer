#include "follow/health_server.hpp"

#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#include "follow/logger.hpp"

#include <httplib.h>

namespace follow {
namespace {

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string esc(const std::string& s) {
  std::string o;
  o.reserve(s.size() + 8);
  for (const char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        // 控制字符必须转义：last_error 里可能带设备返回的原始字节，一个裸 0x01 就
        // 能让整份 JSON 在客户端解析失败，而故障现象看起来像"服务没响应"。
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o += c;
        }
    }
  }
  return o;
}

std::string num(double v) {
  if (!std::isfinite(v)) {
    return "null";  // JSON 没有 NaN/Infinity。写成 null 让调用方必须判空，而不是拿到 -nan
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

}  // namespace

std::string to_json(const HealthSnapshot& s) {
  std::ostringstream o;
  o << "{"
    << "\"state\":\"" << esc(s.state) << "\""
    << ",\"status\":\"" << esc(s.status) << "\""
    << ",\"estimator\":\"" << esc(s.estimator) << "\""
    << ",\"align\":\"" << esc(s.align) << "\""
    << ",\"dry_run\":" << (s.dry_run ? "true" : "false")
    << ",\"frames\":" << s.frames
    << ",\"fps\":" << num(s.fps)
    << ",\"period_ms\":" << num(s.period_ms)
    << ",\"compute_ms\":" << num(s.compute_ms)
    << ",\"dropouts\":" << s.dropouts
    << ",\"unpaired_framesets\":" << s.unpaired_framesets
    << ",\"bad_frames\":" << s.bad_frames
    << ",\"device_present\":" << (s.device_present ? "true" : "false")
    << ",\"lock_held\":" << (s.lock_held ? "true" : "false")
    << ",\"uptime_ms\":" << s.uptime_ms
    << ",\"correction_mm\":[" << s.correction[0] << "," << s.correction[1] << ","
    << s.correction[2] << "]"
    << ",\"correction_deg\":[" << s.correction[3] << "," << s.correction[4] << ","
    << s.correction[5] << "]"
    << ",\"sigma_t_mm\":" << num(s.sigma_t_mm)
    << ",\"sigma_r_deg\":" << num(s.sigma_r_deg)
    << ",\"inlier_ratio\":" << num(s.inlier_ratio)
    << ",\"gicp_inliers\":" << s.gicp_inliers
    << ",\"cloud_points\":" << s.cloud_points
    << ",\"map_path\":\"" << esc(s.map_path) << "\""
    << ",\"map_hash\":\"" << std::hex << s.map_hash << std::dec << "\""
    << ",\"map_points\":" << s.map_points
    << ",\"map_built_ts_ns\":" << s.map_built_ts_ns
    << ",\"last_error\":\"" << esc(s.last_error) << "\""
    << "}";
  return o.str();
}

struct HealthServer::Impl {
  httplib::Server server;
  std::thread th;
  mutable std::mutex mu;
  HealthSnapshot snap;
  std::atomic<bool> teach_requested{false};
  std::atomic<bool> stopping{false};
  int64_t started_ms = 0;
};

HealthServer::HealthServer() : im_(new Impl()) {}

HealthServer::~HealthServer() { stop(); }

bool HealthServer::start(int port, std::string* err) {
  if (im_->th.joinable()) {
    if (err) {
      *err = "健康服务器已经在跑";
    }
    return false;
  }
  // 覆盖 httplib 的默认 socket option：它在 Linux 上设的是 SO_REUSEPORT，于是**第二个进程
  // 绑同一个端口会"成功"**，两个实例的 /health 各说各话，请求被内核随机分到其中一边 —— 健康
  // 端点最不能有的性质就是"看起来通了，其实读到的是别人"。换成只设 SO_REUSEADDR：重启时能立刻
  // 拿走 TIME_WAIT 的端口，但活着的监听者会被老老实实拒掉。
  im_->server.set_socket_options([](auto sock) {
    const int one = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const void*>(&one), sizeof(one));
  });
  port_ = port;
  Impl* im = im_.get();
  im->started_ms = now_ms();

  im->server.Get("/", [im](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(im->mu);
    res.set_content("follow_node / GET /health / GET /state / POST /teach\nstate=" + im->snap.state +
                        " status=" + im->snap.status + " frames=" + std::to_string(im->snap.frames),
                    "text/plain; charset=utf-8");
  });
  const auto report = [im](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(im->mu);
    HealthSnapshot s = im->snap;
    s.uptime_ms = now_ms() - im->started_ms;
    res.set_content(to_json(s), "application/json");
  };
  im->server.Get("/health", report);
  im->server.Get("/state", report);
  // 503 而不是 200：Python 门面按"能不能连上"判断健康会永远是对的，必须让它按状态判。
  im->server.Get("/ready", [im](const httplib::Request&, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(im->mu);
    const bool ready = im->snap.state == "tracking";
    res.status = ready ? 200 : 503;
    res.set_content(ready ? "ready\n" : ("not ready: " + im->snap.state + "\n"),
                    "text/plain; charset=utf-8");
  });
  im->server.Post("/teach", [im](const httplib::Request&, httplib::Response& res) {
    im->teach_requested.store(true);
    res.set_content("{\"queued\":true}\n", "application/json");
  });
  im->server.set_error_handler([](const httplib::Request& req, httplib::Response& res) {
    if (res.status == 0) {
      res.status = 500;
    }
    res.set_content("follow_node: " + std::to_string(res.status) + " " + req.path + "\n",
                    "text/plain; charset=utf-8");
  });

  // bind 与 listen 分开：bind 失败是同步可知的，"换个端口再试"或"等一会儿再看"都不如
  // 立刻把 EADDRINUSE 报给拉起我的人。
  if (!im->server.bind_to_port("127.0.0.1", port)) {
    if (err) {
      *err = "端口 " + std::to_string(port) + " 绑定失败（被占用？follow 用 18081，18080 属于相机服务）";
    }
    port_ = 0;
    return false;
  }
  im->th = std::thread([im]() {
    im->server.listen_after_bind();
    im->stopping.store(true);
  });
  // 起来一半就退出会留下一个绑了端口但不答应的线程，所以等 is_running 而不是"start 返回即成功"。
  for (int i = 0; i < 100 && !im->server.is_running(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!im->server.is_running()) {
    if (err) {
      *err = "健康服务器线程启动后立刻退出（端口 " + std::to_string(port) + "）";
    }
    stop();
    return false;
  }
  return true;
}

void HealthServer::stop() {
  if (!im_) {
    return;
  }
  im_->server.stop();
  if (im_->th.joinable()) {
    im_->th.join();
  }
}

void HealthServer::update(const HealthSnapshot& s) {
  std::lock_guard<std::mutex> lock(im_->mu);
  im_->snap = s;
}

HealthSnapshot HealthServer::snapshot() const {
  std::lock_guard<std::mutex> lock(im_->mu);
  return im_->snap;
}

bool HealthServer::take_teach_request() {
  return im_->teach_requested.exchange(false);
}

void HealthServer::request_teach() { im_->teach_requested.store(true); }

}  // namespace follow
