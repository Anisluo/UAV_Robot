#ifndef UAV_PROC_REALSENSE_CTRL_SERVER_H
#define UAV_PROC_REALSENSE_CTRL_SERVER_H

#include <functional>
#include <string>
#include <vector>

// JSON-RPC over Unix SOCK_STREAM 控制通道。
// 请求格式（newline-delimited）：
//   {"id":1,"method":"realsense.start","params":{}}\n
// 响应格式：
//   {"id":1,"result":{"ok":true}}\n
//
// Handler 参数：id（回包用）、method、params_json（原始 params JSON 字符串）
// Handler 返回值：result 对象的 JSON 字符串，例如 "{\"ok\":true}"

class CtrlServer {
public:
    using Handler = std::function<std::string(int id,
                                              const std::string &method,
                                              const std::string &params_json)>;

    CtrlServer();
    ~CtrlServer();

    bool start(const std::string &path, Handler handler);
    void stop();
    // 非阻塞处理一轮：accept 新连接 + 读所有已连接客户端的待处理请求
    void poll_once(int timeout_ms);

private:
    struct Client {
        int fd;
        std::string buf;  // 未完成行缓冲
    };

    void accept_new();
    void handle_client(Client &c);
    void send_response(int fd, int id, const std::string &result_json);
    void close_client(Client &c);

    int listen_fd_;
    std::string path_;
    Handler handler_;
    std::vector<Client> clients_;
};

#endif
