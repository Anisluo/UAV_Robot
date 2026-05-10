#include <cstdio>

#include "npu_application.h"

int main() {
    // Unbuffer stdout/stderr so model-load + heartbeat lines appear in
    // the systemd journal in real time. Default block buffering on a
    // non-TTY pipe was hiding "[npu_infer] loaded ..." for minutes,
    // making it impossible to tell whether an RKNN model attempt
    // succeeded or fell into the stub path.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    NpuApplication app;
    return app.run();
}
