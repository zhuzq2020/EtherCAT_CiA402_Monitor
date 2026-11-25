#include "csp_mode.hpp"
#include <iostream>
#include <stdexcept>

using namespace robot_maint;

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int main() {
    try {
        std::string error;
        int writes = 0, reads = 0, exchanges = 0;
        bool operational = false;
        std::int8_t requested = 0;

        auto write = [&](std::int8_t mode) { ++writes; requested = mode; return true; };
        auto read = [&](std::int8_t& mode) { ++reads; mode = operational ? requested : 0; return true; };
        auto exchange = [&] { ++exchanges; return true; };

        check(!request_csp_mode([](std::int8_t) { return false; }, error), "rejected mode write fails");
        check(error.find("0x6060") != std::string::npos, "write failure identified");
        check(request_csp_mode(write, error) && requested == 8 && reads == 0,
              "PRE-OP requests without premature mode confirmation");

        operational = true;
        check(confirm_csp_mode(read, exchange, error) && reads == 1 && exchanges == 1,
              "mode display confirms after OP and exchange");

        reads = exchanges = 0;
        check(confirm_csp_mode([&](std::int8_t& mode) { mode = ++reads < 3 ? 0 : 8; return true; },
                               exchange, error), "delayed OP display accepted");
        check(reads == 3 && exchanges == 3, "PDO exchange continues between mailbox reads");

        reads = exchanges = 0;
        check(!confirm_csp_mode([&](std::int8_t& mode) { ++reads; mode = 0; return true; },
                                exchange, error), "OP display remaining zero fails");
        check(reads == 10 && exchanges == 10 && error.find("最后有效模式=0") != std::string::npos,
              "bounded mismatch reports actual mode");

        reads = exchanges = 0;
        check(!confirm_csp_mode([&](std::int8_t&) { ++reads; return false; },
                                exchange, error), "unreadable display never assumed CSP");
        check(reads == 10 && exchanges == 10 && error.find("读取 0x6061:00 失败") != std::string::npos,
              "read failure identified");

        reads = 0;
        check(!confirm_csp_mode(read, [] { return false; }, error) && reads == 0,
              "communication loss aborts confirmation");

        std::cout << "PRE-OP request, OP mode activation, and continued PDO exchange checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
