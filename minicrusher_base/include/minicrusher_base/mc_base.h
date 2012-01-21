#include <string>

#include <boost/asio.hpp>

namespace minicrusher_base {
    class MCBase {
        public:
            MCBase(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

            ~MCBase() {};

            void close() { shutting_down_ = true };
        private:
            std::string port_;
            uint32_t baud_rate_;
            bool shutting_down_;
            boost::asio::serial_port serial_;
    };
}; //namespace minicrusher_base
