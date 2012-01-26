#include <string>
#include <boost/asio.hpp>

#include "minicrusher_base/mc_structs.h"

namespace minicrusher_base {
    class MCBase {
        public:
            MCBase(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

            ~MCBase() {};

            bool sendCommandPacket(command_packet_t& packet);

            void receivePacket(mc_packet_t& mc_packet); //will wait until getting a valid packet then return

            void close() { shutting_down_ = true; };

        private:
            std::string port_;
            uint32_t baud_rate_;
            bool shutting_down_;
            boost::asio::serial_port serial_;

            //Hide copy constructor, default constructor and assignment operator
            MCBase();
            MCBase(const MCBase&);
            void operator=(const MCBase&);
    };
}; //namespace minicrusher_base
