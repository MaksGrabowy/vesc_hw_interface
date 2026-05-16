#ifndef INC_VESC_H_
#define INC_VESC_H_

#include <iostream>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>      
#include <net/if.h>         
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>

#include "state.h"

#include <functional>
#include <string>
#include <thread>
#include <vector>


class vesc {
public:
    using Callback = std::function<void(const struct canfd_frame&)>;

    vesc();
    ~vesc();

    void setup(std::vector<uint8_t> can_ids, const std::string& ifname);
    void deactivate();
    void send_standard_query(int driver_number);
    void write_velocity(float velocity);
    void write_velocity(std::vector<double> velocities);
    void write_stop();
    void write_brake();

    // MoteusState get_state();
    std::vector<VescState> get_state();

    double commanded_velocity;
    VescState state_;
    std::vector<VescState> states_;

    int position;
    int velocity;
private:
    u_int8_t can_ID;
    std::vector<uint8_t> drivers;
    std::string interface;
    
    int sock{-1};
    bool running{true};
    bool resend_frame{false};

    std::thread rxThread;
    std::thread txThread;
    std::thread queryThread;

    Callback callback;

    can_frame current_frame;
    std::vector<can_frame> current_frames;
    std::mutex current_frame_mutex;

    void receiveLoop();
    void sendLoop();
    void queryLoop();

    void interpret_frame(const struct can_frame& frame);

    // driver state
    std::mutex current_state_mutex;


    // nan
    u_int32_t nan = 0x7fc00000;
    u_int32_t zero = 0;

    int findIndex(std::vector<uint8_t>& v, uint8_t val);
};

#endif /* INC_MOTEUS_H_ */