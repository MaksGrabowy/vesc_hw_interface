#include "vesc_hw_interface/vesc.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>      
#include <net/if.h>         
#include <arpa/inet.h>
#include <iomanip>
#include <sstream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <iomanip>

#include <chrono>
#include <thread>
#include <vector>
#include <math.h>

int16_t get_int16(const uint8_t* buf) { return (int16_t)((buf[0] << 8) | buf[1]); }
int32_t get_int32(const uint8_t* buf) { return (int32_t)((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]); }

vesc::vesc()
{}

void vesc::setup(std::vector<uint8_t> can_ids, const std::string& ifname){
    drivers = can_ids;
    std::cout << "In vesc setup, can_ids length = "<< can_ids.size() <<std::endl;
    current_frames.resize(drivers.size());
    states_.resize(drivers.size());

    interface = ifname;
    // Open socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        throw std::runtime_error("Error opening socket: " + std::string(strerror(errno)));
    }

    // // Enable CAN FD
    // int enable_canfd = 1;
    // if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0) {
    //     close(sock);
    //     throw std::runtime_error("Error enabling CAN FD: " + std::string(strerror(errno)));
    // }

    // Locate the interface
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        close(sock);
        throw std::runtime_error("Error getting interface index: " + std::string(strerror(errno)));
    }

    // Bind
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        throw std::runtime_error("Error binding socket: " + std::string(strerror(errno)));
    }

    // Start threads
    rxThread = std::thread(&vesc::receiveLoop, this);
    txThread = std::thread(&vesc::sendLoop, this);
    // queryThread = std::thread(&vesc::queryLoop, this);

    // let's reset the driver once everything is up
    write_stop();
    // write_brake();
}

vesc::~vesc(){
    running = false;
    if (rxThread.joinable()) rxThread.join();
    if (txThread.joinable()) txThread.join();
    if (sock >= 0) close(sock);
}

void vesc::deactivate(){
    running = false;
    if (rxThread.joinable()) rxThread.join();
    if (txThread.joinable()) txThread.join();
    if (sock >= 0) close(sock);
}


void vesc::write_velocity(std::vector<double> velocities){
    std::vector<can_frame> drivers_frames;
    drivers_frames.resize(drivers.size());
    for (auto i = 0u; i < drivers.size(); i++){
        struct can_frame frame{};
        frame.can_id  = 0x00|drivers[i]|(0x03<<8)|1<<31; 
        frame.len     = 4;       // Data length

        int vel = (int)(velocities[i]*9.55*10);

        int32_t rpm_net = htonl(vel);
        memcpy(frame.data, &rpm_net, 4);

        drivers_frames[i] = frame;
    }
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    current_frames = drivers_frames;
    resend_frame = true;
}


void vesc::write_stop(){
    std::vector<can_frame> drivers_frames;
    drivers_frames.resize(drivers.size());
    for (auto i = 0u; i < drivers.size(); i++){
        struct can_frame frame{};
        frame.can_id  = 0x00|drivers[i]|(0x03<<8)|1<<31;   // CAN ID + extended ID flag. has to be changed to incorporate other IDs than 1
        frame.len     = 4;       // Data length


        for(int i=0;i<4;i++)
	        frame.data[i] = 0; //add padding to make the frame CAN FD compliant

        drivers_frames[i] = frame;
    }
    std::lock_guard<std::mutex> lock(current_frame_mutex);
    current_frames = drivers_frames;
    resend_frame = true;
}


void vesc::receiveLoop() {
    while (running) {
        struct can_frame frame {};
        // std::cout << "dupa" << std::endl;
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
            continue;
        }
        if (nbytes == sizeof(struct can_frame)) { /*means we received correct data*/
            interpret_frame(frame);
        }
    }
}

void vesc::sendLoop() {
    int driver_number = 0;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if(driver_number == drivers.size())
            driver_number = 0;
        if(resend_frame){
            std::lock_guard<std::mutex> lock(current_frame_mutex);
            int n = write(sock, &current_frames[driver_number], sizeof(current_frames[driver_number]));
            if (n != sizeof(current_frames[driver_number])) {
                std::cerr << "Failed to send CAN frame "<< std::endl;
            }
            driver_number++;
        }
    }
}

void vesc::interpret_frame(const struct can_frame& frame){
    // std::cout << "RX ID=0x" << std::hex << frame.can_id
    //             << " LEN=" << std::dec << (int)frame.len << " Data=";
    // for (int i = 0; i < frame.len; i++) {
    //     std::cout << std::hex << (int)frame.data[i] << " ";
    // }
    // std::cout << std::dec << std::endl;
    // memcpy(&this->mode,frame.data+2,1);
    uint8_t in_id = (frame.can_id&CAN_EFF_MASK)&0x00FF;
    uint8_t packet_id = ((frame.can_id&CAN_EFF_MASK)&0xFF00)>>8;

    int driver_index = findIndex(drivers,in_id);
    if(driver_index != -1 && packet_id == 9){
        std::lock_guard<std::mutex> lock(current_state_mutex);
        states_[driver_index].velocity = get_int32(frame.data)*0.105f; //conversion from RPM to rad/s
    }
    if(driver_index != -1 && packet_id == 16){
        std::lock_guard<std::mutex> lock(current_state_mutex);
        states_[driver_index].position = ((get_int16(frame.data + 6) / 50.0f)/360.0f)*6.2830; //conversion from degrees to radian
    }
}
std::vector<VescState> vesc::get_state(){
    std::lock_guard<std::mutex> lock(current_state_mutex);
    // MoteusState current_state;
    // current_state.mode = this->mode;
    // current_state.position = this->position;
    // current_state.velocity = this->velocity;
    // current_state.torque = this->torque;
    // current_state.power = this->power;
    // current_state.voltage = this->voltage;
    // current_state.board_temperature = this->temperature;
    // current_state.fault = this->fault;
    // std::vector<MoteusState> current_states = states_;
    return states_;
}

int vesc::findIndex(std::vector<uint8_t>& v, uint8_t val) {
    for (int i = 0; i < v.size(); i++) {
      
      	// When the element is found
        if (v[i] == val) {
            return i;
        }
    }

  	// When the element is not found
  	return -1;
}