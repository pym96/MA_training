<<<<<<< HEAD
#include <eigen3/Eigen/Dense>
#include<iostream>
=======
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include<iostream>
#include <bit>
#include <sys/cdefs.h>
#include <cstdint>

enum class TEST{
    RED,
    BLUE
};

struct T{
    enum TEST t;
};

void f(const TEST t){
    if(t == TEST::RED){
        std::cout << "Current color is red";
    }
}

struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  float aim_x = 0.0;
  float aim_y = 0.0;
  float aim_z = 0.0;
  uint16_t checksum = 0;
};

struct ReceivePacket_
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  float aim_x = 0.0;
  float aim_y = 0.0;
  float aim_z = 0.0;
  uint16_t checksum = 0;
}__attribute__((packed));
>>>>>>> 21002ca (Add some openvino demo)


int main(){

<<<<<<< HEAD
    Eigen::Matrix<double, 6, 1> X;
    
    X << 0, 0, 0, 0, 0, 0;

    std::cout << X;


=======


    ReceivePacket p;
    std::cout << sizeof(p) << '\n';
    std::cout << sizeof(ReceivePacket_) << '\n';
    
    uint8_t reserved = p.reserved;
    int bitWidth = 0;
    while (reserved > 0) {
        reserved >>= 1;
        bitWidth++;
    }

    int w = 0;
    uint8_t x = p.detect_color;
    while(x > 0){
        x >>= 1;
        w++;
    }


    std::cout << sizeof(p.aim_x) << '\t' 
                                 << sizeof(p.aim_y)
                                 <<'\t'
                                 << sizeof(p.aim_z)
                                 <<'\t'
                                 << sizeof(p.checksum)
                                 <<'\t'
                                 << sizeof(p.pitch)
                                 << '\t'
                                 << sizeof(p.header)
                                 << '\t'
                                 << bitWidth
                                 << '\t'
                                 << w;

    
>>>>>>> 21002ca (Add some openvino demo)
    return 0;
}