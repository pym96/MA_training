#include <cmath>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <fmt/color.h>

const double& angular_velocity = 10.0;
const double& linear_velocity = 5.0;
bool use_quaternion = false;

int main(int argc, char** argv){

    double angular_velocity_rad = angular_velocity / 180 * M_PI;

    fmt::print(fg(fmt::color::blue), "Current angular_velocity_rad is {}", angular_velocity_rad);

    return 0;
}