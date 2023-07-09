#include "ekf/ekf.hpp"

using namespace Eigen;

constexpr int Z_N = 2, X_N = 3;
/**
 * x[0] : y
 * x[1] : x
 * x[2] : k
 */
struct Predict
{
    template <typename T>
    void operator () (const T x0[X_N], T x1[X_N])
    {
        x1[0] = x0[2] * (x0[1] + delta_x) * (x0[1] + delta_x);
        x1[1] = x0[1] + delta_x;
        x1[2] = x0[2];
    }

    double delta_x = 1;
}predict;

struct Measure
{
    template <typename T>
    void operator () (const T x0[X_N], T z0[Z_N])
    {
        z0[0] = x0[0];
        z0[1] = z0[1];
    }
}measure;

int main()
{
    srand(0);

    // generate data with noise
    const int N = 100;
    const double k = 1;
    Matrix<double, 1, N> noise = Matrix<double, 1, N>::Random();
    Matrix<double, 1, N> data = Matrix<double, 1, N>::LinSpaced(0, k * (N - 1));
    data += noise;
    data = (data.array() * data.array()).matrix();
    std::cout << "a = [" << data << "]" << std::endl;


    // calculate speed
    AdaptiveEKF<X_N, Z_N> ekf;
    ekf.Q <<    1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    ekf.R << 1, 0, 0, 1;
    ekf.Xe << data[0], 0, 0;
    
    std::cout << "b = [0 ";
    for (int i = 1; i < N; i++)
    {
        ekf.predict(predict);
        Matrix<double, Z_N, 1> watch;
        watch << data[i], i;
        auto xe = ekf.update(measure, watch);

        std::cout << 2 * xe[1] * xe[2] << " ";
    }
    std::cout << "]";
    

    return 0;
}