// Copyright 2023 Pan Jia xiang
// Licensed under the MIT License.

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>


using namespace Eigen;
using namespace cv;


// BA by g2o
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;


// BA by gauss-newton
void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose
);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char ** argv){

    if (argc != 5) {
        std::cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2\n";
        return 1;
    }

    Mat img_1 = imread(argv[1],IMREAD_COLOR);
    Mat img_2 = imread(argv[2],IMREAD_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    /**
        Utilizing EPnp to get pose
    */
    #if 1
        Mat r,t;

         // 建立3D点
        Mat d1 = imread(argv[3], IMREAD_COLOR);       // 深度图为16位无符号数，单通道图像
        Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

        std::vector<KeyPoint> keypoints_1, keypoints_2;
        std::vector<DMatch> matches;
        std::vector<Point3f> pts_3d;
        std::vector<Point2f> pts_2d;

        
        for (DMatch m:matches) {
            ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
            if (d == 0)   // bad depth
                continue;
            float dd = d / 5000.0;
            Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
            pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
            pts_2d.push_back(keypoints_2[m.trainIdx].pt);
        }

        // solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec)
        cv::solvePnP(pts_3d, pts_2d, K, Mat(), r, t,false);

        Mat R;
        Rodrigues(r,R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        std::cout << "R = \n" << R << '\n';
        std::cout << "t = \n" << t << '\n';

    #endif 



    return 0;
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose) {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  const int iterations = 10;
  double cost = 0, lastCost = 0;
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();

    cost = 0;
    // compute cost
    for (int i = 0; i < points_3d.size(); i++) {
      Eigen::Vector3d pc = pose * points_3d[i];
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

      Eigen::Vector2d e = points_2d[i] - proj;

      cost += e.squaredNorm();
      Eigen::Matrix<double, 2, 6> J;
      J << -fx * inv_z,
        0,
        fx * pc[0] * inv_z2,
        fx * pc[0] * pc[1] * inv_z2,
        -fx - fx * pc[0] * pc[0] * inv_z2,
        fx * pc[1] * inv_z,
        0,
        -fy * inv_z,
        fy * pc[1] * inv_z2,
        fy + fy * pc[1] * pc[1] * inv_z2,
        -fy * pc[0] * pc[1] * inv_z2,
        -fy * pc[0] * inv_z;

      H += J.transpose() * J;
      b += -J.transpose() * e;
    }

    Vector6d dx;
    dx = H.ldlt().solve(b);

    if (isnan(dx[0])) {
      std::cout << "result is nan!\n";
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good
      std::cout << "cost: " << cost << ", last cost: " << lastCost << '\n';
      break;
    }

    // update your estimation
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;

    std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << '\n';
    if (dx.norm() < 1e-6) {
      // converge
      break;
    }
  }

  std::cout << "pose by g-n: \n" << pose.matrix() << '\n';
}
