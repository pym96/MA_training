#include <iostream>
#include "mv_camera.hpp"
#include <eigen3/Eigen/Dense>

int main() {

    Devices::MV_Camera c;
    c.open();
    cv::Mat read_img;
    cv::Mat img;
    namedWindow("test", cv::WINDOW_NORMAL);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;


    //相机内参矩阵
    cameraMatrix = (cv::Mat_<double>(3,3) << 1562.718391961961,    4.478471970184, 616.284509563135,
                                                             0, 1563.803986201450, 489.040600564709,
                                                             0,                 0,                1);                 
    
    distCoeffs = (cv::Mat_<double>(5,1) << -0.088250992965441, 0.154271559512706, -0.000829270875611, -0.001394637877056, 0);

    //定义一些变量
    cv::Mat gray,mask,med;  //灰度图矩阵，二值化图矩阵，中值滤波图矩阵
    std::vector<std::vector<cv::Point>> contours;   //放二值化后检测到的轮廓用的向量
    std::vector<cv::Vec4i> hierarchy;   //cv::findContours中必须用的向量，该卡尔曼滤波未使用此功能，照着这样写就行
    std::vector<cv::Point3f> world_vec(4); //放世界坐标系用的向量，里面的元素都是三维点坐标
    std::vector<cv::Point2f> camera_vec(4);    //放相机坐标系用的向量，里面的元素都是二维点坐标

    
    cv::Point2f pts1[4];    
    cv::Point2f pts2[4];
    cv::Point2f top1;
    cv::Point2f bottom1;
    cv::Point2f top2;
    cv::Point2f bottom2;




    double len = 0.148;
    double width = 0.062;
    len = len/2;
    width = width/2;
    cv::Point3f wtop1(-len,width,0);
    cv::Point3f wtop2(len,width,0);
    cv::Point3f wbottom1(-len,-width,0);
    cv::Point3f wbottom2(len,-width,0);

    cv::Mat rvec;
    cv::Mat tvec;

    cv::Mat rotM;
    cv::Mat rotT;

    // Unused variables
    double theta_x;
    double theta_y;
    double theta_z;

    auto last_time = std::chrono::system_clock::now();
    auto new_time = std::chrono::system_clock::now();

    double k1,k2,b1,b2;
    cv::Point2f centre;

    //扩展卡尔曼变量
    Eigen::Matrix<double,6,1> X;
    //      X  vx   y  vy   z  vz
    X <<    0,  0,  0,  0,  0,  0;
    //上一次运算坐标位置
    Eigen::Matrix<double,3,1> X1;
    //     lx  ly  lz  
    X1 <<   0,  0,  0;

    //测量值矩阵
    Eigen::Matrix<double,3,1> X2;
    
    Eigen::Matrix<double,6,6> P;
    P.setIdentity();

    // Eigen::Matrix<double,6,6> F;
    // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> F; 这种自动大小的不能用
    Eigen::Matrix<double,6,6> F;
    F.setIdentity();

    //加速度a(待测量估计)
    double a = 0.02;
    double variable1;
    double variable2;
    double variable3;



    
    Eigen::Matrix<double,6,6> Q;
    Q.setIdentity();


    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> K;

    
    Eigen::Matrix<double,3,6> H;
    Eigen::Matrix<double,3,3> R;

    //单位矩阵
    Eigen::Matrix<double,6,6> I;
    I.setIdentity();

    //定义坐标变换输入坐标
    cv::Mat dian3 = (cv::Mat_<double>(3,1)<<0,0,0);
    cv::Mat dian2;

    cv::Point2f jieguo;

    cv::Mat X3(3,1,CV_64F);
    //test
    cv::Mat dian22;
    cv::Point2f jieguo1;






    


    while(true)
    {   
        auto start_time = std::chrono::system_clock::now();
        c.read(read_img);
        img = read_img.clone();
        cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY);
        
        cv::medianBlur(gray,gray,3);
        cv::Scalar lower = 160;
        cv::Scalar upper = 255;
        cv::inRange(gray,lower,upper,mask);
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if(contours.size()>1)
            {
                cv::RotatedRect rect1 = cv::minAreaRect(contours[0]);   //创建最小的能够框出检测到的轮廓的旋转矩形
                rect1.points(pts1);     //将矩形的四个点储存在pts1中
                std::sort(pts1,pts1 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});     //给上句话的四个点排序，y越小放在越前面
                top1 = (pts1[0] + pts1[1]) / 2;     //前两个pts1[0]和pts[1]的y小，一定是一个灯条拟合出的最小矩形的上面的两个点，求这两个点的中点。 (详见文档“灯条”页)
                bottom1 = (pts1[2] + pts1[3]) / 2;      //和上面一样，求灯条下面的端点
                cv::RotatedRect rect2 = cv::minAreaRect(contours[1]);   //因为是两个灯条，so理想情况下会识别到两个轮廓，这里和上面一样求第二个轮廓的上端点和下端点
                rect2.points(pts2);     
                std::sort(pts2,pts2 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});
                top2 = (pts2[0] + pts2[1]) / 2;
                bottom2 = (pts2[2] + pts2[3]) / 2;

                cv::line(img,top1,bottom2,cv::Scalar(0,255,0),1,8,0);   //根据上面求的四个点在装甲板上画一个x
                cv::line(img,top2,bottom1,cv::Scalar(0,255,0),1,8,0);

                if(top1.x>top2.x){                      //由于上面识别出来的轮廓不知道哪个是左边的哪个是右边的，so将他俩根据x坐标排一下序，
                    cv::Point2f top3;                   //根据相机坐标系，x小的在左边，x大的在右边
                    top3 = top1;
                    top1 = top2;
                    top2 = top3;
                    cv::Point2f bottom3;
                    bottom3 = bottom1;
                    bottom1 = bottom2;
                    bottom2 = bottom3;


                }

                camera_vec[0] = top1;              //将“识别到的四个二维点”(就是上面一段求的)储存在camera_vec向量中
                camera_vec[1] = top2;
                camera_vec[2] = bottom2;
                camera_vec[3] = bottom1;

                world_vec[0] = wtop1;          //world_vec向量中储存世界坐标系的点，要问世界坐标系怎么来的，是上面自己设的；关于怎么设，见文档“坐标系”
                world_vec[1] = wtop2;
                world_vec[2] = wbottom2;
                world_vec[3] = wbottom1;

                cv::solvePnP(world_vec,camera_vec,cameraMatrix,distCoeffs,rvec,tvec,false,cv::SOLVEPNP_IPPE);     
                //PnP解算，用法详见文档“pnp结算”把需要的数据喂进去即可，会吐出来所谓的旋转向量和平移向量，平移向量中储存的是(x,y,z)坐标，具体长什么样可以std::cout一下打印出来看看
               


                //在画面中用数字标出四个点
                cv::putText(img,"0",camera_vec[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"1",camera_vec[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"2",camera_vec[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"3",camera_vec[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);


                //求两直线交点centre
                k1 = (top1.y - bottom2.y)/(top1.x - bottom2.x);
                b1 = top1.y - k1*top1.x;
                k2 = (bottom1.y - top2.y)/(bottom1.x - top2.x);
                b2 = bottom1.y - k2*bottom1.x;

                centre.x = (b2 - b1)/(k1 - k2);
                centre.y = k1*centre.x + b1;

                


                

                cv::circle(img,centre,10,cv::Scalar(0,255,0),-1);       //画点用的

                //时间戳
                auto end_time = std::chrono::system_clock::now();       //记下现在的时间
        
                new_time = end_time;
               
                auto waste_time = std::chrono::duration_cast<std::chrono::microseconds>(new_time-last_time).count();
                last_time = end_time;       //更新时间
                double t = waste_time * 0.000001;   //微妙转换成秒  t为每次循环所用的时间
                
                //关于时间这里自己慢慢思考一下，应该能想明白

               


                //扩展卡尔曼//

                //预测方程
                X(1,0) = (tvec.at<double>(0,0) - X1(0,0)) / t;     //计算x,y,方向的速度，把它们放在X矩阵中该放的位置
                X(3,0) = (tvec.at<double>(1,0) - X1(1,0)) / t;     //计算方法用上面pnp解算出的tvec的坐标，x,y,z分别(新的x-上次循环的x) / t;
                X(5,0) = (tvec.at<double>(2,0) - X1(2,0)) / t;

               

                

                X(0,0) = X(0,0) + X(1,0) * 0.1;     //位置更新方程，原理就是 x = x0 + vt;
                X(2,0) = X(2,0) + X(3,0) * 0.1;
                X(4,0) = X(4,0) + X(5,0) * 0.1;

                //将预测后的点放进X3                
                X3.at<double>(0,0) = X(0,0);
                X3.at<double>(1,0) = X(2,0);
                X3.at<double>(2,0) = X(4,0);
               

                
                //将三维点转化为二维点,dian3在上面设为(0,0,0)，因为这里把装甲板中心点设为原点
                cv::projectPoints(dian3,rvec,X3,cameraMatrix,distCoeffs,dian2);
               
                jieguo.x = dian2.at<double>(0,0);       //将dain2矩阵转换为jieguo二维点坐标
                jieguo.y = dian2.at<double>(0,1);
                cv::circle(img,jieguo,5,cv::Scalar(0,0,255),-1);    //在图中画出jieguo这个点，使预测的数据能够更直观的表现在画面中
               
                
                //以下内容解释详见""公式解释"文档

                F <<    1,  0.1,  0,  0,  0,  0,
                        0,  1,  0,  0,  0,  0,
                        0,  0,  1,  0.1,  0,  0,
                        0,  0,  0,  1,  0,  0,
                        0,  0,  0,  0,  1,  0.1,
                        0,  0,  0,  0,  0,  1;

     

                variable1 = pow(t,4) / 4;
                variable2 = pow(t,2);
                variable3 = pow(t,3) / 2;
                Q <<    variable1,  variable3,          0,          0,          0,          0,
                        variable3,  variable2,          0,          0,          0,          0,
                                0,          0,  variable1,  variable3,          0,          0,
                                0,          0,  variable3,  variable2,          0,          0,
                                0,          0,          0,          0,  variable1,  variable3,
                                0,          0,          0,          0,  variable3,  variable2;

                Q = Q * a;
              
                
                P = F * P * F.transpose() + Q;

               

                //更新方程
                //Hnew1.3
                H <<    1,  0,  0,  0,  0,  0,
                        0,  0,  1,  0,  0,  0,
                        0,  0,  0,  0,  1,  0,
                
                
             
                //(R矩阵中的值待估计）
                R <<    0,  0,  0,
                        0,  0,  0,
                        0,  0,  0;

                K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
               
                
                X2 << tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0);
              

                X = X + K * (X2 - H * X);
               

                P = (I - K * H) * P;
               
                



                X1(0,0) = tvec.at<double>(0,0);
                X1(1,0) = tvec.at<double>(1,0);
                X1(2,0) = tvec.at<double>(2,0);


            }

        
        // cv::imshow("GRAY",gray);
        cv::imshow("test",img);
        

        

        if(cv::waitKey(1) == 27)
        {
            break;
        }
    }
    cv::destroyAllWindows();
    c.close();
    return 0;

}

