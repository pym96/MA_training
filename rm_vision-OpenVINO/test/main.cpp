#include <opencv2/core/mat.hpp>
#include <opencv2/core/traits.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Robot{
    enum class Color{
        BLUE,
        RED
    };

    struct Light: public cv::RotatedRect{
    Light() = default;
    explicit Light(cv::RotatedRect box): cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b){
            return a.y < b.y;
        });

        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);  
    }  

    
    Color color;
    cv::Point2f top, bottom;
    double width, length;

    };


    struct Armour{
        Armour() = default;
        Armour(const Light& light1, const Light & light2): camera_points(), world_points()
        {
            if(light1.center.x < light2.center.x){
                left_light = light1, right_light = light2;
            }else{
                left_light = light2, right_light = light1;
            }

            center = (left_light.center + right_light.center) / 2;
        } 

        cv::Point3f camera_points;
        cv::Point3f world_points;
        cv::Point2f center;

        Light left_light, right_light;
    };


    struct DetectionPack{
        cv::Mat img;
        double time_stamp;
        
        std::vector<Armour> armours;

        DetectionPack() = default;
        DetectionPack(cv::Mat& img, double time_stamp): img(img), time_stamp(time_stamp){
            
        }
    };

};



class Detect{

    public:
        explicit Detect(const Robot::Color& color)
        : color(color){

        }
        ~Detect() = default;
        Detect & operator = (Detect const&) = delete;

    private:
        Robot::Color color;

    public:
        bool detect(Robot::DetectionPack& detection_pack){
            cv::Mat process_img;
            
            std::vector<Robot::Light> lights;

            process(detection_pack.img, process_img);

            cv::imshow("Gray img", process_img);

            

            
            return false;
        }

    private:
        bool process(cv::Mat &img, cv::Mat &process_img ){
            cv::Mat temp_img;
            cv::cvtColor(img, process_img, cv::COLOR_BGR2GRAY);
            cv::inRange(temp_img, cv::Scalar(160), cv::Scalar(255), process_img);

            return true;
        }
        
    
};

int main(){
    
    
    Robot::Color color = Robot::Color::BLUE;


    Detect detector{color};




    return 0;
}