#include <opencv2/core/mat.hpp>
#include <opencv2/core/traits.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn/dnn.hpp>
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

    enum class ArmourType{
        Small,
        Big
    };

};



class Detect{

    public:
        explicit Detect(const Robot::Color& color)
        : color{color}{

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



class NumberClassfier{

    public:
        NumberClassfier(
            const std::string & model_path, const double& threshold){
                net_ = cv::dnn::readNetFromONNX(model_path,0.85);
            }

    void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors){
        // Light length in image
        const int light_length = 12;
        // Image size after warp
        const int warp_height = 28;
        const int small_armor_width = 32;
        const int large_armor_width = 54;
        // Number ROI size
        const cv::Size roi_size(20, 28);

        int i = 0;
        for (auto & armor : armors) {
            // Warp perspective transform
            cv::Point2f lights_vertices[4] = {
                armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
                armor.right_light.bottom};

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = armor.armour_type == Robot::ArmourType::Small ? small_armor_width : large_armor_width;
            cv::Point2f target_vertices[4] = {
                cv::Point(0, bottom_light_y),
                cv::Point(0, top_light_y),
                cv::Point(warp_width - 1, top_light_y),
                cv::Point(warp_width - 1, bottom_light_y),
            };
        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
        cv::imshow("warp", number_image);
        // Get ROI
        number_image =
            number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
        // cv::imshow(std::to_string(i), number_image);
        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        armor.number_img = number_image;
        // cv::imshow("Gray" + std::to_string(i), number_image);
    }
}

    void doClassify(std::vector<Armor> & armors){

    int i = 0;
    for (auto & armor : armors) {
        cv::Mat image = armor.number_img.clone();

        // Normalize -> 1
        image = image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1., cv::Size(28, 20));

        // Set the input blob for the neural network
        net_.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number = class_names_[label_id];

        std::stringstream result_ss;
        result_ss 
        << armor.number 
        << ":_" << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
        armor.classfication_result = result_ss.str();
    }

    armors.erase(
        std::remove_if(
            armors.begin(), armors.end(),
                [this](const Armor & armor) {

                if (armor.confidence < threshold || armor.number == 'N') {
                    return true;
                }   

            bool mismatch = false;
        if (armor.armour_type == Robot::ArmourType::Small) {
           mismatch = armor.number == 'O' || armor.number == '2' || armor.number == '3' ||
                      armor.number == '4' || armor.number == '5';
         } else if (armor.armour_type == Robot::ArmourType::Big) {
           mismatch = armor.number == '1' || armor.number == 'B' || armor.number == 'G';
         }
         return mismatch;
      }),
    armors.end());
}

    private:
        cv::dnn::Net net_;
        std::vector<char> class_names_;
        double threshold;

    public:
        using Armor = Robot::Armour;

};


int main(){
    
    
    Robot::Color color = Robot::Color::BLUE;
    Detect detector{color};
    NumberClassfier number_classfier("/home/dan/train/train/MA_training/MA_training/rm_vision-OpenVINO/test/opt-1208-001.onnx");

    // Create a VideoCapture object to access the camera (usually camera index 0 is the default built-in camera)
    cv::VideoCapture cap(0);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (true) {
        // Capture a frame from the camera
        cap >> frame;

        // Check if the frame was captured successfully
        if (frame.empty()) {
            std::cerr << "Error: Frame is empty." << std::endl;
            break;
        }

        // Display the frame in a window
        cv::imshow("Camera", frame);

        // Check for user input to exit (usually by pressing the 'q' key)
        char key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 'q' or Esc key
            break;
        }
    }

    // Release the camera and close the window
    cap.release();
    cv::destroyAllWindows();

    return 0;




    return 0;
}