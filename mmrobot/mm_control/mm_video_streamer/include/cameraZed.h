 // Standard includes
#include <stdio.h>
#include <string.h>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>


cv::Mat slMat2cvMat(sl::Mat& input);

// Using std and sl namespaces
using namespace std;
using namespace sl;

class ZedWrapper{
private:
    Camera zed;
    sl::Mat zed_image;
    sl::RuntimeParameters runtime_param;
public:
    ZedWrapper(){
        runtime_param.sensing_mode = SENSING_MODE_STANDARD;
        runtime_param.enable_depth = false;
        runtime_param.enable_point_cloud = false;
    };
    ~ZedWrapper(){
        zed.close();
    };
    double getFPS(){
        return zed.getCameraFPS();
    }
    cv::Size getResolution(){
        return cv::Size((int) zed.getResolution().width, (int) zed.getResolution().height);
    }
    void init(){
        // --- Initialize a Camera object and open the ZED
        // Set configuration parameters
        InitParameters init_params;
        // Use HD720 video mode
        init_params.camera_resolution = RESOLUTION_HD2K;
        
        // Set fps at 15
        init_params.camera_fps = 15; 
        
        // Open the camera
        ERROR_CODE err = zed.open(init_params);
        if (err != SUCCESS) {
            cout << toString(err) << endl;
            printf("Zed is not open");
            zed.close(); 
            // Quit if an error occurred
        }
        //zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE,70,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_GAIN,50,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_BRIGHTNESS,4,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_CONTRAST,4,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_HUE,0,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_SATURATION,4,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_WHITEBALANCE,4700,false);
        //zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE,60,fals
        //zed.setCameraSettings(CAMERA_SETTINGS_BRIGHTNESS,6,false);
        
        // Print camera information
        //printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
        printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
        printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
        printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
        printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());
    }
    int grab(cv::Mat& image, string side){
        if (zed.grab(runtime_param) == SUCCESS) {
            if(side == "left"){
                // Retrieve left image
                zed.retrieveImage(zed_image, VIEW_LEFT_UNRECTIFIED);
                image = slMat2cvMat(zed_image);
                cv::cvtColor(image, image, cv::COLOR_RGBA2RGB);
                return 0;
            }
            if(side == "right"){
                // Retrieve left image
                zed.retrieveImage(zed_image, VIEW_RIGHT_UNRECTIFIED);
                image = slMat2cvMat(zed_image);
                cv::cvtColor(image, image, cv::COLOR_RGBA2RGB);
                return 0;
            }
        }
        return -1; //no image is grabed 
    }
    int grab(cv::Mat& left, cv::Mat& right){
        if (zed.grab(runtime_param) == SUCCESS) {
            
            // Retrieve left image
            zed.retrieveImage(zed_image, VIEW_LEFT_UNRECTIFIED);
            cv::Mat cv_left_image = slMat2cvMat(zed_image);
            cv::cvtColor(cv_left_image, left, cv::COLOR_RGBA2RGB);
            zed.retrieveImage(zed_image, VIEW_RIGHT_UNRECTIFIED);
            cv::Mat cv_right_image = slMat2cvMat(zed_image);
            cv::cvtColor(cv_right_image, right, cv::COLOR_RGBA2RGB);
            return 0;
            
        }
        return -1; //no image is grabed 
    }
};



cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
