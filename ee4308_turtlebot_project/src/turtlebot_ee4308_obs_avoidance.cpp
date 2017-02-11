#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <cmath>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <tr1/tuple> // if gcc > 4.7 need to use #include <tuple> // std::tuple, std::make_tuple, std::tie

namespace patch{
    template < typename T > std::string to_string(const T& n){
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "RGB Camera";
static const char dWINDOW[] = "depth Camera";
static const char gWINDOW[] = "Canny image";

/*--------- classes for image converter------------*/

std::tr1::tuple<cv::Mat, std::vector<float> > computeVLines(cv::Mat imageMat_after){

    cv::Mat err, gimageMat_canny, imageMat_canny;
    std::vector<float> Pt;
    std::tr1::tuple<cv::Mat, std::vector<float> > cannyReturn;


    Canny( imageMat_after, gimageMat_canny, 7, 40, 3);
    cvtColor(gimageMat_canny, imageMat_canny, CV_GRAY2BGR);

    cv::imshow("Canny Image", gimageMat_canny);

    std::vector<cv::Vec2f> lines;
    HoughLines(gimageMat_canny, lines, 1, CV_PI/180, 100);

    for (size_t i =0; i<lines.size(); i++){
        float rho = lines[i][0];
        float theta = lines[i][1];

        if(theta==0){
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;

            cv::Point pt1(cvRound(x0 +1000*(-b)), cvRound(y0+1000*(a)));
            cv::Point pt2(cvRound(x0 -1000*(-b)), cvRound(y0-1000*(a)));

            line(imageMat_canny, pt1, pt2, cv::Scalar(0,0,255), 2, 8);
            Pt.push_back(pt1.x);
        }
    }

    cannyReturn = std::tr1::make_tuple(imageMat_canny,Pt);
    return(cannyReturn);
}

class ImageConverter{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh2_;
    ros::Subscriber pos_sub;
    ros::Publisher cmd_vel_pub;
    image_transport::ImageTransport it_;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::Subscriber< nav_msgs::Odometry> OdometrySubscriber;
    ImageSubscriber rgb_image_sub_, depth_image_sub_;
    OdometrySubscriber odoSub_;

    typedef message_filters::sync_policies::ApproximateTime< nav_msgs::Odometry,sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer < MySyncPolicy > sync;

    cv::Mat rgbImageMat, imageMat_canny;
    float big_f, small_l;

public:
  ImageConverter(ros::NodeHandle &nh):
    // Subscribing to the image data and odometry data
    it_(nh_),
    odoSub_(nh2_,"/odom", 1),
    rgb_image_sub_(it_,"/camera/rgb/image_raw", 1),
    depth_image_sub_(it_,"/camera/depth/image_raw",1),

    sync ( MySyncPolicy(10),odoSub_,rgb_image_sub_,depth_image_sub_)
    {   cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
        sync.registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2,_3));

    }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }


  void imageCb(const nav_msgs::OdometryConstPtr& poseMsg, const sensor_msgs::ImageConstPtr& rgbmsg, const sensor_msgs::ImageConstPtr& depthmsg )
  {

    cv_bridge::CvImageConstPtr cv_ptr;

    /*
     *  For displaying depth images
     */

    try{
        cv_ptr = cv_bridge::toCvShare(depthmsg, enc::TYPE_32FC1);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // To get the depth at the optical centre (320,240)px
    std::cout << " depth at centre " << cv_ptr->image.at<float>( 320,240 ) << std::endl;


    /*
     *  For displaying RGB images
     */


    try
    {
      cv_ptr = cv_bridge::toCvShare(rgbmsg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow(WINDOW, cv_ptr->image);

    /*
     *  To identify the vertical edges in the RGB image
     */

    rgbImageMat = cv_ptr->image;

    std::tr1::tuple<cv::Mat, std::vector<float> > cannyReturn;
    std::vector<float> pointReturn;

    cv::Size rgb_size_ = rgbImageMat.size();

    if(rgb_size_.height>0 && rgb_size_.width>0){
        cannyReturn = computeVLines(rgbImageMat);

        std::tr1::tie(imageMat_canny, pointReturn) = cannyReturn;

        if(pointReturn.size()>0){
            std::vector<float> big, small;
            big_f=640;
            small_l=0;

            for(int i=0; i< pointReturn.size();i++){

                if(pointReturn[i]>320){
                    big.push_back(pointReturn[i]);
                }else{
                    small.push_back(pointReturn[i]);
                }
            }
            if(big.size()>0){
                sort(big.begin(), big.end());
               // big_f = big[0];
                big_f = big[big.size()-1];
            }
            if(small.size()>0){
                sort(small.begin(), small.end());
                //small_l = small[small.size()-1];
                small_l = small[0];
            }
        }
        //std::cout << "Right edge " << big_f << " " << " Left edge " << small_l << std::endl;
    }

    cv::waitKey(3);

  }

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_viewer");
  ros::NodeHandle nh;

  ImageConverter ic(nh);

  ros::spin();
  return 0;
}
