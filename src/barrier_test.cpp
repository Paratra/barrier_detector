#include "barrier_test/barrier_test.h"


uint8_t* pC;
uint8_t* pM;
using namespace std;
using namespace cv;
#define counter 1




int gapr=3, gapc=3, wantmeanNr=7, wantmeanNc=7;
Mat image_hsv;


Mat colorFilter(Mat frame){
    //cv::cvtColor(frame, image_hsv, CV_RGB2HSV); // From RGB to HSV

    Mat Res = Mat::zeros(frame.rows,frame.cols, CV_8U);
    //inRange(image_hsv,Scalar(85,111,170) , Scalar(99,256,256), Res);

    int nr = frame.rows;
    int nc = frame.cols;
    cv::cvtColor(frame, image_hsv,CV_RGB2HSV); // From RGB to HSV

    for (int ri = 0; ri < nr; ri = ri+gapr)
    {

        for (int cj = 0; cj <nc; cj = cj + gapc)
        {

            if ((((double)image_hsv.at<Vec3b>(ri,cj)[0] >= 85) && ((double)image_hsv.at<Vec3b>(ri,cj)[0] <= 99)) &&
            (((double)image_hsv.at<Vec3b>(ri,cj)[1] >= 111) && ((double)image_hsv.at<Vec3b>(ri,cj)[1] <= 256)) &&
            (((double)image_hsv.at<Vec3b>(ri,cj)[2] >= 170) && ((double)image_hsv.at<Vec3b>(ri,cj)[2] <= 256)))
            {


                Res.at<uchar>(ri,cj) = 255;


            }
             else
             {
                 Res.at<uchar>(ri,cj) = 0;

             }

        }
    }
    return Res;
}
Mat regionGrowing(Mat Res){
    Mat OutRes = Mat::zeros(Res.rows,Res.cols, CV_8U);
    // Vars
    int gapr=3, gapc=3, wantmeanNr=7, wantmeanNc=7;
    float thresR = 0.4, thresC = 0.5;
    int nr = Res.rows;
    int nc = Res.cols;
    double MR, MC;

    for (int r = 0; r < nr; r = r+ gapr)
    {

        for (int c = 0; c <nc; c = c+gapc)
        {

            if ((Res.at<uchar>(r,c)==255) && ((r>((wantmeanNr*gapr-gapr+1))) && (c>((wantmeanNc*gapc-gapc+1)))))
            {

                    MR = mean(Res(cv::Range(r-(wantmeanNr*gapr-gapr+1), r+1), cv::Range(c, c+1)))[0];
                    MC = mean(Res(cv::Range(r, r+1), cv::Range(c-(wantmeanNc*gapc-gapc+1), c+1)))[0];


                if (MR<=(((wantmeanNr*255)/(wantmeanNr*gapr-gapr+1))*thresR) && MC<=(((wantmeanNc*255)/(wantmeanNc*gapc-gapc+1))*thresC))
                {
                    OutRes(cv::Range(r-(wantmeanNr*gapr-gapr+1), r+1), cv::Range(c-(wantmeanNc*gapc-gapc+1), c+1)) = 0;

                } else {

                    OutRes(cv::Range(r-(wantmeanNr*gapr-gapr+1), r+1), cv::Range(c-(wantmeanNc*gapc-gapc+1), c+1)) = 255;
            }

             }
             else
             {
                 OutRes.at<uchar>(r,c)=(double)Res.at<uchar>(r,c);

             }

        }
    }
    return OutRes;
}








//########## CONSTRUCTOR ###############################################################################################
barrier_test_node::barrier_test_node(ros::NodeHandle &node_handle):
node_(&node_handle)
{
  // === PARAMETERS ===
  // node_->param("my_pkg/my_double_param", my_double_parameter_, 0.0);
  // node_->param("my_pkg/my_string_param", my_string_parameter_, std::string(""));
  // node_->param("my_pkg/my_int_param", my_int_parameter_, 0);
  // node_->param("my_pkg/my_bool_param", my_bool_parameter_, false);

  // === SUBSCRIBERS ===
  //my_subscriber_ = node_->subscribe("/joy", 10, &MyNode::subscriberCallback, this);
  //Turtle_subscriber_ = node_->subscribe("/teleop_twist_keyboard", 10, &MyNode::subscriberCallback, this);

  barrier_test_sub_ = node_->subscribe("/camera/rgb/image_raw", 1, &barrier_test_node::barrier_testCallback, this);

  // === PUBLISHERS ===
  //image_publisher_ = node_->advertise("/barrier_detector/output", 1);

  //test_pub_ = node_->advertise<std_msgs::String>("chatter", 1000);
  //Turtle_publisher_ = node_->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // === SERVICE SERVERS ===
  //my_service_server_ = node_->advertiseService("my_namespace/my_service", &MyNode::serviceCallback, this);

  // === SERVICE CLIENTS ===
  //my_service_client_ = node_->serviceClient<std_srvs::Empty>("some_service");

  // === TIMER ===
  //my_timer_ = node_->createTimer(ros::Duration(10), &MyNode::timerCallback, this);

  ROS_INFO("ROS Node Started");
}

//#################################

  void barrier_test_node::barrier_testCallback(const sensor_msgs::ImageConstPtr &msg)
  {

    Mat C, image_hsv;

    if(msg->height != C.rows)
    {
        C = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
    }
    // Resolution
    //res_x = (int)msg->width;     //res_x = 1280;
    //res_y = (int)msg->height;    //res_y = 720;

    pC = C.ptr<uint8_t>(0);
    memcpy(pC, &(msg->data.front()), C.cols*C.rows*3);

    // Convert to HSV
    cv::Mat hsv;                                                            //Image im HSV-Farbraum
    if(msg->encoding == sensor_msgs::image_encodings::RGB8)
        cv::cvtColor(C, hsv, CV_RGB2HSV);
    else
        cv::cvtColor(C, hsv, CV_RGB2HSV);

        Mat test[3];
      split(hsv, test);

      // namedWindow( "1", WINDOW_NORMAL ); // Create a window for display.
       imshow( "h", test[0]);                // Show our image inside it.
       cvWaitKey(1);
       imshow( "s", test[1]);                // Show our image inside it.
       cvWaitKey(1);
       imshow( "v", test[2]);                // Show our image inside it.
       cvWaitKey(1);



//std::cout << test[0] << '\n';

  }





//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "barrier_test_node_name");

  ros::NodeHandle node_handle;
  barrier_test_node barrier_test_node(node_handle);



  //ROS_INFO("DingDingDingDing....");
  ros::spin();

  return 0;
}
