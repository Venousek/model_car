#include "laneDetection.h"


using namespace std;
using namespace cv;

#define PAINT_OUTPUT

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;


cLaneDetection::cLaneDetection(ros::NodeHandle nh, int cam_w_, int cam_h_, int proj_y_start_,
        int proj_image_h_, int proj_image_w_, int roi_top_w_, int roi_bottom_w_, int roi_horizontal_offset_,
        int detector_size, int lane_width,
        std::string path_2features, std::string path_30features)
    : nh_(nh), priv_nh_("~"),detector(detector_size,Point(0,0),Point(proj_image_w_,proj_image_h_),
        proj_image_h_,proj_image_w_,roi_top_w_,roi_bottom_w_,roi_horizontal_offset_,path_2features, path_30features),model(true, proj_image_w_/2, lane_width)
{
    //m_Busy = false;
    //priv_nh_.param<std::string>("PATH_2FEATURES", PATH_2FEATURES, "");
    //priv_nh_.param<std::string>("PATH_30FEATURES", PATH_30FEATURES, "");

    cam_w = cam_w_;
    cam_h = cam_h_;
    proj_y_start = proj_y_start_;
    proj_image_h = proj_image_h_;
    proj_image_w = proj_image_w_;
    proj_image_w_half = proj_image_w/2;
    roi_top_w = roi_top_w_;
    roi_bottom_w = roi_bottom_w_;
    roi_horizontal_offset = roi_horizontal_offset_;

    m_LastValue = 0;
    read_images_ = nh.subscribe(nh_.resolveName("/camera/ground_image_ipmapped"), 1,&cLaneDetection::ProcessInput,this);


    publish_images = nh.advertise<sensor_msgs::Image>("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);

    publish_parabola = nh.advertise<std_msgs::Float32>("/lane_model/parabola", MY_ROS_QUEUE_SIZE);
}

cLaneDetection::~cLaneDetection()
{
}

int cLaneDetection::Init()
{
	//firstFrame = True;
	imagecount = 0;
	m_LastValue = 0;
	return 1;
}

//re-initialize the whole system
void cLaneDetection::resetSystem()
{
    //model = LaneModel(false);
}

void cLaneDetection::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    // VideoInput
    //std::cout << "Hey, listen!" << std::endl;
    //ROS_INFO("CERTAINTY:");
    //ros::Time begin = ros::Time::now();

    try
    {
        ros::WallTime begin = ros::WallTime::now();

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        
        Mat image;
        image = cv_ptr->image.clone();


        //Rect(X,Y,Width,Height)
        Mat transformedImage = image(Rect((cam_w/2)-proj_image_w_half,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat sobeledImage     = image(Rect((cam_w/2)-proj_image_w_half,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat groundPlane      = image(Rect((cam_w/2)-proj_image_w_half,
            proj_y_start,proj_image_w,proj_image_h)).clone();


		// cv_bridge::CvImage out_msg;
		// out_msg.header   = msg->header; // Same timestamp and tf frame as input image
		// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		// out_msg.image    = transformedImage; // Your cv::Mat
  		// publish_images_.publish(out_msg.toImageMsg());
        
        //create an output image for debugging
        //Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));
        vector<Point2d> laneMarkings = detector.detect(transformedImage,sobeledImage,groundPlane);

        //---------------------- DEBUG OUTPUT LANE MARKINGS ---------------------------------//
        #ifdef PAINT_OUTPUT
            Mat transformedImagePaintable = transformedImage.clone();
            cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
            for(int i = 0;i < (int)laneMarkings.size();i++)
            {
                circle(transformedImagePaintable,laneMarkings.at(i),1,Scalar(0,0,255),-1);
            }

            Point2d p1(proj_image_w_half-(roi_bottom_w/2)+roi_horizontal_offset,proj_image_h-1);
            Point2d p2(proj_image_w_half+(roi_bottom_w/2)+roi_horizontal_offset,proj_image_h-1);
            Point2d p3(proj_image_w_half+(roi_top_w/2)+roi_horizontal_offset,0);
            Point2d p4(proj_image_w_half-(roi_top_w/2)+roi_horizontal_offset,0);
            line(transformedImagePaintable,p1,p2,Scalar(0,200,0));
            line(transformedImagePaintable,p2,p3,Scalar(0,200,0));
            line(transformedImagePaintable,p3,p4,Scalar(0,200,0));
            line(transformedImagePaintable,p4,p1,Scalar(0,200,0));
            cv::imshow("Lane markings", transformedImagePaintable);
            //cv::imshow("Original image", image);
            cv::waitKey(1);
        #endif
        //---------------------- END DEBUG OUTPUT LANE MARKINGS ------------------------------//
        //use the detected lane markings to find contours in the image
        Mat circleImage(proj_image_h,proj_image_w,CV_8UC1,Scalar(0));
        for(int i = 0;i < (int)laneMarkings.size();i++)
        {
            circle(circleImage,laneMarkings.at(i),3,Scalar(255),-1);
        }

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
        ContourModel cModel;
        bool midLaneFound = cModel.update(contours,laneMarkings,proj_image_w_half);
        vector<vector<Point2d> > nicelyGroupedPoints = cModel.points;

        if (midLaneFound)
        	ROS_ERROR("Found a midlane!");
        
        #ifdef PAINT_OUTPUT
                //---------------------- DEBUG OUTPUT CONTOURS ---------------------------------//
            Mat contourImagePaintable = transformedImage.clone();
            cvtColor(contourImagePaintable,contourImagePaintable,CV_GRAY2BGR);            
            //draw the detected contours on the output image
            for(int i = 0;i < (int)nicelyGroupedPoints.size();i++)
            {
                Scalar color(255,255,255);
                if(i == 0)color = Scalar(0,0,255);if(i == 1)color = Scalar(0,255,0);if(i == 2)color = Scalar(255,0,0);
                if(i == 3)color = Scalar(255,255,0);if(i == 4)color = Scalar(255,0,255);if(i == 5)color = Scalar(255,255,0);

                vector<Point2d> pointGroup = nicelyGroupedPoints.at(i);                
                for(int j = 0;j < (int)pointGroup.size();j++)
                {
                    Point2d currP = pointGroup.at(j);
                    currP.x += proj_image_w_half;
                    circle(contourImagePaintable,currP,1,color,-1);
                }
            }
            cv::imshow("Contours", contourImagePaintable);

            //---------------------- END DEBUG OUTPUT CONTOURS------------------------------//
        #endif
       //ROS_ERROR("Found: %d",midLaneFound);
        model.improvedUpdate(&nicelyGroupedPoints,midLaneFound);
        //ROS_ERROR("CERTAINTY: %d",model.certainty);
        // SEND OUT OLD LANE INFORMATION
        double curvature; //1/cm
        double distanceRightLane; //cm
        double angle; //rad
        bool isCurve;
        model.getCarState(&curvature,&distanceRightLane,&angle,&isCurve);
        //send is curve info:
        int stamp = 0;
        ros::WallTime end = ros::WallTime::now();
        ros::WallDuration d= end-begin;
        //ROS_ERROR("time: %ld", d.toNSec()/1000000); 


        // ros::Time end = ros::Time::now();
        // ROS_ERROR("time: %d", ((end.nsec-begin.nsec)/1000000)); 
        
            //---------------------- DEBUG OUTPUT LANE MODEL---------------------------------//
            int carOffset = 50;
            Mat laneModelDrawing(proj_image_h+carOffset,proj_image_w,CV_8UC3,Scalar(0,0,0));
            Mat transformedImageCopy = transformedImage.clone();
            cvtColor(transformedImageCopy,transformedImageCopy,CV_GRAY2BGR);
            transformedImageCopy.copyTo(laneModelDrawing(Rect(0,carOffset,transformedImageCopy.cols,transformedImageCopy.rows)));
            model.getDebugImage(laneModelDrawing);
        #ifdef PAINT_OUTPUT
            cv::imshow("Lane model", laneModelDrawing);
             
           //---------------------- END DEBUG OUTPUT LANE MODEL------------------------------//
        #endif

        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
        out_msg.image    = laneModelDrawing; // Your cv::Mat
        publish_images.publish(out_msg.toImageMsg());

        std_msgs::Float32 parabola;
        parabola.data = model.getHistoryParabola().at(0);
        publish_parabola.publish(parabola);

    } 
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetection");
    ros::NodeHandle nh;

    //std::string node_name = "/line_detection_node/";
    std::string node_name = ros::this_node::getName();

    ROS_ERROR("%s",node_name.c_str());


    int cam_w;
    int cam_h;
    int proj_y_start;
    int proj_image_h;
    int proj_image_w;
    int roi_top_w;
    int roi_bottom_w;
    int roi_horizontal_offset;
    int detector_size;
    int lane_width;
    std::string path_2features;
    std::string path_30features;

    //nh.param<std::string>("camera_name", camera_name, "/usb_cam/image_raw"); 
    nh.param<int>("cam_w", cam_w, 640);
    nh.param<int>("cam_h", cam_h, 480);
    nh.param<int>(node_name+"/proj_y_start", proj_y_start, 400);
    nh.param<int>(node_name+"/proj_image_h", proj_image_h, 40);
    nh.param<int>(node_name+"/proj_image_w", proj_image_w, 80);
    nh.param<int>(node_name+"/roi_top_w", roi_top_w, 30);
    nh.param<int>(node_name+"/roi_bottom_w", roi_bottom_w, 20);
    nh.param<int>(node_name+"/roi_horizontal_offset", roi_horizontal_offset, 0);
    nh.param<int>(node_name+"/detector_size", detector_size, 16);
    nh.param<int>(node_name+"/lane_width", lane_width, 17);
    nh.param<std::string>(node_name+"/path_2features", path_2features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_2features.txt");
    nh.param<std::string>(node_name+"/path_30features", path_30features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_30features.txt");

    cLaneDetection node(nh, cam_w, cam_h, proj_y_start, proj_image_h, proj_image_w,
        roi_top_w, roi_bottom_w, roi_horizontal_offset, detector_size, lane_width, path_2features, path_30features);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}