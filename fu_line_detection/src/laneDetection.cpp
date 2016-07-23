#include "laneDetection.h"
//#include "tools/Point.h"


using namespace std;
//using namespace cv;

//#define PAINT_OUTPUT

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

#define PI 3.14159265

//image_transport::CameraPublisher realsense_rgb_image_pub;

//msgs head
/*unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;
std::string rgb_frame_id = "_rgb_optical_frame";
sensor_msgs::CameraInfoPtr rgb_camera_info;*/

// try kernel width 5 for now
const static int g_kernel1DWidth = 5;

cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh, int cam_w_, int cam_h_, int proj_y_start_,
        int proj_image_h_, int proj_image_w_, int roi_top_w_, int roi_bottom_w_, int proj_image_horizontal_offset_)
    : nh_(nh), priv_nh_("~")/*,detector(detector_size,Point(0,0),Point(proj_image_w_,proj_image_h_),
        proj_image_h_,proj_image_w_,roi_top_w_,roi_bottom_w_,path_2features, path_30features),model(true, proj_image_w_/2, lane_width)*/
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

    polyDetectedLeft     = false;
    polyDetectedCenter   = false;
    polyDetectedRight    = false;

    bestPolyLeft         = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter       = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight        = std::make_pair(NewtonPolynomial(), 0);

    laneMarkingsLeft     = std::vector<FuPoint<int>>();
    laneMarkingsCenter   = std::vector<FuPoint<int>>();
    laneMarkingsRight    = std::vector<FuPoint<int>>();

    polyLeft             = NewtonPolynomial();
    polyCenter           = NewtonPolynomial();
    polyRight            = NewtonPolynomial();

    supportersLeft       = std::vector<FuPoint<int>>();
    supportersCenter     = std::vector<FuPoint<int>>();
    supportersRight      = std::vector<FuPoint<int>>();

    prevPolyLeft         = NewtonPolynomial();
    prevPolyCenter       = NewtonPolynomial();
    prevPolyRight        = NewtonPolynomial();

    pointsLeft           = std::vector<FuPoint<int>>();
    pointsCenter         = std::vector<FuPoint<int>>();
    pointsRight          = std::vector<FuPoint<int>>();

    lanePoly             = NewtonPolynomial();
    lanePolynomial       = LanePolynomial();

    iterationsRansac     = 0;
    proportionThreshould = 0;


    defaultYLeft = 10;
    defaultYCenter = 40;
    defaultYRight = 70;

    m_gradientThreshold = 10;
    m_nonMaxWidth = 10;

    //int threshold(6);
    squaredThreshold = 36;

    //laneMarkings();

    //proj_image_horizontal_offset = proj_image_horizontal_offset_;

    //head_time_stamp = ros::Time::now();

    //m_LastValue = 0;
    read_images_ = nh.subscribe(nh_.resolveName("/camera/ground_image_ipmapped"), 1,&cLaneDetectionFu::ProcessInput,this);

    //publish_curvature = nh.advertise<std_msgs::Float32>("/lane_model/curvature", MY_ROS_QUEUE_SIZE);

    //image_transport::ImageTransport image_transport(nh);
    
    //realsense_rgb_image_pub = image_transport.advertiseCamera("/lane_model/lane_model_image", 1);

    /*if (!rgb_camera_info)
    {
        rgb_camera_info.reset(new sensor_msgs::CameraInfo());
        rgb_camera_info->width = proj_image_w;
        rgb_camera_info->height = proj_image_h+50;
    }*/

    //from camera properties and ROI etc we get scanlines (=line segments, úsečky)
    //these line segments are lines in image on whose we look for edges
    //the outer vector represents rows on image, inner vector is vector of line segments of one row, usualy just one line segment
    //we should generate this only once in the beginning! or even just have it pregenerated for our cam
    scanlines = getScanlines();


    ROS_ERROR_STREAM(scanlines.size() << " scanlines generated.");

    for(int i=0; i<scanlines.size(); ++i) {
        ROS_ERROR_STREAM(scanlines[i].size() << " LineSegment in scanline.");
        for(int j=0; j<scanlines[i].size(); ++j) {
            ROS_ERROR_STREAM(scanlines[i][j].getStart() << " is start.");
            ROS_ERROR_STREAM(scanlines[i][j].getEnd() << " is end.");
        }
    }
}

cLaneDetectionFu::~cLaneDetectionFu()
{
}

int cLaneDetectionFu::Init()
{
	//firstFrame = True;
	imagecount = 0;
	m_LastValue = 0;
	return 1;
}

//re-initialize the whole system
void cLaneDetectionFu::resetSystem()
{
    //model = LaneModel(false);
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Got IMG");

    // set variables to config values
    /*defaultYLeft   = cfgDefaultLeft->get() * meters;
    defaultYCenter = cfgDefaultCenter->get() * meters;
    defaultYRight  = cfgDefaultRight->get() * meters;*/

    /*minXRoi        = cfgMinXRoi->get() * meters;
    maxXDefaultRoi = cfgMaxXDefaultRoi->get() * meters;
    maxXPolyRoi    = cfgMaxXPolyRoi->get() * meters;*/

    interestDistancePoly = 10;//(cfgPolyRoiWidth->get() / 2) * meters;
    interestDistanceDefault = 10;//(cfgDefaultRoiWidth->get() / 2) * meters;

   /* iterationsRansac = cfgIterRansac->get();
    proportionThreshould = cfgThreshould->get();

    maxDistance    = cfgMaxDistance->get() * meters;*/

    // clear some stuff from the last cycle
    bestPolyLeft = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight = std::make_pair(NewtonPolynomial(), 0);

    supportersLeft.clear();
    supportersCenter.clear();
    supportersRight.clear();




    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    cv::Mat image = cv_ptr->image.clone();
    
    

    //scanlines -> edges (in each scanline we find maximum and minimum of kernel fn ~= where the edge is)
    //this is where we use input image!
    vector<vector<EdgePoint>> edges = cLaneDetectionFu::scanImage(image);

    //edges -> lane markings
    vector<FuPoint<int>> laneMarkings = cLaneDetectionFu::extractLaneMarkings(edges);

    // start actual execution
    buildLaneMarkingsLists(laneMarkings);

    ransac();

    detectLane(7);
/*
    // Debugging

    // debug default lines and rois
    drawDefaultLines(dbgDefaultLines, 0, 255, 255);
    drawDefaultRois(dbgDefaultRois, 0, 170, 170);

    // lane markings found in current used rois
    drawLaneMarkings(dbgLaneMarkingsLeft, laneMarkingsLeft, 0, 100, 100);
    drawLaneMarkings(dbgLaneMarkingsCenter, laneMarkingsCenter, 100, 100, 0);
    drawLaneMarkings(dbgLaneMarkingsRight, laneMarkingsRight, 100, 0, 0);

    // debug polys and poly rois
    if (polyDetectedLeft) {
        drawPoly(dbgPolyLeft, polyLeft, 0, 255, 255);
        drawPolyRoi(dbgRoiLeft, polyLeft, 0, 200, 200);
        drawSupporters(dbgSupportersLeft, supportersLeft, 200, 255, 255);
        drawPoints(dbgPointsLeft, pointsLeft, 0, 50, 50);
    }

    if (polyDetectedCenter) {
        drawPoly(dbgPolyCenter, polyCenter, 255, 255, 0);
        drawPolyRoi(dbgRoiCenter, polyCenter, 200, 200, 0);
        drawSupporters(dbgSupportersCenter, supportersCenter, 255, 255, 200);
        drawPoints(dbgPointsCenter, pointsCenter, 50, 50, 0);
    }

    if (polyDetectedRight) {
        drawPoly(dbgPolyRight, polyRight, 255, 0, 0);
        drawPolyRoi(dbgRoiRight, polyRight, 200, 0, 0);
        drawSupporters(dbgSupportersRight, supportersRight, 255, 200, 200);
        drawPoints(dbgPointsRight, pointsRight, 50, 0, 0);
    }

    writeProportions();

    if (lanePoly.getDegree() != -1) {
        drawPoly(dbgLanePoly, lanePoly, 255, 255, 255);
    }*/


/*



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
        Mat transformedImage = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat sobeledImage     = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat groundPlane      = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
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

            Point2d p1(proj_image_w_half-(roi_bottom_w/2),proj_image_h-1);
            Point2d p2(proj_image_w_half+(roi_bottom_w/2),proj_image_h-1);
            Point2d p3(proj_image_w_half+(roi_top_w/2),0);
            Point2d p4(proj_image_w_half-(roi_top_w/2),0);
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
        ROS_ERROR("time: %ld", d.toNSec()/1000000); 


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

        pubRealSenseRGBImageMsg(laneModelDrawing);



        //out_msg.header   = std_msgs::Header();
        //out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
        //out_msg.image    = laneModelDrawing; // Your cv::Mat
        //publish_images.publish(out_msg.toImageMsg());

        std_msgs::Float32 curvMsg;
        curvMsg.data = curvature*100;
        publish_curvature.publish(curvMsg);

    } 
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
*/
}



/* EdgeDetector methods */

/**
 * Compute scanlines. Each may consist of multiple segments, split at regions
 * that should not be inspected by the kernel.
 * @param side
 * @return vector of segments of scanlines, walk these segments with the kernel
 */
vector<vector<LineSegment<int>> > cLaneDetectionFu::getScanlines() {
    vector<vector<LineSegment<int>> > scanlines;

    vector<cv::Point> checkContour;
    checkContour.push_back(cv::Point(proj_image_w_half-(roi_bottom_w/2),proj_image_h-1));
    checkContour.push_back(cv::Point(proj_image_w_half+(roi_bottom_w/2),proj_image_h-1));
    checkContour.push_back(cv::Point(proj_image_w_half+(roi_top_w/2),0));
    checkContour.push_back(cv::Point(proj_image_w_half-(roi_top_w/2),0));


    int distance = 3;
    int count = 100;
    
    int scanlineStart = 0;
    int scanlineEnd = proj_image_w;

    int segmentStart = -1;
    vector<LineSegment<int>> scanline;
    //i = y; j = x;
    for (int i = 0; (i/distance) < count && i <= proj_image_h; i += distance) {
        scanline = vector<LineSegment<int>>();
        
        // walk along line
        for (int j = scanlineStart; j <= scanlineEnd; j ++) {
            bool isInside = pointPolygonTest(checkContour, cv::Point(j, i),false) >= 0;//   roi.isInside(Point<int>(offset, j));
            
            // start new scanline segment
            if (isInside && j < scanlineEnd) {
                if (segmentStart == -1) segmentStart = j;
            // found end of scanline segment, reset start
            } else if (segmentStart != -1) {                
                scanline.push_back(
                        LineSegment<int>(
                                FuPoint<int>(segmentStart, i),
                                FuPoint<int>(j-1, i)
                            )
                        );
                
                segmentStart = -1;
            }
        }
        // push segments found
        if (scanline.size()) {
            scanlines.push_back(scanline);
        }
    }
    return scanlines;
}

/**
 * Walk with prewitt/sobel kernel along all scanlines.
 * @param image
 * @return All edgePoints on side, sorted by scanlines.
 */
vector<vector<EdgePoint>> cLaneDetectionFu::scanImage(cv::Mat image) {
    vector<vector<EdgePoint>> edgePoints;
    
    //const Image &image = getImage();
    //const ImageDimensions &imgDim = getImageDimensions();
    //const OmnidirectionalCameraMatrix &cameraMatrix = getOmnidirectionalCameraMatrix();

    // scanline length can maximal be image height/width
    int scanlineMaxLength = image.cols;
    
    // store kernel results on current scanline in here
    vector<int> scanlineVals(scanlineMaxLength, 0);

    // walk over all scanlines
    for (auto scanline : scanlines) {
        // set all brightness values on scanline to 0;
        std::fill(scanlineVals.begin(), scanlineVals.end(), 0);
        int offset = 0;
        if (scanline.size()) {
            offset = scanline.front().getStart().getX();            
        }
        // scanline consisting of multiple segments
        // walk over each but store kernel results for whole scanline
        for (auto segment : scanline) {         
            int start = segment.getStart().getY();
            int end = segment.getEnd().getY();
            
            // walk along segment
            for (int i = start; i < end - g_kernel1DWidth; i++) {
                int sum = 0;                

                // use kernel width 5 and try sobel kernel
                sum -= image.at<int>(offset -1, i);
                sum -= image.at<int>(offset -1, i+1);
                // kernel is 0
                sum += image.at<int>(offset -1, i+3);
                sum += image.at<int>(offset -1, i+4);

                sum -= 2*image.at<int>(offset, i);
                sum -= 2*image.at<int>(offset, i+1);
                // kernel is 0
                sum += 2*image.at<int>(offset, i+3);
                sum += 2*image.at<int>(offset, i+4);

                sum -= image.at<int>(offset +1, i);
                sum -= image.at<int>(offset +1, i+1);
                // kernel is 0
                sum += image.at<int>(offset +1, i+3);
                sum += image.at<int>(offset +1, i+4);
        
                // +4 because of sobel weighting
                sum = sum / (3 * g_kernel1DWidth + 4);
                if (std::abs(sum) > m_gradientThreshold) {
                    // set scanlineVals at center of kernel
                    scanlineVals[i + g_kernel1DWidth/2] = sum;
                }
            }
        }

        // after walking over all segments of one scanline
        // do non-max-suppression
        // for both minima and maxima at same time
        // TODO: Jannis: find dryer way
        int indexOfLastMaximum = 0;
        int valueOfLastMaximum = 0;
        int indexOfLastMinimum = 0;
        int valueOfLastMinimum = 0;
        for (int i = 1; i < scanlineMaxLength -1; i++) {
            // check if maximum
            if (scanlineVals[i] > 0) {
                if (scanlineVals[i] < scanlineVals[i-1] or scanlineVals[i] < scanlineVals[i+1]) {
                    scanlineVals[i] = 0;
                }
                else {
                    // this pixel can just survive if the next maximum is not too close
                    if (i - indexOfLastMaximum > m_nonMaxWidth) {
                        // this is a new maximum
                        indexOfLastMaximum = i;
                        valueOfLastMaximum = scanlineVals[i];
                    }
                    else {
                        if (valueOfLastMaximum < scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMaximum] = 0;
                            indexOfLastMaximum = i;
                            valueOfLastMaximum = scanlineVals[i];
                        }
                        else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
            // check if minimum
            if (scanlineVals[i] < 0) {
                if (scanlineVals[i] > scanlineVals[i-1] or scanlineVals[i] > scanlineVals[i+1]) {
                    scanlineVals[i] = 0;
                }
                else {
                    // this pixel can just survive if the next minimum is not too close
                    if (i - indexOfLastMinimum > m_nonMaxWidth) {
                        // this is a new minimum
                        indexOfLastMinimum = i;
                        valueOfLastMinimum = scanlineVals[i];
                    }
                    else {
                        if (valueOfLastMinimum > scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMinimum] = 0;
                            indexOfLastMinimum = i;
                            valueOfLastMinimum = scanlineVals[i];
                        }
                        else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
        }
        // collect all the edgePoints for scanline
        vector<EdgePoint> scanlineEdgePoints;
        for (int i = 0; i < static_cast<int>(scanlineVals.size()); i++) {
            if (scanlineVals[i] != 0) {
                FuPoint<int> imgPos = FuPoint<int>(offset, i);
                
                FuPoint<Meter> relPos = FuPoint<Meter>();//offset, i);//cameraMatrix.transformToLocalCoordinates(imgPos);
                scanlineEdgePoints.push_back(EdgePoint(imgPos, relPos, scanlineVals[i]));
            }
        }
        edgePoints.push_back(std::move(scanlineEdgePoints));
    }
    // after walking along all scanlines
    // return edgePoints
    return edgePoints;
}


/* LaneMarkingDetector methods */


//uses Edges to extract lane markings
std::vector<FuPoint<int>> cLaneDetectionFu::extractLaneMarkings(const std::vector<std::vector<EdgePoint>>& edges) {
    vector<FuPoint<int>> result;

    for (const auto& edge : edges) {
        if (edge.empty()) continue;
    
        for (
            auto edgePosition = edge.begin(), nextEdgePosition = edgePosition + 1;
            nextEdgePosition != edge.end();
            edgePosition = nextEdgePosition, ++nextEdgePosition
        ) {
            if (edgePosition->isPositive() and not nextEdgePosition->isPositive()) {
                FuPoint<int> candidateStartEdge = edgePosition->getImgPos();
                FuPoint<int> candidateEndEdge = nextEdgePosition->getImgPos();
                if ((candidateStartEdge - candidateEndEdge).squaredMagnitude() < squaredThreshold) {
                    result.push_back(center(candidateStartEdge, candidateEndEdge));
                }
            }
        }
    }
    return result;
}


/**
 * Creates three vectors of lane marking points out of the given lane marking
 * point vector.
 *
 * A point has to lie within the ROI of the previously detected lane polynomial
 * or within the default ROI, if no polynomial was detected.
 * The lists are the input data for the RANSAC algorithm.
 *
 * @param laneMarkings  a vector containing all detected lane markings
 */
void cLaneDetectionFu::buildLaneMarkingsLists(
        const std::vector<FuPoint<int>> &laneMarkings) {
    laneMarkingsLeft.clear();
    laneMarkingsCenter.clear();
    laneMarkingsRight.clear();

    for (FuPoint<int> laneMarking : laneMarkings) {
        if (polyDetectedLeft) {
            if (isInPolyRoi(polyLeft, laneMarking)) {
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedCenter) {
            if (isInPolyRoi(polyCenter, laneMarking)) {
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedRight) {
            if (isInPolyRoi(polyRight, laneMarking)) {
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        if (isInDefaultRoi(LEFT, laneMarking)) {
            laneMarkingsLeft.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(CENTER, laneMarking)) {
            laneMarkingsCenter.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(RIGHT, laneMarking)) {
            laneMarkingsRight.push_back(laneMarking);
            continue;
        }
    }
}




/**
 * Calculates the horizontal distance between a point and the default line given
 * by its position.
 *
 * @param line  The position of the default line (LEFT, CENTER or RIGHT)
 * @param p     The given point
 * @return      The horizontal distance between default line and point
 */
int cLaneDetectionFu::horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p) {
    double pY = p.getY();
    double distance = 0;

    switch (line) {
    case LEFT:
        distance = std::abs(pY - defaultYLeft);
        break;
    case CENTER:
        distance = std::abs(pY - defaultYCenter);
        break;
    case RIGHT:
        distance = std::abs(pY - defaultYRight);
        break;
    }

    return distance;
}

/**
 * Calculates the horizontal distance between a point and a polynomial.
 *
 * @param poly  The given polynomial
 * @param p     The given point
 * @return      The horizontal distance between the polynomial and the point
 */
int cLaneDetectionFu::horizDistanceToPolynomial(NewtonPolynomial& poly, FuPoint<int> &p) {
    double pY = p.getY();
    double pX = p.getX();

    double polyY = poly.at(pX);
    double distance = std::abs(pY - polyY);

    return distance;
}

/**
 * Method, that checks if a point lies within the default ROI of a position.
 *
 * @param position  The position of the default ROI
 * @param p         The given point, which is checked
 * @return          True, if the point lies within the default ROI
 */
bool cLaneDetectionFu::isInDefaultRoi(ePosition position, FuPoint<int> &p) {
    if (p.getX() < minXRoi || p.getX() > maxXDefaultRoi) {
        return false;
    }
    else if (horizDistanceToDefaultLine(position, p)
            <= interestDistanceDefault) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * Method, that checks if a point lies within the the ROI of a polynomial.
 *
 * @param poly      The polynomial, whose ROI is used
 * @param p         The point, which is checked
 * @return          True, if the point lies within the polynomial's ROI
 */
bool cLaneDetectionFu::isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p) {
    if (p.getX() < minXRoi || p.getX() > maxXPolyRoi) {
        return false;
    }
    else if (horizDistanceToPolynomial(poly, p) <= interestDistancePoly) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * Calculates the horizontal distance between two points.
 *
 * @param p1    The first point
 * @param p2    The second point
 * @return      The horizontal distance between the two points
 */
int cLaneDetectionFu::horizDistance(FuPoint<int> &p1, FuPoint<int> &p2) {
    double y1 = p1.getY();
    double y2 = p2.getY();

    return std::abs(y1 - y2);
}

/**
 * Calculates the gradient of a polynomial at a given x value. The used formula
 * was obtained by the following steps:
 * - start with the polynomial of 2nd degree in newton basis form:
 *   p(x) = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 * - expand the equation and sort it by descending powers of x
 * - form the first derivative
 *
 * Applying the given x value then results in the wanted gradient.
 *
 * @param x         The given x value
 * @param points    The data points used for interpolating the polynomial
 * @param coeffs    The coefficients under usage of the newton basis
 * @return          The gradient of the polynomial at x
 */
double cLaneDetectionFu::gradient(double x, std::vector<FuPoint<int>> &points,
        std::vector<double> coeffs) {
    return 2 * coeffs[2] * x + coeffs[1] - coeffs[2] * points[1].getX()
            - coeffs[2] * points[0].getX();
}

/**
 * Calculates the x value of the point where the normal of the tangent of a
 * polynomial at a given point p intersects with a second polynomial.
 *
 * The formula for the intersection point is obtained by setting equal the
 * following two formula:
 *
 * 1. the formula of the normal in point-slope-form:
 *     y - p_y = -(1 / m) * (x - p_x) which is the same as
 *           y = -(x / m) + (p_x / m) + p_y
 *
 * 2. the formula of the second polynomial of 2nd degree in newton basis form:
 *           y = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 *
 * Expanding everything and moving it to the right side gives a quadratic
 * equation in the general form of 0 = ax^2 + bx + c, which can be solved using
 * the general quadratic formula x = (-b +- sqrt(b^2 - 4ac)) / 2a
 *
 * The three cases for the discriminant are taken into account.
 *
 * @param p         The point of the first poly at which its tangent is used
 * @param m         The gradient of the tangent
 * @param points    The data points used for interpolating the second polynomial
 * @param coeffs    The coeffs of the second polynomial with newton basis
 * @return          The x value of the intersection point of normal and 2nd poly
 */
double cLaneDetectionFu::intersection(FuPoint<double> &p, double &m,
        std::vector<FuPoint<int>> &points, std::vector<double> &coeffs) {
    double a = coeffs[2];
    double b = coeffs[1] - (coeffs[2] * points[1].getX())
            - (coeffs[2] * points[0].getX()) + (1.0 / m);
    double c = coeffs[0] - (coeffs[1] * points[0].getX())
            + (coeffs[2] * points[0].getX() * points[1].getX())
            - p.getY() - (p.getX() / m);

    double dis = std::pow(b, 2) - (4 * a * c);
    double x1 = 0;
    double x2 = 0;

    if (dis < 0) {
        return -1;
    }
    else if (dis == 0) {
        return -b / (2 * a);
    }
    else {
        x1 = (-b + std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
        x2 = (-b - std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    return fmax(x1, x2);
}

/**
 * Calculates the gradient of a second polynomial at the point, at which the
 * normal of the tangent of the first polynomial at the given point
 * intersects with the second polynomial.
 *
 * @param x         The given x value of the point on the first polynomial
 * @param poly1     The first polynomial
 * @param points1   The data points used for interpolating the first poly
 * @param points2   The data points used for interpolating the second poly
 * @param coeffs1   The coeffs of the first poly using newton basis
 * @param coeffs2   The coeffs of the second poly using newton basis
 * @param m1        The gradient of the first poly at x
 * @return          The gradient of the second poly at the intersection point
 */
double cLaneDetectionFu::nextGradient(double x, NewtonPolynomial &poly1,
        std::vector<FuPoint<int>> &points1, std::vector<FuPoint<int>> &points2,
        std::vector<double> coeffs1, std::vector<double> coeffs2, double m1) {

    FuPoint<double> p = FuPoint<double>(x, poly1.at(x));
    double x2 = intersection(p, m1, points2, coeffs2);

    return gradient(x2, points2, coeffs2);
}

/**
 * Check two gradients for similarity. Return true if the difference in degree
 * is less than 10.
 *
 * @param m1    The first gradient
 * @param m2    The second gradient
 * @return      True, if the diffenence between the gradients is less than 10°
 */
bool cLaneDetectionFu::gradientsSimilar(double &m1, double &m2) {
    double a1 = atan(m1) * 180 / PI;
    double a2 = atan(m2) * 180 / PI;

    if (abs(a1 - a2) < 10) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * Finds the position of the polynomial with the highest proportion.
 * @return The position of the best polynomial
 */
ePosition cLaneDetectionFu::maxProportion() {
    ePosition maxPos = LEFT;
    double maxVal = bestPolyLeft.second;

    if (bestPolyCenter.second > maxVal) {
        maxPos = CENTER;
        maxVal = bestPolyCenter.second;
    }

    if (bestPolyRight.second > maxVal) {
        maxPos = RIGHT;
    }

    return maxPos;
}

/**
 * Create the lane polynomial starting from the detected polynomial of the
 * given position. A lane polynomial is formed by shifting points with
 * different x-values of the used polynomial along the normals of the polynomial
 * at this points to the distance, where the respective lane polynomial is
 * expected to lie.
 *
 * @param position  The position of the detected polynomial used as reference
 */
void cLaneDetectionFu::createLanePoly(ePosition position) {
    lanePoly.clear();

    double x1 = 0.05;
    double x2 = 0.4;
    double x3 = 1.0;

    FuPoint<double> pointRight1;
    FuPoint<double> pointRight2;
    FuPoint<double> pointRight3;

    FuPoint<double> pointLeft1;
    FuPoint<double> pointLeft2;
    FuPoint<double> pointLeft3;

    double m1 = 0;
    double m2 = 0;
    double m3 = 0;

    double dRight = 0;

    NewtonPolynomial usedPoly;

    /*
     * Depending on the sign of the gradient of the poly at the different
     * x-values and depending on which position we are, we have to add or
     * subtract the expected distance to the respective lane polynomial, to get
     * the wanted points.
     *
     * The calculation is done for the x- and y-components of the points
     * separately using the trigonometric ratios of right triangles and the fact
     * that arctan of some gradient equals its angle to the x-axis in degree.
     */
    if (position == LEFT) {
        usedPoly = polyLeft;
        m1 = gradient(x1, pointsLeft, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsLeft, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsLeft, usedPoly.getCoefficients());

        dRight = defaultYLeft;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
    }
    else if (position == CENTER) {
        usedPoly = polyCenter;
        m1 = gradient(x1, pointsCenter, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsCenter, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsCenter, usedPoly.getCoefficients());

        dRight = defaultYCenter;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
    }
    else if (position == RIGHT) {
        usedPoly = polyRight;
        m1 = gradient(x1, pointsRight, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsRight, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsRight, usedPoly.getCoefficients());

        dRight = defaultYCenter;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
    }

    // create the lane polynomial out of the shifted points
    lanePoly.addData(pointRight1);
    lanePoly.addData(pointRight2);
    lanePoly.addData(pointRight3);

    lanePolynomial.setLanePoly(lanePoly);
    lanePolynomial.setDetected();
}

/**
 * Decide, which of the detected polynomials (if there are any) should be used
 * as reference for creating the lane polynomials.
 *
 * @param startX    The x-value, starting from which we compare the detected polys
 */
void cLaneDetectionFu::detectLane(int startX) {
    if (polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight) {
        createLanePoly(LEFT);
    }
    else if (!polyDetectedLeft && polyDetectedCenter && !polyDetectedRight) {
        createLanePoly(CENTER);
    }
    else if (!polyDetectedLeft && !polyDetectedCenter && polyDetectedRight) {
        createLanePoly(RIGHT);
    }
    else if (polyDetectedLeft && polyDetectedCenter && !polyDetectedRight) {
        double gradLeft = gradient(startX, pointsLeft,
                polyLeft.getCoefficients());

        double gradCenter = nextGradient(startX, polyLeft, pointsLeft,
                pointsCenter, polyLeft.getCoefficients(),
                polyCenter.getCoefficients(), gradLeft);

        if (gradientsSimilar(gradLeft, gradCenter)) {
            createLanePoly(LEFT);
        }
        else {
            if (bestPolyLeft.second >= bestPolyCenter.second) {
                createLanePoly(LEFT);
            }
            else {
                createLanePoly(CENTER);
            }
        }
    }
    else if (!polyDetectedLeft && polyDetectedCenter && polyDetectedRight) {
        double gradCenter = gradient(startX, pointsCenter,
                polyCenter.getCoefficients());

        double gradRight = nextGradient(startX, polyCenter, pointsCenter,
                pointsRight, polyCenter.getCoefficients(),
                polyRight.getCoefficients(), gradCenter);

        if (gradientsSimilar(gradCenter, gradRight)) {
            createLanePoly(RIGHT);
        }
        else {
            if (bestPolyCenter.second >= bestPolyRight.second) {
                createLanePoly(CENTER);
            }
            else {
                createLanePoly(RIGHT);
            }
        }
    }
    else if (polyDetectedLeft && !polyDetectedCenter && polyDetectedRight) {
        double gradLeft = gradient(startX, pointsLeft,
                polyLeft.getCoefficients());

        double gradRight = nextGradient(startX, polyLeft, pointsLeft,
                pointsRight, polyLeft.getCoefficients(),
                polyRight.getCoefficients(), gradLeft);

        if (gradientsSimilar(gradLeft, gradRight)) {
            createLanePoly(LEFT);
        }
        else {
            if (bestPolyLeft.second >= bestPolyRight.second) {
                createLanePoly(LEFT);
            }
            else {
                createLanePoly(RIGHT);
            }
        }
    }
    else if (polyDetectedLeft && polyDetectedCenter && polyDetectedRight) {
        double gradLeft = gradient(startX, pointsLeft,
                polyLeft.getCoefficients());

        double gradCenter2 = gradient(startX, pointsCenter,
                polyCenter.getCoefficients());

        double gradCenter1 = nextGradient(startX, polyLeft, pointsLeft,
                pointsCenter, polyLeft.getCoefficients(),
                polyCenter.getCoefficients(), gradLeft);

        double gradRight1 = nextGradient(startX, polyLeft, pointsLeft,
                pointsRight, polyLeft.getCoefficients(),
                polyRight.getCoefficients(), gradLeft);

        double gradRight2 = nextGradient(startX, polyCenter, pointsCenter,
                pointsRight, polyCenter.getCoefficients(),
                polyRight.getCoefficients(), gradCenter2);

        if (gradientsSimilar(gradLeft, gradCenter1)) {
            if (gradientsSimilar(gradCenter1, gradRight1)) {
                createLanePoly(LEFT);
            }
            else {
                createLanePoly(LEFT);
            }
        }
        else {
            if (gradientsSimilar(gradCenter2, gradRight2)) {
                createLanePoly(RIGHT);
            }
            else {
                ePosition maxPos = maxProportion();
                if (maxPos == LEFT) {
                    createLanePoly(LEFT);
                }
                else if (maxPos == CENTER) {
                    createLanePoly(CENTER);
                }
                else if (maxPos == RIGHT) {
                    createLanePoly(RIGHT);
                }
            }
        }
    }
    else if (!polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight) {
        lanePoly.clear();

        lanePolynomial.setNotDetected();
    }
}

/**
 * Starts the RANSAC algorithm for detecting each of the three lane marking
 * polynomials.
 */
void cLaneDetectionFu::ransac() {
    polyDetectedLeft = ransacInternal(LEFT, laneMarkingsLeft, bestPolyLeft,
            polyLeft, supportersLeft, prevPolyLeft, pointsLeft);

    polyDetectedCenter = ransacInternal(CENTER, laneMarkingsCenter,
            bestPolyCenter, polyCenter, supportersCenter, prevPolyCenter,
            pointsCenter);

    polyDetectedRight = ransacInternal(RIGHT, laneMarkingsRight, bestPolyRight,
            polyRight, supportersRight, prevPolyRight, pointsRight);
}

/**
 * Detects a polynomial with RANSAC in a given list of lane marking edge points.
 *
 * @param position      The position of the wanted polynomial
 * @param laneMarkings  A reference to the list of lane marking edge points
 * @param bestPoly      A reference to a pair containing the present best
 *                      detected polynomial and a value representing the fitting
 *                      quality called proportion
 * @param poly          A reference to the polynomial that gets detected
 * @param supporters    A reference to the supporter points of the present best
 *                      polynomial
 * @param prevPoly      A reference to the previous polynomial detected at this
 *                      position
 * @param points        A reference to the points selected for interpolating the
 *                      present best polynomial
 * @return              true if a polynomial could be detected and false when not
 */
bool cLaneDetectionFu::ransacInternal(ePosition position,
        std::vector<FuPoint<int>>& laneMarkings,
        std::pair<NewtonPolynomial, double>& bestPoly, NewtonPolynomial& poly,
        std::vector<FuPoint<int>>& supporters, NewtonPolynomial& prevPoly,
        std::vector<FuPoint<int>>& points) {

    if (laneMarkings.size() < 7) {
        return false;
    }

    int iterations = 0;

    // sort the lane marking edge points
    std::vector<FuPoint<int>> sortedMarkings = laneMarkings;

    std::sort(sortedMarkings.begin(), sortedMarkings.end(),
            [](FuPoint<int> a, FuPoint<int> b) {
                return a.getX() < b.getX();
            });

    std::vector<FuPoint<int>> tmpSupporters = std::vector<FuPoint<int>>();

    // vectors for points selected from the bottom, mid and top of the sorted
    // point vector
    std::vector<FuPoint<int>> markings1 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings2 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings3 = std::vector<FuPoint<int>>();

    bool highEnoughX = false;

    // Points are selected from the bottom, mid and top. The selection regions
    // are spread apart for better results during RANSAC
    for (std::vector<FuPoint<int>>::size_type i = 0; i != sortedMarkings.size();
            i++) {
        if (i < double(sortedMarkings.size()) / 7) {
            markings1.push_back(sortedMarkings[i]);
        }
        else if (i >= (double(sortedMarkings.size()) / 7) * 3
                && i < (double(sortedMarkings.size()) / 7) * 4) {
            markings2.push_back(sortedMarkings[i]);
        }
        else if (i >= (double(sortedMarkings.size()) / 7) * 6) {
            markings3.push_back(sortedMarkings[i]);
        }

        if (sortedMarkings[i].getX() > 0.5) {
            highEnoughX = true;
        }
    }

    if (position == CENTER) {
        if (!highEnoughX) {
            prevPoly = poly;
            poly.clear();
            return false;
        }
    }

    // save the polynomial from the previous picture
    prevPoly = poly;

    while (iterations < iterationsRansac) {
        iterations++;

        // randomly select 3 different lane marking points from bottom, mid and
        // top
        int pos1 = rand() % markings1.size();
        int pos2 = rand() % markings2.size();
        int pos3 = rand() % markings3.size();

        FuPoint<int> p1 = markings1[pos1];
        FuPoint<int> p2 = markings2[pos2];
        FuPoint<int> p3 = markings3[pos3];

        double p1X = p1.getX();
        double p1Y = p1.getY();
        double p2X = p2.getX();
        double p2Y = p2.getY();
        double p3X = p3.getX();
        double p3Y = p3.getY();

        // clear poly for reuse
        poly.clear();

        // create a polynomial with the selected points
        poly.addData(p1X, p1Y);
        poly.addData(p2X, p2Y);
        poly.addData(p3X, p3Y);

        // check if this polynomial is not useful
        if (!polyValid(position, poly, prevPoly)) {
            poly.clear();
            continue;
        }

        // count the supporters and save them for debugging
        int count1 = 0;
        int count2 = 0;
        int count3 = 0;

        // find the supporters
        tmpSupporters.clear();

        for (FuPoint<int> p : markings1) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count1++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings2) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count2++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings3) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count3++;
                tmpSupporters.push_back(p);
            }
        }

        if (count1 == 0 || count2 == 0 || count3 == 0) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly had no supporters in one of the regions");
            continue;
        }

        // calculate the proportion of supporters of all lane markings
        double proportion = (double(count1) / markings1.size()
                + double(count2) / markings2.size()
                + 3 * (double(count3) / markings3.size())) / 5;

        if (proportion < proportionThreshould) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly proportion was smaller than threshold");
            continue;
        }

        // check if poly is better than bestPoly
        if (proportion > bestPoly.second) {
            bestPoly = std::make_pair(poly, proportion);
            supporters = tmpSupporters;

            points.clear();
            points.push_back(p1);
            points.push_back(p2);
            points.push_back(p3);
        }
    }

    poly = bestPoly.first;

    if (poly.getDegree() == -1) {
        return false;
    }

    //DEBUG_TEXT(dbgValues, "best poly proportion: %f", bestPoly.second);

    return true;
}

/**
 * Method for drawing the default lines in furemote
 */
/*void cLaneDetectionFu::drawDefaultLines(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        int r, int g, int b) {
    // left line
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi), Centimeter(defaultYLeft),
                    Centimeter(maxXDefaultRoi), Centimeter(defaultYLeft));
    });

    // center line
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi), Centimeter(defaultYCenter),
                    Centimeter(maxXDefaultRoi), Centimeter(defaultYCenter));
    });

    // right line
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi), Centimeter(defaultYRight),
                    Centimeter(maxXDefaultRoi), Centimeter(defaultYRight));
    });
}*/

/**
 * Method for drawing the ROIs around the default lines
 */
/*void cLaneDetectionFu::drawDefaultRois(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        int r, int g, int b) {
    // left roi
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYLeft - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYLeft - interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYLeft - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYLeft + interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYLeft + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYLeft + interestDistanceDefault));
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYLeft + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYLeft - interestDistanceDefault));
    });

    // center roi
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYCenter - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYCenter - interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYCenter - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYCenter + interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYCenter + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYCenter + interestDistanceDefault));
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYCenter + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYCenter - interestDistanceDefault));
    });

    // right roi
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYRight - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYRight - interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYRight - interestDistanceDefault),
                    Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYRight + interestDistanceDefault));
            LINE(Centimeter(maxXDefaultRoi),
                    Centimeter(defaultYRight + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYRight + interestDistanceDefault));
            LINE(Centimeter(minXRoi),
                    Centimeter(defaultYRight + interestDistanceDefault),
                    Centimeter(minXRoi),
                    Centimeter(defaultYRight - interestDistanceDefault));
    });
}*/

/**
 * Method for drawing a detected polynomial
 */
/*void cLaneDetectionFu::drawPoly(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        NewtonPolynomial& poly, int r, int g, int b) {
    std::vector<double> xCoordinates = produceXCoordinates(maxXPolyRoi);
    std::vector<double> polyYCoordinates = getPolyYCoordinates(xCoordinates,
            poly);

    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            for (unsigned int i = 0; i < xCoordinates.size(); i++) {
                CIRCLE(Centimeter(xCoordinates[i] * meters),
                        Centimeter(polyYCoordinates[i] * meters),
                        1 * centimeters);
            }
    });
}*/

/**
 * Method for drawing the ROI around a detected polynomial
 */
/*void cLaneDetectionFu::drawPolyRoi(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        NewtonPolynomial& poly, int r, int g, int b) {
    std::vector<double> xCoordinates = produceXCoordinates(maxXPolyRoi);
    std::vector<double> polyYCoordinates = getPolyYCoordinates(xCoordinates,
            poly);

    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            for (unsigned int i = 0; i < xCoordinates.size(); i++) {
                CIRCLE(Centimeter(xCoordinates[i] * meters),
                        Centimeter((polyYCoordinates[i]
                                - interestDistancePoly.value()) * meters),
                                1 * centimeters);
                CIRCLE(Centimeter(xCoordinates[i] * meters),
                        Centimeter((polyYCoordinates[i]
                                + interestDistancePoly.value()) * meters),
                                1 * centimeters);
            }
    });

}

void cLaneDetectionFu::writeProportions() {
    DRAWDEBUG(dbgProportions, {
            SETCOLORWHITE;
            TEXT(-10 * centimeters, Centimeter(defaultYLeft),
                    std::to_string(bestPolyLeft.second));
            TEXT(-10 * centimeters, Centimeter(defaultYCenter),
                    std::to_string(bestPolyCenter.second));
            TEXT(-10 * centimeters, Centimeter(defaultYRight),
                    std::to_string(bestPolyRight.second));
    });
}*/

/**
 * Calculates a vector of y-values of the given polynomial at the x-values given
 * as vector. The "at"-method of the NewtonPolynomial class is used, which
 * relies on the horner scheme.
 *
 * @param xCoordinates  The vector of x-values
 * @param poly          The given polynomial
 * @return              The vector of calculated y-values.
 */
/*std::vector<double> cLaneDetectionFu::getPolyYCoordinates(
        std::vector<double> xCoordinates, NewtonPolynomial& poly) {
    std::vector<double> yCoordinates;
    double yValue;

    for (double xValue : xCoordinates) {
        yValue = poly.at(xValue);
        yCoordinates.push_back(yValue);
    }

    return yCoordinates;
}*/

/**
 * Create a vector of ascending x-values starting with the minimal x-value of
 * the ROIs and ending with the x-value distance. The x-values are 0.05 apart.
 *
 * @param distance  x-values smaller than this are added
 * @return          The vector of x-values
 */
/*std::vector<double> cLaneDetectionFu::produceXCoordinates(Meter distance) {
    std::vector<double> xCoordinates;
    double x = minXRoi.value();

    while (x < distance.value()) {
        xCoordinates.push_back(x);
        x += 0.05;
    }

    return xCoordinates;
}*/

/**
 * Method for drawing the supporters of a detected polynomial
 */
/*void cLaneDetectionFu::drawSupporters(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        std::vector<Point<Meter>> supporters, int r, int g, int b) {
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            for (Point<Meter> p : supporters) {
                CIRCLE(Centimeter(p.x), Centimeter(p.y),
                        0.5 * centimeters);
            }
    });
}*/

/**
 * Method for drawing the lane marking points used to detect a polynomial
 */
/*void cLaneDetectionFu::drawLaneMarkings(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        std::vector<Point<Meter>> laneMarkings, int r, int g, int b) {
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            for (Point<Meter> p : laneMarkings) {
                CIRCLE(Centimeter(p.x), Centimeter(p.y),
                        2 * centimeters);
            }
    });
}*/

/**
 * Method for drawing the points selected to interpolate the polynomial
 */
/*void cLaneDetectionFu::drawPoints(
        std::shared_ptr<DebuggingOption<DRAWING_RELATIVE>> debugTarget,
        std::vector<Point<Meter>> points, int r, int g, int b) {
    DRAWDEBUG(debugTarget, {
            SETCOLOR(r, g, b);
            for (Point<Meter> p : points) {
                CIRCLE(Centimeter(p.x), Centimeter(p.y),
                        2.5 * centimeters);
            }
    });
}*/

/**
 * Method, that checks, if a polynomial produced during RANSAC counts as usable.
 *
 * @param position  The position of the polynomial, that is checked
 * @param poly      The polynomial, that is checked
 * @param prevPoly  The previous polynomial detected at this position
 * @return          True, if the polynomial counts as valid
 */
bool cLaneDetectionFu::polyValid(ePosition position, NewtonPolynomial poly,
        NewtonPolynomial prevPoly) {

    FuPoint<int> p1 = FuPoint<int>(3, poly.at(3));

    if (horizDistanceToDefaultLine(position, p1) > 5) {
        //DEBUG_TEXT(dbgMessages, "Poly was to far away from default line at x = 3");
        return false;
    }

    FuPoint<int> p2 = FuPoint<int>(6, poly.at(6));

    if (horizDistanceToDefaultLine(position, p2) > 8) {
        //DEBUG_TEXT(dbgMessages, "Poly was to far away from default line at x = 6");
        return false;
    }

    FuPoint<int> p3 = FuPoint<int>(10, poly.at(10));

    if (horizDistanceToDefaultLine(position, p3) > 10) {
        //DEBUG_TEXT(dbgMessages, "Poly was to far away from default line at x = 10");
        return false;
    }

    if (prevPoly.getDegree() != -1) {
        FuPoint<int> p4 = FuPoint<int>(5, poly.at(5));
        FuPoint<int> p5 = FuPoint<int>(5, prevPoly.at(5));

        if (horizDistance(p4, p5) > 1) {//0.05 * meters) {
            //DEBUG_TEXT(dbgMessages, "Poly was to far away from previous poly at x = 0.5");
            return false;
        }

        FuPoint<int> p6 = FuPoint<int>(3, poly.at(3));
        FuPoint<int> p7 = FuPoint<int>(3, prevPoly.at(3));

        if (horizDistance(p6, p7) > 1) {//0.05 * meters) {
            //DEBUG_TEXT(dbgMessages, "Poly was to far away from previous poly at 0.3");
            return false;
        }
    }

    return true;
}










/*void cLaneDetectionFu::pubRealSenseRGBImageMsg(cv::Mat& rgb_mat)
{
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);
*/
    /*rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;*/

    //realsense_rgb_image_pub.publish(rgb_img, rgb_camera_info);


    //save rgb img
//  static int count = 0;
//  count++;
//  if(count > 0)
//  {
//      struct timeval save_time;
//        gettimeofday( &save_time, NULL );
//        char save_name[256];
//        sprintf(save_name, "~/temp/realsense_rgb_%d.jpg", (int)save_time.tv_sec);
//        printf("\nsave realsense rgb img: %s\n", save_name);
//      cv::imwrite(save_name, rgb_mat);
//      count = 0;
//  }
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;

    //std::string node_name = "/line_detection_node/";
    std::string node_name = ros::this_node::getName();

    ROS_ERROR("Node name: %s",node_name.c_str());


    int cam_w;
    int cam_h;
    int proj_y_start;
    int proj_image_h;
    int proj_image_w;
    int roi_top_w;
    int roi_bottom_w;
    int proj_image_horizontal_offset;
    int detector_size;
    int lane_width;
    /*std::string path_2features;
    std::string path_30features;*/

    //nh.param<std::string>("camera_name", camera_name, "/usb_cam/image_raw"); 
    nh.param<int>("cam_w", cam_w, 640);
    nh.param<int>("cam_h", cam_h, 480);
    nh.param<int>(node_name+"/proj_y_start", proj_y_start, 400);
    nh.param<int>(node_name+"/proj_image_h", proj_image_h, 40);
    nh.param<int>(node_name+"/proj_image_w", proj_image_w, 80);
    nh.param<int>(node_name+"/roi_top_w", roi_top_w, 40);
    nh.param<int>(node_name+"/roi_bottom_w", roi_bottom_w, 40);
    nh.param<int>(node_name+"/proj_image_horizontal_offset", proj_image_horizontal_offset, 0);
    /*nh.param<std::string>(node_name+"/path_2features", path_2features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_2features.txt");
    nh.param<std::string>(node_name+"/path_30features", path_30features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_30features.txt");
*/
    cLaneDetectionFu node(nh, cam_w, cam_h, proj_y_start, proj_image_h, proj_image_w,
        roi_top_w, roi_bottom_w, proj_image_horizontal_offset);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
