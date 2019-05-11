//
// Created by tipakorng on 6/1/16.
//

#include <cwru_opencv_common/manual_labeling.h>

ManualLabeling::ManualLabeling(ros::NodeHandle &nodeHandle) :
nodeHandle_(nodeHandle), it(nodeHandle_), imagePt(-1,-1), imageRdy(false)
{
    manualLabelingServer_ = nodeHandle_.advertiseService("manual_labeling_service",
        &ManualLabeling::manualLabelingCallback, this);
    
    img_sub = it.subscribe("/catadioptric_play/image_raw", 1, 
        boost::function < void(const sensor_msgs::ImageConstPtr &)>
        (boost::bind(&ManualLabeling::newImageCallback, this, _1)));

    cv::namedWindow("Selectable Points");
    cv::setMouseCallback("Selectable Points", cv_ui::getCoordinates, &imagePt);
    cv::startWindowThread();
    ROS_INFO("Manual image labeling  server initialized");


}

ManualLabeling::~ManualLabeling() {
    cv::destroyWindow("Selectable Points");
}

bool ManualLabeling::manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
                                            cwru_opencv_common::image_label::Response& response)
{   
    if (!imageRdy) return false;

    // Apply adaptive threshold
    // create a click window:
    
    response.pointsResp.points.clear();
    int imageCount(0);
    // fill the blobs.
    // @Todo, add memory for the point selection.
    bool oldPtList(true);
    while (true)
    {
        cv::Mat displayImage(localImage.clone());    

        for (int ind(0); ind < ptList.points.size(); ind++)
        {
        	cv::circle(displayImage, cv::Point(ptList.points[ind].x, ptList.points[ind].y), 7, cv::Scalar(255, 0, 0), 2);
        }

        imshow("Selectable Points", displayImage);
        char keyIn = cv::waitKey(50);
        if (imagePt.x > 0)
        {
            if (oldPtList)
            {
            	oldPtList = false;
            	ptList.points.clear();
            }
            geometry_msgs::Point32 localPt;
            localPt.x = static_cast<float> (imagePt.x);
            localPt.y = static_cast<float> (imagePt.y);
            localPt.z = 0.0;
            response.pointsResp.points.push_back(localPt);
            ptList.points.push_back(localPt);
            imageCount++;
            
            imagePt.x = -1;
        }
        // if the Esc key is pressed, break out.
        if (keyIn == 27 || imageCount >= request.requestedPoints) break;
        
        // reuse the previous point list.
        if (keyIn == 'r' && oldPtList)
        {
            ROS_INFO("Re-using previous point set \n");
            for (int ind(0); ind < ptList.points.size(); ind++)
            {
                response.pointsResp.points.push_back(ptList.points[ind]);
            }
            break;
        }
    }

    ROS_INFO("Finished the acquiring the point list. \n");
    cv::Mat blank(cv::Mat::zeros(50, 50, CV_8UC1));
    imshow("Selectable Points", blank);
    cv::waitKey(10);
    // Merge blob to label image
    if (response.pointsResp.points.size() > 0)
    {
        return true;
    }
    else return false;
}


//updates the local image.
void ManualLabeling::newImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
       localImage =  cv_bridge::toCvShare(msg, "bgr8")->image.clone();
       imageRdy = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

    ROS_INFO("Finished processing the input image");
}

