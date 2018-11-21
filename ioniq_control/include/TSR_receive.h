#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//#include <ros/ros.h>
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>

typedef struct TSR
{
    int data_class;
    float data_probability;
    unsigned int data_xmin;
    unsigned int data_ymin;
    unsigned int data_xmax;
    unsigned int data_ymax;
    bool data_search_flag;
    float roi_size;
} TSR;
extern TSR *tsr;
extern int result_num; //

//int sel_biggest_one(TSR *array_, float count_);

//extern int TSR_seq;

//free(tsr);

//void TSR_MsgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
//    //ROS_INFO("Data:%d, %d\n", msg->header.stamp.sec, msg->bounding_boxes[0].xmin);

//    TSR_seq = msg->header.seq;
//    // The number of Bounding boxes
//    int nboxes = msg->bounding_boxes.size();
//    //std::cout << nboxes << std::endl;

//    if(nboxes > 0)
//    {
//    std::cout << "---------------------------------------" << std::endl;
//    std::cout << "nboxes = " << nboxes << std::endl;
//    //std::cout << "Bounding_boxes : " << nboxes << std::endl;
//        for(int i = 0; i<msg->bounding_boxes.size(); i++)
//        {

//        if(("TR_3_R" == msg->bounding_boxes[i].Class) || ("TR_4_R" == msg->bounding_boxes[i].Class)) tsr[i].data_class = 0;
//        if(("TR_3_Y" == msg->bounding_boxes[i].Class) || ("TR_4_Y" == msg->bounding_boxes[i].Class)) tsr[i].data_class = 1;
//        if(("TR_3_G" == msg->bounding_boxes[i].Class) || ("TR_4_G" == msg->bounding_boxes[i].Class)) tsr[i].data_class = 2;
//        if("TR_4_LG" == msg->bounding_boxes[i].Class) tsr[i].data_class = 3;
			
//        // To get ROS msg data to TSR struct
//        tsr[i].data_probability = msg->bounding_boxes[i].probability;
//        tsr[i].data_xmin = msg->bounding_boxes[i].xmin;
//        tsr[i].data_ymin = msg->bounding_boxes[i].ymin;
//        tsr[i].data_xmax = msg->bounding_boxes[i].xmax;
//        tsr[i].data_ymax = msg->bounding_boxes[i].ymax;
//        tsr[i].data_search_flag = msg->bounding_boxes[i].search_flag;

//        // ROI size calc
//        int tmp_w_ = tsr[i].data_xmax - tsr[i].data_xmin;
//        int tmp_h_ = tsr[i].data_ymax - tsr[i].data_ymin;
//        tmp_w_ = tmp_w_>=0 ? tmp_w_:0;
//        tmp_h_ = tmp_h_>=0 ? tmp_h_:0;

//        tsr[i].roi_size = tmp_w_ * tmp_h_;

//        std::cout << "Search flag : " <<tsr[i].data_search_flag << std::endl;
//#ifdef _PRINT_
//        //std::cout << fixed;
//        std::cout.precision(4);
//            std::cout << "Bounding_boxes[" << i << "] :" << tsr[i].data_class << "\t\t" << tsr[i].data_probability << "\t\t" << tsr[i].data_xmin << "\t\t" << tsr[i].data_xmax << "\t\t" << tsr[i].data_ymin <<  "\t\t" << tsr[i].data_ymax << "\t\t" << tsr[i].roi_size << std::endl;
//#endif

//    }

//    int biggest_roi_size_num = sel_biggest_one(tsr, nboxes);
//    result_num = biggest_roi_size_num;

//#ifdef _PRINT_
//    std::cout << "Biggest one is " << result_num << std::endl;
//    std::cout << "Classifier result :[" << result_num << "] :" << tsr[result_num].data_class << "\t\t" << tsr[result_num].data_probability << "\t\t" << tsr[result_num].data_xmin << "\t\t" << tsr[result_num].data_xmax << "\t\t" << tsr[result_num].data_ymin <<  "\t\t" << tsr[result_num].data_ymax << "\t\t" << tsr[result_num].roi_size << std::endl;
//#endif
//    }
//    else std::cout << "No BBoxes" << std::endl; // No!
    

//}





//int sel_biggest_one(TSR *array_, float count_)
//{
//    //int tmp = array_[0];
//    int tmp = 0;
//    for (int i = 0; i < count_; i++)
//    {
//        tmp = (array_[tmp].roi_size > array_[i].roi_size) ? tmp:i;
//    }
//    return tmp;
//}

