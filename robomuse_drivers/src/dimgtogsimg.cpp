#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <std_msgs/UInt8.h>

using namespace std;
using namespace ros;

class ImageConverter
{
        NodeHandle gdd;
        Subscriber depth_sub;
        Publisher gray_pub;

        public:
        void imageCb(const sensor_msgs::Image::ConstPtr& );

        ImageConverter()
        {

                depth_sub = gdd.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);
                gray_pub = gdd.advertise<sensor_msgs::Image>("depthTOgreyscale_image", 1);                
        }


};

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& depth_image)
  {
	depth_image->width = 480;
	depth_image->height = 640;

        sensor_msgs::Image gray_image;       
        gray_image.header.seq = depth_image->header.seq;
        gray_image.header.stamp = depth_image->header.stamp;
        gray_image.header.frame_id = "/camera_rgb_optical_frame";
        gray_image.height = 480;
        gray_image.width = 640;
        gray_image.encoding = "mono8";
        gray_image.is_bigendian = 0;
        gray_image.step = 640;

        float max_depth = 0;
        for(int i=0; i<depth_image->height; i++)
        {
                for(int j=0; j<depth_image->width; j++)
                {

                        float distance = depth_image->data[i*depth_image->width+j];
                        if (distance == distance) { 
                            max_depth = max(distance, max_depth);
                        }

                }

        }
        ROS_INFO("%f", max_depth);
       //ACTUAL CONVERSION             
       for(int i=0; i<depth_image->height; i++)
        {
                for(int j=0; j<depth_image->width; j++)
                {
                       gray_image.data[i*depth_image->width+j] = (unsigned int)(depth_image->data[i*depth_image->width+j]/max_depth*255);    
                }

        }

        gray_pub.publish(depth_image);

}


int main(int argc, char **argv)
{
        init(argc, argv, "get_depth_data");
        ImageConverter ic;               
        spin();    
        return 0;
}
