#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget service;
    service.request.linear_x = lin_x;    // Pass linear velocity value of robot to ROS service
    service.request.angular_z = ang_z;  // Pass angular velocity value of robot to ROS service
    
    // If service unable to return a response or execute the request, return error message
    if(!client.call(service))
        ROS_ERROR("The ROS service 'command_robot' has failed to execute the request.");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int white_pixel = 255; // Pixel value for white color
    int white_pixel_pos; // 'X' position of pixel with white color
    int sum_white_pixel_pos = 0; // Sum 'X' position of pixel with white color
    int avg_white_pixel_pos; // Average 'X' position of pixel with white color
    int num_white_pixel = 0; // Number of pixels with white color
    int image_size = img.data.size(); // actual matrix data, size is (step * rows), step = column

    //  A digital color image pixel is just numbers representing a RGB data value (Red, Green, Blue). Each pixel's color sample has three numerical RGB components (Red, Green, Blue) to represent the color of that tiny pixel area. These three RGB components are three 8-bit numbers for each pixel. Three 8-bit bytes (one byte for each of RGB) is called 24 bit color. Each 8 bit RGB component can have 256 possible values, ranging from 0 to 255. For example, three values like (250, 165, 0), meaning (Red=250, Green=165, Blue=0) to denote one Orange pixel. 

    // Each pixel has 3 components of RGB, so for pixel in row 0, column 0, the pixel consists of i=0, i=1 and i=2 for 3 RGB components.
    // Hence the for loop steps through 3 numbers for each pixel.
    // e.g.: Row1: [[i=0,1,2],[i=3,4,5]] Row2: [[i=6,7,8],[i=9,10,11]] Row3: [[i=12,13,14],[15,16,17]]
    
    for (int i = 0; i + 2 < image_size; i+=3)
    {
        int red_component = img.data[i];
        int green_component = img.data[i+1];
        int blue_component = img.data[i+2];

        if(red_component == white_pixel && green_component == white_pixel && blue_component == white_pixel)
        {
            white_pixel_pos = (i % (img.width * 3)) / 3;
            sum_white_pixel_pos += white_pixel_pos;
            num_white_pixel++;
        }
    }

    if(num_white_pixel == 0)    // if no white ball / pixel found in image, robot stays still
    {
        drive_robot(0, 0);
    }
    else
    {
        avg_white_pixel_pos = sum_white_pixel_pos / num_white_pixel;
        if (avg_white_pixel_pos < img.width * 3 / 10)    // ball in the 3/10 section of the image (left side)
        {
            drive_robot(0.08, 0.5);
        } else if (avg_white_pixel_pos > img.width * 7 / 10)    // ball in the 3/10 section of the image (right section)
        {
            drive_robot(0.08, -0.5);
        } else  // ball in the 4/10 section of the image (middle)
        {
            drive_robot(0.08, 0);
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}