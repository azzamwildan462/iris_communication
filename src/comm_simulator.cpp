#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "iris_its/VisionBall.h"
#include "iris_its/VisionObstacle.h"

//-----Publisher
ros::Publisher pub_velocity;
ros::Publisher pub_kicker;
ros::Publisher pub_odometry_buffer;
ros::Publisher pub_handle_ball;
ros::Publisher pub_vision_ball;
ros::Publisher pub_vision_obstacle;

float pos_x_buffer = 0;
float pos_y_buffer = 0;
float theta_buffer = 0;

//-----Velocity
short int kecepatan_x;
short int kecepatan_y;
short int kecepatan_sudut;

//-----Bola dribble
unsigned char status_bola_dribble;

//-----Bola vision
unsigned char status_bola_vision;

short int bola_x_pada_frame;
short int bola_y_pada_frame;
short int bola_theta_pada_frame;

short int bola_x_pada_lapangan;
short int bola_y_pada_lapangan;
short int bola_theta_pada_lapangan;

//-----Penendang
bool reset_kicker = true;
unsigned char mode_tendang = 0;
unsigned int kecepatan_tendang = 1500;

//-----Obstacle
unsigned short int halangan[144];

void cllbck_sub_velocity(const geometry_msgs::TwistConstPtr &msg)
{
    kecepatan_x = msg->linear.x;
    kecepatan_y = msg->linear.y;
    kecepatan_sudut = msg->angular.z;
}

void cllbck_sub_kicker(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    mode_tendang = msg->data[0];
    kecepatan_tendang = msg->data[1];
    std_msgs::Int16MultiArray msg_kicker;
    msg_kicker.data.push_back(mode_tendang);
    msg_kicker.data.push_back(kecepatan_tendang);
    pub_kicker.publish(msg_kicker);
}

void cllbck_sub_odometry(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    pos_x_buffer = msg->x;
    pos_y_buffer = msg->y;
    theta_buffer = msg->theta;
}

void cllbck_sub_handle_ball(const std_msgs::UInt8ConstPtr &msg)
{
    status_bola_dribble = msg->data;
}

void cllbck_sub_vision_ball(const iris_its::VisionBallConstPtr &msg)
{
    status_bola_vision = msg->status_bola;
    bola_x_pada_frame = msg->bola_x_pada_frame;
    bola_y_pada_frame = msg->bola_y_pada_frame;
    bola_theta_pada_frame = msg->bola_theta_pada_frame;

    bola_x_pada_lapangan = msg->bola_x_pada_lapangan;
    bola_y_pada_lapangan = msg->bola_y_pada_lapangan;
    bola_theta_pada_lapangan = msg->bola_theta_pada_lapangan;
}

void cllbck_sub_vision_obstacle(const iris_its::VisionObstacleConstPtr &msg)
{
    for(int i = 0;i < 144; i++)
        halangan[i] = msg->halangan_pada_lapangan[i];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_simulator");

    ros::NodeHandle NH;
    ros::Rate rosRate(50);

    //-----Subscriber
    ros::Subscriber sub_velocity = NH.subscribe("pc2stm/velocity", 16, cllbck_sub_velocity);
    ros::Subscriber sub_kicker = NH.subscribe("pc2stm/kicker", 16, cllbck_sub_kicker);

    ros::Subscriber sub_odometry = NH.subscribe("sim2pc/odometry", 16, cllbck_sub_odometry);
    ros::Subscriber sub_handle_ball = NH.subscribe("sim2pc/robot_handle_ball", 16, cllbck_sub_handle_ball);
    ros::Subscriber sub_vision_ball = NH.subscribe("sim2pc/robot_vision_ball_v2", 16, cllbck_sub_vision_ball);
    ros::Subscriber sub_vision_obstacle = NH.subscribe("sim2pc/robot_vision_obstacle_v2", 16, cllbck_sub_vision_obstacle);

    //-----Publisher
    pub_velocity = NH.advertise<geometry_msgs::Twist>("pc2sim/velocity", 16);
    pub_kicker = NH.advertise<std_msgs::Int16MultiArray>("pc2sim/kicker", 16);

    pub_odometry_buffer = NH.advertise<geometry_msgs::Pose2D>("stm2pc/odometry_buffer", 16);
    pub_handle_ball = NH.advertise<std_msgs::UInt8>("robot_handle_ball", 16);
    pub_vision_ball = NH.advertise<iris_its::VisionBall>("robot_vision_ball", 16);
    pub_vision_obstacle = NH.advertise<iris_its::VisionObstacle>("robot_vision_obstacle", 16);

    while(ros::ok())
    {
        ros::spinOnce();
        rosRate.sleep();

        geometry_msgs::Pose2D msg_odometry_buffer;
        msg_odometry_buffer.x = pos_x_buffer;
        msg_odometry_buffer.y = pos_y_buffer;
        msg_odometry_buffer.theta = theta_buffer;
        pub_odometry_buffer.publish(msg_odometry_buffer);

        geometry_msgs::Twist msg_velocity;
        msg_velocity.linear.x = kecepatan_x;
        msg_velocity.linear.y = kecepatan_y;
        msg_velocity.angular.z = kecepatan_sudut;
        pub_velocity.publish(msg_velocity);

        std_msgs::UInt8 msg_handle_ball;
        msg_handle_ball.data = status_bola_dribble;
        pub_handle_ball.publish(msg_handle_ball);

        iris_its::VisionBall msg_vision_ball;
        msg_vision_ball.status_bola = status_bola_vision;
        msg_vision_ball.bola_x_pada_frame = bola_x_pada_frame;
        msg_vision_ball.bola_y_pada_frame = bola_y_pada_frame;
        msg_vision_ball.bola_theta_pada_frame = bola_theta_pada_frame;
        msg_vision_ball.bola_x_pada_lapangan = bola_x_pada_lapangan;
        msg_vision_ball.bola_y_pada_lapangan = bola_y_pada_lapangan;
        msg_vision_ball.bola_theta_pada_lapangan = bola_theta_pada_lapangan;
        pub_vision_ball.publish(msg_vision_ball);

        iris_its::VisionObstacle msg_vision_obstacle;
        for(int i = 0;i < 144; i++)
        {
            msg_vision_obstacle.halangan_pada_lapangan.push_back(halangan[i]);
            msg_vision_obstacle.batas_pada_lapangan.push_back(9999);
        }
        pub_vision_obstacle.publish(msg_vision_obstacle);

    }

    return 0;
}