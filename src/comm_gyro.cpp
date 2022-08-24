// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

//  std-libraries
#include <iostream>
#include <string>
#include <regex>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <boost/algorithm/string/trim.hpp>

int serial_port;
struct termios tty;
int usbStat;
bool buttons[17];

ros::Publisher pub_theta_buffer;

std_msgs::Float32 msg_theta_buffer;

int initUSB(){    
    FILE *proc = popen("/bin/ls -l /sys/bus/usb-serial/devices","r");
    char buf[1024];
    int count = 0;
    // while ( !feof(proc) && fgets(buf,sizeof(buf),proc) )
    // {
        // std::string s(buf);
        // boost::algorithm::trim(s);
        // std::smatch sm;
        // std::regex reg(".*usb1[/]1-1[/]1-1[.]1[/]1-1[.]1[.](.)[/]1-1[.]1[.].:1[.]0[/]ttyUSB(.).*");
        // if(std::regex_match(s,sm,reg)){
        //     std::string port("/dev/ttyUSB");
        //     port += sm[2];
            // if(sm[1] == "1"){
                serial_port = open("/dev/ttyUSB3", O_RDWR);
                // std::cout << "gyro: " << port << " on port no." << sm[1] << std::endl;
            // }
        // }
    // }

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    cfsetspeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

void publish(){
    // if(!buttons[9]){
    //     unsigned char serialmsg[] = {0x68, 0x04, 0x00, 0x28, 0x2C};
    //     write(serial_port, serialmsg, sizeof(serialmsg));
    //     return;
    // }
    unsigned char serialmsg[] = {0x68, 0x04, 0x00, 0x04, 0x08};
    write(serial_port, serialmsg, sizeof(serialmsg));

    // unsigned char serialmsg2[] = {0x86, 0x05, 0x00, 0x0C, 0x05, 0x18};
    // write(serial_port, serialmsg, sizeof(serialmsg));

    char read_buf [256];
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    // std::cout << "recv data len: " << n << std::endl;    
    if(n > 0){
        int _sign, a,b,c,d,e;
        _sign = (read_buf[10]&0xFF)/16;
        if(_sign == 1 )_sign = 1;
        else _sign = -1;
        //  10 10 55 --> -010.55
        a = (read_buf[10]&0xFF)%16;
        b = (read_buf[11]&0xFF)/16%16;
        c = (read_buf[11]&0xFF)%16;
        d = (read_buf[12]&0xFF)/16%16;
        e = (read_buf[12]&0xFF)%16;
        // printf("%d %d %d\n",read_buf[10], read_buf[11], read_buf[12]);
        // printf("%x %x %x\n",read_buf[10], (read_buf[11]&0xFF), (read_buf[12]&0xFF));
        // printf("%d %d %d %d %d %d\n",_sign, a, b, c, d, e);
        // std::cout << msg_heading.data << std::endl;                       
        msg_theta_buffer.data = _sign*abs(a*100.0 + b*10.0 + c + d*0.1 + e*0.01);
        // ROS_INFO("theta_buffer %f", msg_theta_buffer.data);
        pub_theta_buffer.publish(msg_theta_buffer);
        // publish_marker(msg_heading.data);
        // msg_pose.x = 0.0;
        // msg_pose.y = 0.0;
        // msg_pose.theta = msg_heading.data;
        // pub_pose.publish(msg_pose);
    }
}


int main(int argc, char** argv){    
    ros::init(argc,  argv, "comm_gyro");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;
    ros::Rate rate(100);

    usbStat = initUSB();    
    
    pub_theta_buffer = NH.advertise<std_msgs::Float32>("gyro2pc/theta_buffer", 16);    

    while(ros::ok()){        
        // ROS_INFO("Gyro Serial Connection Status: %d", usbStat);
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}