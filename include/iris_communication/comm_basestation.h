#ifndef COMM_BASESTATION_H_
#define COMM_BASESTATION_H_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include "iris_its/config.h"
#include "iris_its/BasestationRx.h"
#include "iris_its/BasestationTx.h"

#define BUFFER_LENGTH 312

class CommBasestation
{
private:
    std::string MODE_COMM;
    std::string CLIENT_IP;
    uint16_t CLIENT_PORT;
    std::string SERVER_IP;
    uint16_t SERVER_PORT;

    ros::Subscriber sub_basestation;
    ros::Publisher pub_basestation;

    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    struct sockaddr_in dummy_addr;
    socklen_t dummy_socklen;
    //-----MULTICAST
    struct ip_mreq server_mreq;

    int sockfd_rx;
    int sockfd_tx;

    char nomor_robot;

    char tx_len, tx_buffer[BUFFER_LENGTH];
    char rx_len, rx_buffer[BUFFER_LENGTH];

public:
    CommBasestation(ros::NodeHandle *NH);
    ~CommBasestation();

    void loadConfig();
    void loop();

    void callbackSubBasestation(const iris_its::BasestationTxConstPtr &msg);
};

#endif