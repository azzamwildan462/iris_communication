#include "iris_communication/comm_basestation.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_basestation");
    ros::NodeHandle NH;

    CommBasestation comm_basestation = CommBasestation(&NH);

    comm_basestation.loop();
    return 0;
}

CommBasestation::CommBasestation(ros::NodeHandle *NH)
{
    loadConfig();

    sub_basestation = NH->subscribe("pc2bs_telemetry", 16, &CommBasestation::callbackSubBasestation, this);
    pub_basestation = NH->advertise<iris_its::BasestationRx>("bs2pc_telemetry", 16);

    //====================================
    ROS_INFO("UDP Server Initialization");
    //====================================
    sockfd_rx = socket(AF_INET, SOCK_DGRAM, 0);
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(SERVER_PORT);
    bind(sockfd_rx, (struct sockaddr *)&server_addr, sizeof(server_addr));

    //-----MULTICAST
    server_mreq.imr_multiaddr.s_addr = inet_addr(SERVER_IP.c_str());
    server_mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sockfd_rx, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&server_mreq, sizeof(server_mreq));

    //====================================
    ROS_INFO("UDP Client Initialization");
    //====================================
    sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0);
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(CLIENT_IP.c_str());
    client_addr.sin_port = htons(CLIENT_PORT);

    ROS_INFO("Client UDP %s %d", CLIENT_IP.c_str(), CLIENT_PORT);
}

CommBasestation::~CommBasestation()
{
}

void CommBasestation::loadConfig()
{
    Config config;
    config.load("communication.yaml");

    config.parseMapBegin("basestation");
    config.parseKeyValue("mode", &MODE_COMM);

    config.load("communication.yaml");

    config.parseMapBegin(MODE_COMM);
    config.parseKeyValue("client_ip", &CLIENT_IP);
    config.parseKeyValue("client_port", &CLIENT_PORT);
    config.parseKeyValue("server_ip", &SERVER_IP);
    config.parseKeyValue("server_port", &SERVER_PORT);
    config.parseMapEnd();

    ROS_INFO("======================================%s", SERVER_IP.c_str());

    config.load("robot.yaml");
    config.parseMapBegin("identity");
    int config_nomor;
    config.parseKeyValue("nomor", &config_nomor);
    config.parseMapEnd();

    nomor_robot = config_nomor;
}

void CommBasestation::callbackSubBasestation(const iris_its::BasestationTxConstPtr &msg)
{
    // iris_its::BasestationTx msg_basestation;
    //     msg_basestation.pos_x = robot->pos_x;
    //     msg_basestation.pos_y = robot->pos_y;
    //     msg_basestation.theta = robot->theta;
    //     msg_basestation.status_bola = ball->getStatus();
    //     if (ball->getStatus() == 0)
    //     {
    //         msg_basestation.bola_x_pada_lapangan = 9999;
    //         msg_basestation.bola_y_pada_lapangan = 9999;
    //     }
    //     else if (ball->getStatus() == 1)
    //     {
    //         msg_basestation.bola_x_pada_lapangan = ball->pos_x_pada_lapangan_vision;
    //         msg_basestation.bola_y_pada_lapangan = ball->pos_y_pada_lapangan_vision;
    //     }
    //     else if (ball->getStatus() == 2)
    //     {
    //         msg_basestation.bola_x_pada_lapangan = robot->pos_x;
    //         msg_basestation.bola_y_pada_lapangan = robot->pos_y;
    //     }
    //     msg_basestation.robot_condition = robot->condition;
    //     msg_basestation.target_umpan = robot->target_umpan;
    //     msg_basestation.status_algoritma = game->status_algoritma;
    //     msg_basestation.status_sub_algoritma = game->status_sub_algoritma;
    //     msg_basestation.status_sub_sub_algoritma = game->status_sub_sub_algoritma;
    //     msg_basestation.status_sub_sub_sub_algoritma = game->status_sub_sub_sub_algoritma;
    //     msg_basestation.epoch = basestation_epoch;
    //     for (int i = 0; i < 144; i++)
    //     {
    //         msg_basestation.jarak_obstacle.push_back(obstacle->halangan_pada_lapangan[i]);
    //     }
    //     msg_basestation.stm_epoch = stm_epoch;
    //     pub_basestation.publish(msg_basestation);
    //     msg_basestation.jarak_obstacle.clear();
    //     basestation_epoch++;

    tx_buffer[0] = 'i';
    tx_buffer[1] = 't';
    tx_buffer[2] = 's';
    tx_buffer[3] = nomor_robot;
    memcpy(tx_buffer + 4, &msg->pos_x, 2);
    memcpy(tx_buffer + 6, &msg->pos_y, 2);
    memcpy(tx_buffer + 8, &msg->theta, 2);
    memcpy(tx_buffer + 10, &msg->status_bola, 1);
    memcpy(tx_buffer + 11, &msg->bola_x_pada_lapangan, 2);
    memcpy(tx_buffer + 13, &msg->bola_y_pada_lapangan, 2);
    memcpy(tx_buffer + 15, &msg->robot_condition, 2);
    memcpy(tx_buffer + 17, &msg->target_umpan, 1);
    memcpy(tx_buffer + 18, &msg->status_algoritma, 2);
    memcpy(tx_buffer + 20, &msg->status_sub_algoritma, 2);
    memcpy(tx_buffer + 22, &msg->status_sub_sub_algoritma, 2);
    memcpy(tx_buffer + 24, &msg->status_sub_sub_sub_algoritma, 2);
    memcpy(tx_buffer + 26, &msg->epoch, 4);
    for (int i = 0; i < 144; i++)
    {
        memcpy((tx_buffer + 24) + (i * 2), &msg->jarak_obstacle, 2);
    }
    memcpy(tx_buffer + 310, &msg->stm_epoch, 2);
}

void CommBasestation::loop()
{
    static ros::Rate RATE(30);

    while (ros::ok())
    {
        ros::spinOnce();
        RATE.sleep();

        //-----Transmit
        tx_len = sendto(sockfd_tx, tx_buffer, sizeof(tx_buffer), MSG_DONTWAIT, (struct sockaddr *)&client_addr, sizeof(client_addr));
        // printf("tx errno %d\n", errno);
        //-----Receive
        socklen_t server_addr_len = sizeof(server_addr);
        rx_len = recvfrom(sockfd_rx, rx_buffer, sizeof(rx_buffer), MSG_DONTWAIT, (struct sockaddr *)&server_addr, &server_addr_len);
        // printf("rx errno %d\n", errno);

        // printf("BS: tx len %d rx len %d\n", tx_len, rx_len);
        // rx_len = recvfrom(sockfd_rx, rx_buffer, sizeof(rx_buffer), MSG_DONTWAIT, (struct sockaddr *)&dummy_addr, &dummy_socklen);
        if (rx_len > 0)
        {
            // ROS_WARN("Data masuk");
            static iris_its::BasestationRx msg_basestation;

            memcpy(&msg_basestation.header, rx_buffer + 0, 1);

            if (msg_basestation.header == 'i')
            {
                memcpy(&msg_basestation.command, rx_buffer + 1, 1);
                memcpy(&msg_basestation.style, rx_buffer + 2, 1);
                memcpy(&msg_basestation.cnn_status, rx_buffer + 3, 1);
                memcpy(&msg_basestation.n_robot_aktif, rx_buffer + 4, 1);
                memcpy(&msg_basestation.n_robot_dekat_bola, rx_buffer + 5, 1);
                memcpy(&msg_basestation.n_robot_dapat_bola, rx_buffer + 6, 1);
                memcpy(&msg_basestation.bola_x_pada_lapangan, rx_buffer + 7, 2);
                memcpy(&msg_basestation.bola_y_pada_lapangan, rx_buffer + 9, 2);

                memcpy(&msg_basestation.n_robot_sendiri, rx_buffer + 11 + (nomor_robot - 1), 1);
                if (msg_basestation.n_robot_sendiri == 2)
                    msg_basestation.n_robot_teman = 3;
                else if (msg_basestation.n_robot_sendiri == 3)
                    msg_basestation.n_robot_teman = 2;
                else
                    msg_basestation.n_robot_teman = 0;

                int16_t int_buffer[64];
                memcpy(int_buffer, rx_buffer + 15, 128);

                // Posisi Obstacle
                // memcpy(&msg_basestation.obs_1, rx_buffer + 75, 1);
                // memcpy(&msg_basestation.obs_2, rx_buffer + 76, 1);

                msg_basestation.odometry_robot1.x = int_buffer[0];
                msg_basestation.odometry_robot1.y = int_buffer[1];
                msg_basestation.odometry_robot1.theta = int_buffer[2];
                msg_basestation.odometry_offset_robot1.x = int_buffer[3];
                msg_basestation.odometry_offset_robot1.y = int_buffer[4];
                msg_basestation.odometry_offset_robot1.theta = int_buffer[5];
                msg_basestation.trim_kecepatan_robot1 = int_buffer[6];
                msg_basestation.trim_penendang_robot1 = int_buffer[7];
                msg_basestation.switch_robot1 = int_buffer[8];
                msg_basestation.robot_condition1 = int_buffer[9];

                msg_basestation.odometry_robot2.x = int_buffer[10];
                msg_basestation.odometry_robot2.y = int_buffer[11];
                msg_basestation.odometry_robot2.theta = int_buffer[12];
                msg_basestation.odometry_offset_robot2.x = int_buffer[13];
                msg_basestation.odometry_offset_robot2.y = int_buffer[14];
                msg_basestation.odometry_offset_robot2.theta = int_buffer[15];
                msg_basestation.trim_kecepatan_robot2 = int_buffer[16];
                msg_basestation.trim_penendang_robot2 = int_buffer[17];
                msg_basestation.switch_robot2 = int_buffer[18];
                msg_basestation.robot_condition2 = int_buffer[19];

                msg_basestation.odometry_robot3.x = int_buffer[20];
                msg_basestation.odometry_robot3.y = int_buffer[21];
                msg_basestation.odometry_robot3.theta = int_buffer[22];
                msg_basestation.odometry_offset_robot3.x = int_buffer[23];
                msg_basestation.odometry_offset_robot3.y = int_buffer[24];
                msg_basestation.odometry_offset_robot3.theta = int_buffer[25];
                msg_basestation.trim_kecepatan_robot3 = int_buffer[26];
                msg_basestation.trim_penendang_robot3 = int_buffer[27];
                msg_basestation.switch_robot3 = int_buffer[28];
                msg_basestation.robot_condition3 = int_buffer[29];

                msg_basestation.odometry_robot4.x = int_buffer[30];
                msg_basestation.odometry_robot4.y = int_buffer[31];
                msg_basestation.odometry_robot4.theta = int_buffer[32];
                msg_basestation.odometry_offset_robot4.x = int_buffer[33];
                msg_basestation.odometry_offset_robot4.y = int_buffer[34];
                msg_basestation.odometry_offset_robot4.theta = int_buffer[35];
                msg_basestation.trim_kecepatan_robot4 = int_buffer[36];
                msg_basestation.trim_penendang_robot4 = int_buffer[37];
                msg_basestation.switch_robot4 = int_buffer[38];
                msg_basestation.robot_condition4 = int_buffer[39];

                msg_basestation.odometry_robot5.x = int_buffer[40];
                msg_basestation.odometry_robot5.y = int_buffer[41];
                msg_basestation.odometry_robot5.theta = int_buffer[42];
                msg_basestation.odometry_offset_robot5.x = int_buffer[43];
                msg_basestation.odometry_offset_robot5.y = int_buffer[44];
                msg_basestation.odometry_offset_robot5.theta = int_buffer[45];
                msg_basestation.trim_kecepatan_robot5 = int_buffer[46];
                msg_basestation.trim_penendang_robot5 = int_buffer[47];
                msg_basestation.switch_robot5 = int_buffer[48];
                msg_basestation.robot_condition5 = int_buffer[49];

                msg_basestation.trim_kecepatan_sudut_robot1 = int_buffer[50];
                msg_basestation.trim_kecepatan_sudut_robot2 = int_buffer[51];
                msg_basestation.trim_kecepatan_sudut_robot3 = int_buffer[52];
                msg_basestation.trim_kecepatan_sudut_robot4 = int_buffer[53];
                msg_basestation.trim_kecepatan_sudut_robot5 = int_buffer[54];

                msg_basestation.pos_x_target_umpan = int_buffer[55];
                msg_basestation.pos_y_target_umpan = int_buffer[56];
                msg_basestation.status_umpan = int_buffer[57];

                msg_basestation.n_attacker_left = int_buffer[58];
                msg_basestation.n_attacker_right = int_buffer[59];
                msg_basestation.n_defender_left = int_buffer[60];
                msg_basestation.n_defender_right = int_buffer[61];

                msg_basestation.auto_kalibrasi = int_buffer[62];
            }
            else if (msg_basestation.header == 'm')
            {
                memcpy(&msg_basestation.n_robot_manual, rx_buffer + 1, 1);

                int16_t int_buffer[3];
                memcpy(int_buffer + 0, rx_buffer + 2, 6);

                msg_basestation.target_manual.x = int_buffer[0];
                msg_basestation.target_manual.y = int_buffer[1];
                msg_basestation.target_manual.theta = int_buffer[2];
            }

            pub_basestation.publish(msg_basestation);
        }
    }
}