#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <fstream>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>
#include <sstream>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>
#include "zlib.h"
#include "lz4.h"
// #include "iris_routine/utils/shared_data.h"
#include "ros/ros.h"
#include "iris_its/config.h"
#include "iris_its/MulticastTx.h"
#include "iris_its/MulticastRx.h"
#include <redis-cpp/stream.h>
#include <redis-cpp/execute.h>
#include <chrono>

// #define COMM_DEBUG // Activate it to debug
// #define COMM_TEST // Activate it to test the program
#define COMM_DEBUG_MINOR
// #define USE_REALTIME_PROCESS // Activate it to use realtime process

#define PERR(txt, par...) \
    printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ##par)
#define PERRNO(txt) \
    printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

// Compress data
#define UNCOMPRESS 0
#define ZLIB_COMPRESS 1
#define LZ4_COMPRESS 2

// Socket
typedef struct multiSocket_tag
{
    struct sockaddr_in destAddress;
    int socketID;
    bool compressedData;
} multiSocket_t;
typedef struct nw_config
{
    char multicast_ip[16];
    char iface[16];
    char identifier[1];
    unsigned int port;
    uint8_t compress_type;
} config;
multiSocket_t multiSocket;
multiSocket_t *recv_socket;

// Config
config nw_config;

// Data
// char data[128] = "its3123456908its";
char data[128];
unsigned long int actual_data_size = 15;
char its[4] = "its";

unsigned long int max_recv_data_size = 128;
char recv_data[128] = "its1";
unsigned long int recv_data_size;

struct sockaddr src_addr;
socklen_t addr_len = sizeof(src_addr);

// SharedData *shared_data;

// ROS
ros::Timer tim_50hz_send;
ros::Timer tim_50hz_receive;
ros::Publisher pub_multicast_rx;
ros::Subscriber sub_multicast_tx;

// Data buffer
int16_t robot_pos_x;
int16_t robot_pos_y;
int16_t robot_pos_theta;
uint8_t status_bola;
int16_t bola_x_pada_lapangan;
int16_t bola_y_pada_lapangan;
int16_t robot_condition;
int8_t target_umpan;
uint16_t status_algoritma;
uint16_t status_sub_algoritma;
uint16_t status_sub_sub_algoritma;
uint16_t status_sub_sub_sub_algoritma;

// KeyDB
auto keydb_put_stream = rediscpp::make_stream("127.0.0.1", "6969");
auto keydb_get_stream = rediscpp::make_stream("127.0.0.1", "6969");
uint8_t toleransi_data_delay = 15;
int64_t robot_epoch[6];

// ...
int8_t data_valid = 0;

void putDBData()
{
    // printf("halo\n");
    uint8_t total_item = 4;
    char put_keys[4][7] = {"pos_x", "pos_y", "theta", "r_cond"};
    int8_t put_keys_size[4] = {6, 6, 6, 7};
    int8_t robot_value_pos[4] = {12, 14, 16, 23};
    int8_t robot_value_size[4] = {2, 2, 2, 2};

    char val_buffer[64];
    uint16_t val_buffer_int;
    int64_t epoch_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int64_t recv_epoch;
    uint8_t recv_from;
    int8_t status_valid = 0;
    memcpy(&recv_from, recv_data + 3, 1);
    memcpy(&recv_epoch, recv_data + 4, 8);
    recv_from -= '0';
    robot_epoch[recv_from] = recv_epoch;

    for (int i = 0; i < total_item; i++)
    {
        memcpy((void *)put_keys[i] + put_keys_size[i] - 1, recv_data + 3, 1);
        // Final safety
        if (put_keys[i][put_keys_size[i] - 1] >= '1' && put_keys[i][put_keys_size[i] - 1] <= '5') // Robot
        {
            memcpy(&val_buffer_int, recv_data + robot_value_pos[i], robot_value_size[i]);
            // printf("%s: %d\n", put_keys[i], val_buffer_int);
            auto const put_value = std::to_string(val_buffer_int);
            rediscpp::execute_no_flush(*keydb_put_stream, "set", put_keys[i], put_value);
        }
        else if (put_keys[i][put_keys_size[i] - 1] == '0') // BS
        {
        }
    }
    // Set status data apakah valid atau tidak, data dikatakan tidak valid ketika datanya expired
    char s_valid_key[16] = "data_valid";
    for (int8_t i = 1; i <= 5; i++)
    {
        char id[2];
        sprintf(id, "%d", i);
        memcpy(s_valid_key + 10, id, 1);
        status_valid = (abs(epoch_now - robot_epoch[i]) <= toleransi_data_delay);
        auto const put_value = std::to_string(status_valid);
        rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key, put_value);
        // printf("%s: %d %d -> %d (%d)\n", s_valid_key, epoch_now, robot_epoch[i], abs(epoch_now - robot_epoch[i]), status_valid);
    }

    // memcpy((void *)s_valid_key + 10, recv_data + 3, 1);
    // auto const put_value = std::to_string(status_valid);
    // rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key, put_value);
    std::flush(*keydb_put_stream);
}

void getDBData()
{
}

int if_NameToIndex(char *ifname, char *address)
{
    // printf("args: %s %s\n", ifname, address);
    int fd;
    struct ifreq if_info;
    int if_index;

    memset(&if_info, 0, sizeof(if_info));
    strncpy(if_info.ifr_name, ifname, IFNAMSIZ - 1);

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        PERRNO("socket");
        return -1;
    }
    if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }
    if_index = if_info.ifr_ifindex;

    if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }

    close(fd);

    sprintf(address, "%d.%d.%d.%d\n",
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[2],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[3],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[4],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[5]);
#ifdef COMM_DEBUG
    printf("**** Using device %s -> Ethernet %s\n", if_info.ifr_name, address);
#endif

    return if_index;
}
int openSocket()
{
    struct sockaddr_in multicastAddress;
    struct ip_mreqn mreqn;
    struct ip_mreq mreq;
    int opt;
    char address[16]; // IPV4: xxx.xxx.xxx.xxx\0

    bzero(&multicastAddress, sizeof(struct sockaddr_in));
    multicastAddress.sin_family = AF_INET;
    multicastAddress.sin_port = htons(nw_config.port);
    multicastAddress.sin_addr.s_addr = INADDR_ANY;

    bzero(&multiSocket.destAddress, sizeof(struct sockaddr_in));
    multiSocket.destAddress.sin_family = AF_INET;
    multiSocket.destAddress.sin_port = htons(nw_config.port);
    multiSocket.destAddress.sin_addr.s_addr = inet_addr(nw_config.multicast_ip);

    if ((multiSocket.socketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        PERRNO("socket");
        return -1;
    }

    memset((void *)&mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex = if_NameToIndex(nw_config.iface, address);
    if ((setsockopt(multiSocket.socketID, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
    {
        PERRNO("setsockopt 1");
        return -1;
    }

    opt = 1;
    if ((setsockopt(multiSocket.socketID, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt 2");
        return -1;
    }

    memset((void *)&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(nw_config.multicast_ip);
    mreq.imr_interface.s_addr = inet_addr(address);
    // fprintf(stderr, "Index: %d (port %d, %s / %s)\n", multiSocket.socketID, 4321, "224.168.1.80", address);

    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
    {
        PERRNO("setsockopt 3");
        printf("\nerrno %d\n", errno);
        return -1;
    }

    /* Disable reception of our own multicast */
    opt = 1; // default 0
    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt");
        return -1;
    }

    if (bind(multiSocket.socketID, (struct sockaddr *)&multicastAddress, sizeof(struct sockaddr_in)) == -1)
    {
        PERRNO("bind");
        return -1;
    }

    return 0;
}
void closeSocket()
{
    if (multiSocket.socketID != -1)
        shutdown(multiSocket.socketID, SHUT_RDWR);
}
void loadConfig()
{
    // Buffer
    int port;
    std::string iface;
    int compress_type;
    std::string multicast_ip;
    std::string identifier;

    Config config;
    config.load("communication.yaml");

    config.parseMapBegin("multicast_v2");
    config.parseKeyValue("port", &port);
    config.parseKeyValue("iface", &iface);
    config.parseKeyValue("compress_type", &compress_type);
    config.parseKeyValue("multicast_ip", &multicast_ip);
    config.parseKeyValue("identifier", &identifier);
    config.parseMapEnd();

    strcpy(nw_config.multicast_ip, multicast_ip.c_str());
    strcpy(nw_config.iface, iface.c_str());
    strcpy(nw_config.identifier, identifier.c_str());
    nw_config.port = port;
    nw_config.compress_type = compress_type;

    // printf("iface: %s\n", nw_config.iface);
    // printf("ip: %s\n", nw_config.multicast_ip);
    // printf("port: %d\n", nw_config.port);
    // printf("ctype: %d\n", nw_config.compress_type);
    // printf("identifier: %s\n", nw_config.identifier);
}

void saveData()
{
    uint8_t identifier_mux;
    iris_its::MulticastRx msg_multicast_rx;
    memcpy(&msg_multicast_rx.identifier, recv_data + 3, 1);
    msg_multicast_rx.identifier -= '0';

    // Safety
    if (msg_multicast_rx.identifier < 0 || msg_multicast_rx.identifier > 5)
        return;

    if (msg_multicast_rx.identifier == 0) // Terima dari BS
    {
        // printf("Hello from BS\n")
        memcpy(&msg_multicast_rx.header, recv_data + 4, 1);
        memcpy(&msg_multicast_rx.command, recv_data + 5, 1);
        memcpy(&msg_multicast_rx.style, recv_data + 6, 1);
        memcpy(&msg_multicast_rx.bola_x_pada_lapangan, recv_data + 7, 2);
        memcpy(&msg_multicast_rx.bola_y_pada_lapangan, recv_data + 9, 2);
        memcpy(&msg_multicast_rx.auto_kalibrasi, recv_data + 11, 1);
        memcpy(&msg_multicast_rx.offset_bs_x, recv_data + 12, 2);
        memcpy(&msg_multicast_rx.offset_bs_y, recv_data + 14, 2);
        memcpy(&msg_multicast_rx.offset_bs_theta, recv_data + 16, 2);
        memcpy(&msg_multicast_rx.target_manual_x, recv_data + 18, 2);
        memcpy(&msg_multicast_rx.target_manual_y, recv_data + 20, 2);
        memcpy(&msg_multicast_rx.target_manual_theta, recv_data + 22, 2);
        memcpy(&msg_multicast_rx.data_n_robot_mux1, recv_data + 24, 2);
        memcpy(&msg_multicast_rx.data_n_robot_mux2, recv_data + 26, 2);
        memcpy(&msg_multicast_rx.trim_kecepatan_robot, recv_data + 28 + atoi(nw_config.identifier) - 1, 1);
        memcpy(&msg_multicast_rx.trim_kecepatan_sudut_robot, recv_data + 33 + atoi(nw_config.identifier) - 1, 1);
        memcpy(&msg_multicast_rx.trim_kecepatan_tendang_robot, recv_data + 38 + atoi(nw_config.identifier) - 1, 1);
        // total 43 bytes
    }
    else // Recv from Robot
    {
        putDBData();
        // memcpy(&msg_multicast_rx.pos_x_robot, recv_data + 12, 2);
        // memcpy(&msg_multicast_rx.pos_y_robot, recv_data + 14, 2);
        // memcpy(&msg_multicast_rx.theta_robot, recv_data + 16, 2);
        // memcpy(&msg_multicast_rx.robot_condition, recv_data + 23, 2);
        // memcpy(&msg_multicast_rx.status_sub_sub_sub_algoritma, recv_data + 32, 2);
    }

    pub_multicast_rx.publish(msg_multicast_rx);
#ifdef COMM_DEBUG_MINOR
    // printf("[recv] mux %d\n", msg_multicast_rx.identifier);
    // printf("[recv] odom: %d %d %d\n", msg_multicast_rx.pos_x_robot, msg_multicast_rx.pos_y_robot, msg_multicast_rx.theta_robot);
    // printf("[recv] bola: %d %d %d\n", msg_multicast_rx.status_bola_robot, msg_multicast_rx.bola_x_pada_lapangan_robot, msg_multicast_rx.bola_y_pada_lapangan_robot);
    // printf("[recv] misc: %d %d %d\n", msg_multicast_rx.status_bola_robot, msg_multicast_rx.bola_x_pada_lapangan_robot, msg_multicast_rx.bola_y_pada_lapangan_robot);
    // printf("[recv] SM: %d %d %d\n", msg_multicast_rx.status_bola_robot, msg_multicast_rx.bola_x_pada_lapangan_robot, msg_multicast_rx.bola_y_pada_lapangan_robot);
#endif
}

void loadData()
{
#ifdef COMM_DEBUG_MINOR
    // printf("[send] odom: %d %d %d\n", robot_pos_x, robot_pos_y, robot_pos_theta);
    // printf("[send] bola: %d %d %d\n", status_bola, bola_x_pada_lapangan, bola_y_pada_lapangan);

    // printf("[send] misc: %d %d \n", robot_condition, target_umpan);
    // printf("[send] SM: %d %d %d %d\n", status_algoritma, status_sub_algoritma, status_sub_sub_algoritma, status_sub_sub_sub_algoritma);
#endif
    int64_t epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // printf("asudysadyfsau %ld\n", epoch);
    memcpy(data, its, 3);                      // Header
    memcpy(data + 3, nw_config.identifier, 1); // identifier
    memcpy(data + 4, &epoch, 8);               // Unix epoch time
    // Assign data
    memcpy(data + 12, &robot_pos_x, 2);
    memcpy(data + 14, &robot_pos_y, 2);
    memcpy(data + 16, &robot_pos_theta, 2);
    memcpy(data + 18, &status_bola, 1);
    memcpy(data + 19, &bola_x_pada_lapangan, 2);
    memcpy(data + 21, &bola_y_pada_lapangan, 2);
    memcpy(data + 23, &robot_condition, 2);
    memcpy(data + 25, &target_umpan, 1);
    memcpy(data + 26, &status_algoritma, 2);
    memcpy(data + 28, &status_sub_algoritma, 2);
    memcpy(data + 30, &status_sub_sub_algoritma, 2);
    memcpy(data + 32, &status_sub_sub_sub_algoritma, 2);

    actual_data_size = 34; // Selalu update ini ketika merubah data-frame
    // // printf("data_final: %d %s\n", actual_data_size, data);

    // memcpy(data, its, 3);                      // Header
    // memcpy(data + 3, nw_config.identifier, 1); // identifier
    // // Assign data
    // memcpy(data + 4, &robot_pos_x, 2);
    // memcpy(data + 6, &robot_pos_y, 2);
    // memcpy(data + 8, &robot_pos_theta, 2);
    // memcpy(data + 10, &status_bola, 1);
    // memcpy(data + 11, &bola_x_pada_lapangan, 2);
    // memcpy(data + 13, &bola_y_pada_lapangan, 2);
    // memcpy(data + 15, &robot_condition, 2);
    // memcpy(data + 17, &target_umpan, 1);
    // memcpy(data + 18, &status_algoritma, 2);
    // memcpy(data + 20, &status_sub_algoritma, 2);
    // memcpy(data + 22, &status_sub_sub_algoritma, 2);
    // memcpy(data + 24, &status_sub_sub_sub_algoritma, 2);
    // actual_data_size = 26; // Selalu update ini ketika merubah data-frame
}

int sendData()
{
    // printf("send id %d\n", multiSocket.socketID);
    if (nw_config.compress_type == UNCOMPRESS)
    {
        // Just send
        int nsent = sendto(multiSocket.socketID, data, actual_data_size, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
#ifdef COMM_TEST
        printf("[send] size %d, data %s\n", actual_data_size, data);
#endif
        if (nsent == actual_data_size)
            return 0;
        else
            return -2;
    }
    else if (nw_config.compress_type == ZLIB_COMPRESS)
    {
        // Compress with ZLIB before send
        unsigned long int compressed_data_size = compressBound(actual_data_size);
        char compressed_data[compressed_data_size];
#ifdef COMM_DEBUG
        printf("Before: %d and %d -> %s\n", sizeof(data), sizeof(compressed_data), data);
#endif
        compress((Bytef *)compressed_data, &compressed_data_size, (Bytef *)data, (unsigned long)actual_data_size);
#ifdef COMM_DEBUG
        printf("After: %d and %d -> %s\n", sizeof(data), sizeof(compressed_data), compressed_data);
        unsigned long int data_raw_size = compressBound(compressed_data_size);
        char data_raw[data_raw_size];
        uncompress((Bytef *)data_raw, &data_raw_size, (Bytef *)compressed_data, (unsigned long)compressed_data_size);
        printf("Uncompress: %d and %d -> %s\n", sizeof(data_raw), sizeof(compressed_data), data_raw);
#endif
        int nsent = sendto(multiSocket.socketID, compressed_data, compressed_data_size, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
#ifdef COMM_TEST
        printf("[send] size %d, data %s\n", compressed_data_size, compressed_data);
#endif
        if (nsent == compressed_data_size)
            return 0;
        else
            return -2;
    }
    else if (nw_config.compress_type == LZ4_COMPRESS)
    {
        // Compress with LZ4 before send
        int compressed_data_size = LZ4_compressBound(actual_data_size);
        char compressed_data[compressed_data_size];
#ifdef COMM_DEBUG
        printf("Before: %d and %d -> %s\n", sizeof(data), sizeof(compressed_data), data);
#endif
        compressed_data_size = LZ4_compress_default(data, &compressed_data[0], actual_data_size, compressed_data_size);
#ifdef COMM_DEBUG
        printf("After: %d and %d -> %s\n", sizeof(data), sizeof(compressed_data), compressed_data);
        unsigned long int data_raw_size = LZ4_compressBound(compressed_data_size);
        char data_raw[data_raw_size];
        data_raw_size = LZ4_decompress_safe(compressed_data, &data_raw[0], compressed_data_size, data_raw_size);
        printf("Uncompress: %d and %d -> %s\n", sizeof(data_raw), sizeof(compressed_data), data_raw);
#endif
        int nsent = sendto(multiSocket.socketID, compressed_data, compressed_data_size, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
#ifdef COMM_TEST
        printf("[send] size %d, data %s\n", compressed_data_size, compressed_data);
#endif
        if (nsent == compressed_data_size)
            return 0;
        else
            return -2;
    }

    return 0;
}

int recvData()
{
    // struct sockaddr src_addr;
    // socklen_t addr_len;
    char recv_buffer[max_recv_data_size] = "its";
    // int nrecv = recvfrom(recv_socket->socketID, (void *)recv_buffer, max_recv_data_size, MSG_DONTWAIT, &src_addr, &addr_len);
    int nrecv = recvfrom(recv_socket->socketID, recv_buffer, max_recv_data_size, 0, &src_addr, &addr_len);
    // printf("Buffer %d, nrecv: %d | %d -> %s\n", recv_socket->socketID, nrecv, errno, recv_buffer);
    if (nrecv == -1 && errno == 22 && recv_buffer[0] == 'i')
    {
        printf("Error occured, restart socket %s\n", recv_buffer);
        // closeSocket();
        // if (openSocket() == -1)
        // {
        //     PERR("openMulticastSocket");
        //     // return -1;
        // }
        return -1;
    }
#ifdef COMM_DEBUG
    printf("Buffer, nrecv: %d -> %s\n", nrecv, recv_buffer);
#endif
    if (nrecv > 0 && recv_buffer[0] == 'i')
    {
        // printf("data: %s\n", recv_buffer);
        bzero(recv_data, max_recv_data_size);
        if (nw_config.compress_type == UNCOMPRESS)
        {
            // strncpy(recv_data, recv_buffer, nrecv);
            recv_data_size = nrecv;
            memcpy(recv_data, recv_buffer, nrecv);
        }
        else if (nw_config.compress_type == ZLIB_COMPRESS)
        {
            recv_data_size = compressBound(nrecv);
            uncompress((Bytef *)recv_data, &recv_data_size, (Bytef *)recv_buffer, (unsigned long)nrecv);
        }
        else if (nw_config.compress_type == LZ4_COMPRESS)
        {
            // recv_data_size = LZ4_compressBound(nrecv);
            recv_data_size = compressBound(nrecv);
            // printf("size: %d and %d\n", recv_data_size, nrecv);
            recv_data_size = LZ4_decompress_safe(recv_buffer, &recv_data[0], nrecv, recv_data_size);
        }
        // char *src_ip = inet_ntoa(((struct sockaddr_in *)&src_addr)->sin_addr);
#if defined(COMM_DEBUG) || defined(COMM_TEST)
        // printf("[recv] recv_from: %s\n", src_ip);
#endif
        if (recv_data_size > 0)
            saveData();
    }

    return 0;
}

void callbackSubDataTx(const iris_its::MulticastTxPtr &msg)
{
    // #ifdef COMM_DEBUG_MINOR
    //     printf("[send] odom: %d %d %d\n", msg->pos_x, msg->pos_y, msg->theta);
    //     printf("[send] bola: %d %d %d\n", msg->status_bola, msg->bola_x_pada_lapangan, msg->bola_y_pada_lapangan);
    // #endif

    //     memcpy(data, its, 3);                      // Header
    //     memcpy(data + 3, nw_config.identifier, 1); // identifier
    //     memcpy(data + 4, &msg->pos_x, 2);
    //     memcpy(data + 6, &msg->pos_y, 2);
    //     memcpy(data + 8, &msg->theta, 2);
    //     memcpy(data + 10, &msg->status_bola, 1);
    //     memcpy(data + 11, &msg->bola_x_pada_lapangan, 2);
    //     memcpy(data + 13, &msg->bola_y_pada_lapangan, 2);
    //     memcpy(data + 15, &msg->robot_condition, 2);
    //     memcpy(data + 17, &msg->target_umpan, 1);
    //     memcpy(data + 18, &msg->status_algoritma, 2);
    //     memcpy(data + 20, &msg->status_sub_algoritma, 2);
    //     memcpy(data + 22, &msg->status_sub_sub_algoritma, 2);
    //     memcpy(data + 24, &msg->status_sub_sub_sub_algoritma, 2);

    //     actual_data_size = 26;

    robot_pos_x = msg->pos_x;
    robot_pos_y = msg->pos_y;
    robot_pos_theta = msg->theta;
    status_bola = msg->status_bola;
    if (status_bola == 0)
    {
        bola_x_pada_lapangan = 9999;
        bola_y_pada_lapangan = 9999;
    }
    else if (status_bola == 1)
    {
        bola_x_pada_lapangan = msg->bola_x_pada_lapangan;
        bola_y_pada_lapangan = msg->bola_y_pada_lapangan;
    }
    else if (status_bola == 2)
    {
        bola_x_pada_lapangan = robot_pos_x;
        bola_y_pada_lapangan = robot_pos_y;
    }
    robot_condition = msg->robot_condition;
    target_umpan = msg->target_umpan;
    status_algoritma = msg->status_algoritma;
    status_sub_algoritma = msg->status_sub_algoritma;
    status_sub_sub_algoritma = msg->status_sub_sub_algoritma;
    status_sub_sub_sub_algoritma = msg->status_sub_sub_sub_algoritma;

    data_valid = 1;
}

void cllbckTim50HzSend(const ros::TimerEvent &event)
{
    // printf("[send] hello \n");
    if (data_valid)
    {
        loadData();
        if (sendData() == -1)
        {
            PERR("sendData");
            ros::shutdown();
        }
    }
}
void cllbckTim50HzRecv(const ros::TimerEvent &event)
{
    // printf("[recv] hello \n");
    if (recvData() == -1)
    {
        PERR("recvData");
        closeSocket();
        ros::shutdown();
    }
    // saveData();
}

void resetStatusData()
{
    memset(robot_epoch, 0, 6);

    char s_valid_key1[16] = "data_valid1";
    char s_valid_key2[16] = "data_valid2";
    char s_valid_key3[16] = "data_valid3";
    char s_valid_key4[16] = "data_valid4";
    char s_valid_key5[16] = "data_valid5";
    auto const false_value = std::to_string(0);
    rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key1, false_value);
    rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key2, false_value);
    rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key3, false_value);
    rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key4, false_value);
    rediscpp::execute_no_flush(*keydb_put_stream, "set", s_valid_key5, false_value);
    std::flush(*keydb_put_stream);
}

void signalHandler(int sig)
{
    void resetStatusData();
    closeSocket();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // sleep(1.5);

    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    // keydb_put_stream = rediscpp::make_stream("127.0.0.1", "6969");

    // auto keydb_put_stream = rediscpp::make_stream("127.0.0.1", "6969");
    // auto keydb_get_stream = rediscpp::make_stream("127.0.0.1", "6969");

    // const char put_keys[3][7] = {"pos_x1", "pos_y1", "theta1"};
    // const char get_keys[3][7] = {"pos_x", "pos_y", "theta"};

    // Put

    // return 0;

    loadConfig();

    signal(SIGINT, signalHandler);

    resetStatusData();

    // shared_data = SharedData::getInstance();
    // printf("awal %d\n", shared_data->status_bola);
    //

#ifdef USE_REALTIME_PROCESS
    struct sched_param proc_sched;
    proc_sched.sched_priority = 60;
    if ((sched_setscheduler(getpid(), SCHED_FIFO, &proc_sched)) < 0)
    {
        PERRNO("setscheduler");
        return -1;
    }
#endif

    if (openSocket() == -1)
    {
        PERR("openMulticastSocket");
        return -1;
    }

    recv_socket = &multiSocket;

    // printf("send socket id: %d\n", multiSocket.socketID); //s

    sub_multicast_tx = NH.subscribe("multicast_tx", 16, &callbackSubDataTx);
    pub_multicast_rx = NH.advertise<iris_its::MulticastRx>("multicast_rx", 16);

    tim_50hz_send = NH.createTimer(ros::Duration(0.1), cllbckTim50HzSend);
    tim_50hz_receive = NH.createTimer(ros::Duration(0.01), cllbckTim50HzRecv);

    MTS.spin();

    return 0;
}