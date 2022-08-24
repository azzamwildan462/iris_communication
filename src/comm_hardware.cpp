#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include "iris_its/BasestationRx.h"

using namespace std;
#define RECEIVE_PORT 8888
#define SEND_PORT 44444

#define BUFSIZE 64

ros::Timer tim_20hz_send;
ros::Timer tim_20hz_receive;

///--Deklarasi variabel

struct sockaddr_in myaddr;			 /* our address */
struct sockaddr_in remaddr;			 /* remote address */
socklen_t addrlen = sizeof(remaddr); /* length of addresses */
int recvlen;						 /* # bytes received */
int fd;								 /* our socket */
char buf[BUFSIZE];					 /* receive buffer */
char buffer[BUFSIZE];				 /* receive buffer */

//-----Subscriber
ros::Subscriber sub_velocity;
ros::Subscriber sub_odometry_offset;
ros::Subscriber sub_odometry;
ros::Subscriber sub_dribble;
ros::Subscriber sub_kicker;
ros::Subscriber sub_buzzer;
ros::Subscriber sub_basestation;
ros::Subscriber sub_posisi_tendang;
// gyro dari pc
ros::Subscriber sub_theta_buffer;
//-----Publisher
ros::Publisher pub_odometry_buffer;
ros::Publisher pub_tombol;
ros::Publisher pub_sensor_garis;
ros::Publisher pub_sensor_bola;
ros::Publisher pub_kompas;
ros::Publisher pub_lidar_samping;
ros::Publisher pub_lidar_belakang;
ros::Publisher pub_stm_epoch;

//-----Data serial
unsigned char serial_status = 0;
unsigned char serial_data = 0;
unsigned char serial_rx[64] = {'i', 't', 's'};
unsigned char serial_tx[64] = {'i', 't', 's'};
unsigned char header[3];
//-----Odometry
float pos_x, pos_y, theta;
float pos_x_buffer, pos_x_offset;
float pos_y_buffer, pos_y_offset;
float theta_buffer, theta_offset;
float kompas_buffer;
int tester;

float theta_buffer_gyrobaru;

//-----Velocity
short int kecepatan_x;
short int kecepatan_y;
short int kecepatan_sudut;

//-----Input
unsigned char tombol = 255;
unsigned char sensor_garis;
unsigned char sensor_bola[3];
float kompas;
unsigned int lidar_samping;
unsigned int lidar_belakang;

//-----Dribble
short int dribble_kiri;
short int dribble_kanan;

//-----Penendang
unsigned char mode_tendang = 0;
unsigned int kecepatan_tendang = 1500;
unsigned int ketinggian_penendang = 800;
unsigned int ketinggian_penendang_global = 800;
int kicker[2];
bool reset_kicker = true;

//-----Buzzer
short int buzzer_waktu = 10;
short int buzzer_jumlah = 10;

// ADC Baterai
uint32_t stm_epoch;

int counter_error = 0;
short int status_warna = 0;

// Testing
uint32_t counter_menerima_stm = 0;

void cllbck_sub_velocity(const geometry_msgs::TwistConstPtr &msg)
{
	kecepatan_x = msg->linear.x;
	kecepatan_y = msg->linear.y;
	kecepatan_sudut = msg->angular.z;
}

void cllbck_sub_odometry_offset(const geometry_msgs::Pose2DConstPtr &msg)
{
	pos_x_offset = msg->x;
	pos_y_offset = msg->y;
	theta_offset = msg->theta;
}

void cllbck_sub_dribble(const std_msgs::Int16MultiArrayConstPtr &msg)
{
	dribble_kiri = msg->data[0];
	dribble_kanan = msg->data[1];
}

void cllbck_sub_kicker(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
	reset_kicker = false;
	int i = 0;
	for (std::vector<short int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		kicker[i] = *it;
		i++;
	}
	mode_tendang = kicker[0];
	kecepatan_tendang = kicker[1];
	ketinggian_penendang = kicker[2];
	// kecepatan_tendang = kicker[1] - 100;

	ROS_INFO("%d || %d || %d", mode_tendang, kecepatan_tendang, ketinggian_penendang);
}

void cllbck_sub_buzzer(const std_msgs::Int16MultiArrayConstPtr &msg)
{
	buzzer_waktu = msg->data[0];
	buzzer_jumlah = msg->data[1];
}
// void cllbck_sub_theta_buffer(const std_msgs::Float32::ConstPtr &msg){
// 	theta_buffer_gyrobaru = msg->data;
// 	// ROS_INFO("THETA BUFFER %f", theta_buffer_gyrobaru);
// }

void cllbck_sub_odometry(const geometry_msgs::Pose2DConstPtr &msg)
{
	theta = msg->theta;
}

void cllbck_sub_posisi_tendang(const std_msgs::Int16ConstPtr &msg)
{
	ketinggian_penendang_global = msg->data;
}

void cllbck_tim_20hz_send(const ros::TimerEvent &event);
void cllbck_tim_20hz_receive(const ros::TimerEvent &event);

void cllbck_tim_20hz_send(const ros::TimerEvent &event)
{
	if (reset_kicker == true)
	{
		mode_tendang = 0;
		kecepatan_tendang = 1550;
	}

	if ((tombol & 0x40) >> 6 == 0x1)
	{
		status_warna = 1;
	}
	else if ((tombol & 0x40) >> 6 == 0)
	{
		status_warna = 2;
	}

	memcpy(serial_tx + 3, &kecepatan_x, 2);
	memcpy(serial_tx + 5, &kecepatan_y, 2);
	memcpy(serial_tx + 7, &kecepatan_sudut, 2);
	memcpy(serial_tx + 9, &pos_x_offset, 4);
	memcpy(serial_tx + 13, &pos_y_offset, 4);
	memcpy(serial_tx + 17, &theta_offset, 4);
	memcpy(serial_tx + 21, &mode_tendang, 1);
	memcpy(serial_tx + 22, &kecepatan_tendang, 2);
	memcpy(serial_tx + 24, &buzzer_jumlah, 2);
	memcpy(serial_tx + 26, &buzzer_waktu, 2);
	memcpy(serial_tx + 28, &dribble_kanan, 2);
	memcpy(serial_tx + 30, &dribble_kiri, 2);
	memcpy(serial_tx + 32, &ketinggian_penendang, 2);
	memcpy(serial_tx + 34, &status_warna, 2);
	memcpy(serial_tx + 36, &ketinggian_penendang_global, 2);

	// printf("%.02f %.02f\n", pos_x_offset, pos_y_offset);

	// ROS_INFO("bang? %d", ketinggian_penendang_global);

	// memcpy(serial_tx + 32, &theta, 4);
	// ROS_INFO("%c",buffer[0]);
	sendto(fd, (void *)serial_tx, sizeof(serial_tx), 0, (struct sockaddr *)&remaddr, addrlen);
	// sendto(fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&remaddr, addrlen);
	reset_kicker = true;
}

void cllbck_tim_20hz_receive(const ros::TimerEvent &event)
{
	// printf("recv_thread active\n");
	recvlen = recvfrom(fd, buf, BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);
	// sendto(fd, buf, sizeof(buf), 0, (struct sockaddr *)&remaddr, addrlen);
	// ROS_INFO("%c %c %c", header[0], header[1], header[2]);
	// printf("raw: %s\n", header[0]);
	if (recvlen > 0)
	{
		// printf("size: %d | errno %d | data: %s\n", recvlen, errno, buf);
		// counter_menerima_stm++;
		// printf("Gk error?\n");
		counter_error = 0;
		// msg_shared_header.data = shared_header[0];
		// pub_shared_header.publish(msg_shared_header);

		memcpy(&serial_rx, buf, recvlen);

		// memcpy(&header[0], serial_rx, 1);
		// memcpy(&header[1], serial_rx + 1, 1);
		// memcpy(&header[2], serial_rx + 2, 1);

		memcpy(header, serial_rx, 3);

		if (header[0] == 'i' && header[1] == 't' && header[2] == 's')
		{
			memcpy(&pos_x_buffer, serial_rx + 3, 4);
			memcpy(&pos_y_buffer, serial_rx + 7, 4);
			memcpy(&theta_buffer, serial_rx + 11, 4);
			memcpy(&sensor_garis, serial_rx + 15, 1);
			memcpy(&sensor_bola, serial_rx + 16, 2);
			memcpy(&tombol, serial_rx + 18, 1);
			memcpy(&stm_epoch, serial_rx + 19, 4);

			// ROS_INFO("stm_epoch %d", stm_epoch);
			// memcpy(&kompas_buffer, serial_rx + 23, 4);
			// memcpy(&tester, serial_rx + 27, 4);

			// ROS_INFO("sensor bola kiri kanan %d %d", serial_rx[16], serial_rx[17]);
			// memcpy(&kompas, serial_rx + 18, 4);
			// memcpy(&lidar_samping, serial_rx + 22, 2);
			// memcpy(&lidar_belakang, serial_rx + 24, 2);
			// Sensor SRF hanya ada pada robot kiper
#if iris1
			memcpy(&srf_udp[3], serial_rx + 18, 3);

			std_msgs::UInt8MultiArray msg_srf;
			msg_srf.data.push_back(srf_udp[0]);
			msg_srf.data.push_back(srf_udp[1]);
			msg_srf.data.push_back(srf_udp[2]);
			pub_srf.publish(msg_srf);
#endif
		}
	}
	else if (recvlen == -1)
	{
		// counter_error++;
	}

	if (counter_error > 69)
	{
		// ROS_WARN("KONEKSI STM PEDOT");
	}

	geometry_msgs::Pose2D msg_odometry_buffer;
	msg_odometry_buffer.x = pos_x_buffer;
	msg_odometry_buffer.y = pos_y_buffer;
	msg_odometry_buffer.theta = theta_buffer;
	pub_odometry_buffer.publish(msg_odometry_buffer);

	std_msgs::UInt8 msg_tombol;
	msg_tombol.data = tombol;
	pub_tombol.publish(msg_tombol);

	std_msgs::UInt8 msg_sensor_garis;
	msg_sensor_garis.data = sensor_garis;
	pub_sensor_garis.publish(msg_sensor_garis);

	// ROS_INFO("%d dan %d", sensor_bola[0], sensor_bola[1]);
	std_msgs::UInt8MultiArray msg_sensor_bola;
	msg_sensor_bola.data.push_back(sensor_bola[0]);
	msg_sensor_bola.data.push_back(sensor_bola[1]);
	pub_sensor_bola.publish(msg_sensor_bola);

	std_msgs::Float32 msg_data_kompas;
	msg_data_kompas.data = kompas_buffer;
	pub_kompas.publish(msg_data_kompas);

	std_msgs::Int16 msg_lidar_samping;
	msg_lidar_samping.data = lidar_samping;
	pub_lidar_samping.publish(msg_lidar_samping);

	std_msgs::Int16 msg_lidar_belakang;
	msg_lidar_belakang.data = lidar_belakang;
	pub_lidar_belakang.publish(msg_lidar_belakang);

	std_msgs::UInt32 msg_stm_epoch;
	msg_stm_epoch.data = stm_epoch;
	pub_stm_epoch.publish(msg_stm_epoch);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "comm_hardware");
	ros::NodeHandle NH;

	ros::MultiThreadedSpinner MTS;
	ros::Rate rosRate(50);

	/* create a UDP socket */

	if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		perror("cannot create socket\n");
		return 0;
	}

	/* bind the socket to any valid IP address and a specific port */

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_port = htons(RECEIVE_PORT);
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
	{
		perror("bind failed");
		return 0;
	}

	//-----Subscriber
	sub_velocity = NH.subscribe("pc2stm/velocity", 16, cllbck_sub_velocity);
	sub_odometry_offset = NH.subscribe("pc2stm/odometry_offset", 16, cllbck_sub_odometry_offset);
	sub_dribble = NH.subscribe("pc2stm/dribble", 16, cllbck_sub_dribble);
	sub_kicker = NH.subscribe("pc2stm/kicker", 8, cllbck_sub_kicker);
	sub_buzzer = NH.subscribe("pc2stm/buzzer", 16, cllbck_sub_buzzer);
	sub_posisi_tendang = NH.subscribe("pub_posisi_tendang", 16, cllbck_sub_posisi_tendang);
	sub_odometry = NH.subscribe("robot_odometry", 16, cllbck_sub_odometry);
	//-----Publisher
	pub_odometry_buffer = NH.advertise<geometry_msgs::Pose2D>("stm2pc/odometry_buffer", 16);
	pub_tombol = NH.advertise<std_msgs::UInt8>("stm2pc/tombol", 16);
	pub_sensor_garis = NH.advertise<std_msgs::UInt8>("stm2pc/sensor_garis", 16);
	pub_sensor_bola = NH.advertise<std_msgs::UInt8MultiArray>("stm2pc/sensor_bola", 16); // garapan
	pub_kompas = NH.advertise<std_msgs::Float32>("stm2pc/kompas", 16);
	pub_lidar_samping = NH.advertise<std_msgs::Int16>("stm2pc/lidar_samping", 16);
	pub_lidar_belakang = NH.advertise<std_msgs::Int16>("stm2pc/lidar_belakang", 16);
	pub_stm_epoch = NH.advertise<std_msgs::UInt32>("stm2pc/stm_epoch", 16);

	// Percobaan untuk mengecilkan frekuensi
	tim_20hz_send = NH.createTimer(ros::Duration(0.02), cllbck_tim_20hz_send);
	// tim_20hz_receive = NH.createTimer(ros::Duration(0.005), cllbck_tim_20hz_receive); // 200
	tim_20hz_receive = NH.createTimer(ros::Duration(0.005), cllbck_tim_20hz_receive);
	// tim_20hz_receive = NH.createTimer(ros::Duration(0.01), cllbck_tim_20hz_receive);
	// tim_20hz_receive = NH.createTimer(ros::Duration(0.001), cllbck_tim_20hz_receive); // asdaasd

	MTS.spin();
	return 0;
}
