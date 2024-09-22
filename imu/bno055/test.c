#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "bno055.h"

#define error_message printf
//#define DEBUG_UART
#define bno055_register_t int

static int serial_fd;

int
set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		error_message ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
			error_message ("error %d from tcsetattr", errno);
			return -1;
	}
	return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                error_message ("error %d setting term attributes", errno);
}

static int uart_init(const char *pName)
{
	int serial_fd = open (pName, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0)
    {
		error_message ("error %d opening %s: %s", errno, pName, strerror (errno));
		return 0;
    }

    set_interface_attribs (serial_fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (serial_fd, 0);                // set no blocking
	return serial_fd;
}

static int uart_write(uint8_t *data, int len)
{
	if (write(serial_fd, data, len) == -1)
	{
		printf("failed to write uart data:%s \n", data);
		return -1;
	}

#ifdef DEBUG_UART
	printf("Uart Write: \t");
	for(int i=0;i<len;i++)
	{
		printf("0x%02x ", data[i]);
	}
	printf("\n");
#endif
	return 1;
}

static int uart_read(uint8_t *buf, int len)
{
	if (read(serial_fd, buf, len) == -1)
	{
		printf("failed to read uart data\n");
		return -1;
	}
	
#ifdef DEBUG_UART
	printf("Uart Read: \t");
	for(int i=0;i<len;i++)
	{
		printf("0x%02x ", buf[i]);
	}
	printf("\n");
#endif
	return 1;
}

static int bno_write(uint8_t *cmd, int len)
{
RETRY:
	if(uart_write(cmd, len) > 0){
		uint8_t w_ack[2]={};
		uart_read(w_ack, 2);
		if(w_ack[0] != 0xEE || w_ack[1] != 0x01)
		{
			printf("bno055 write failed \n");
			return -1;
		}
	}else{
		printf("uart write failed\n");
		goto RETRY;
	}
	
	printf("bno055 write success\n");
	return 1;
}

static int bno_read(uint8_t *cmd, int len, uint8_t* data, int data_len)
{
	int rBytes=0;
	//uint8_t w_ack[2]={}, r_ack[2]={};
	uint8_t rbuf[data_len]={};

//RETRY_W:
	if(uart_write(cmd, len) > 0){
		
		//uart_read(w_ack, 2);
		//if(w_ack[0] != 0xEE || w_ack[1] != 0x01)
		//{
		//	printf("[%d] bno055 write failed \n", __LINE__);
		//	usleep(10);
		//	goto RETRY_W;
		//}else{
		//	printf("bno055 write success\n");
		//}
	}else{
		printf("[%d] uart write failed\n", __LINE__);
	}
	
RETRY_R:
//	if(uart_read(r_ack, 2) > 0){
//		if(r_ack[0] != 0xBB){
//			printf("bno055 read failed: 0x%02x %02x \n", r_ack[0], r_ack[1]);
//			usleep(1);
//			goto RETRY_R;
//		}
//	}else{
//		printf("uart read failed\n");
//		goto RETRY_R;
//	}
//	
//	printf("bno055 read success\n");
//	rBytes = r_ack[1];
//	if(uart_read(data, rBytes) > 0){
//		return rBytes;
//	}
	
	if(uart_read(rbuf, data_len+2) > 0){
		if(rbuf[0] != 0xBB){
			printf("bno055 read failed: 0x%02x %02x \n", rbuf[0], rbuf[1]);
			usleep(1);
			goto RETRY_R;
		}
	}else{
		printf("uart read failed\n");
		goto RETRY_R;
	}
	
	//success
	memcpy(data, &rbuf[2], data_len);
	
	return 0;
}


void bno055_set_mode(uint8_t opMode) {
    uint8_t set_mode_cmd[5] = {0xAA, 0x00, 0x3D, 0x01, opMode};  // 设置为IMU模式
    //uart_write(set_mode_cmd, sizeof(set_mode_cmd));
	bno_write(set_mode_cmd, sizeof(set_mode_cmd));
    usleep(100);
}

void bno055_read_chipId(){
	
	uint8_t read_chip_id[] = {0xAA, 0x01, 0x00, 0x01};
	uint8_t chipId=0;
	
	bno_read(read_chip_id, sizeof(read_chip_id), &chipId, 1);
	
	printf("chip id: 0x%02x\n", chipId);
}

int bno055_imu_ready(){
	
	uint8_t cmd_intr_stat[] = {0xAA, 0x01, (uint8_t)BNO055_INTR_STAT_ADDR, 0x01};
	uint8_t intr_stat=0;
	bno_read(cmd_intr_stat, sizeof(cmd_intr_stat), &intr_stat, 1);
	
	printf("intr_stat: 0x%02x\n", intr_stat);
	return intr_stat;
}

void bno055_read_imu_data() {
    uint8_t read_accel_cmd[4] = {0xAA, 0x01, 0x08, 0x06};  // 读取加速度6个字节
    uint8_t read_gyro_cmd[4] = {0xAA, 0x01, 0x14, 0x06};   // 读取陀螺仪6个字节
    uint8_t read_mag_cmd[4] = {0xAA, 0x01, 0x0E, 0x06};    // 读取磁力计6个字节

    uint8_t accel_data[6]={}, gyro_data[6]={}, mag_data[6]={};
	
	
    // 发送读取加速度命令并接收数据
    //uart_write(read_accel_cmd, 4);
    //uart_read(accel_data, 6);
	bno_read(read_accel_cmd, 4, accel_data, 6);
	usleep(1);

    // 发送读取陀螺仪命令并接收数据
    //uart_write(read_gyro_cmd, 4);
    //uart_read(gyro_data, 6);
	bno_read(read_gyro_cmd, 4, gyro_data, 6);
	usleep(1);

    // 发送读取磁力计命令并接收数据
    //uart_write(read_mag_cmd, 4);
    //uart_read(mag_data, 6);
	bno_read(read_mag_cmd, 4, mag_data, 6);
	

    // 将读取到的字节数据转换为实际值
    int16_t accel_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t accel_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t accel_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    int16_t gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    int16_t mag_x = (int16_t)((mag_data[1] << 8) | mag_data[0]);
    int16_t mag_y = (int16_t)((mag_data[3] << 8) | mag_data[2]);
    int16_t mag_z = (int16_t)((mag_data[5] << 8) | mag_data[4]);

    // 输出IMU数据
    printf("Accel: X=%d Y=%d Z=%d\n", accel_x, accel_y, accel_z);
    printf("Gyro: X=%d Y=%d Z=%d\n", gyro_x, gyro_y, gyro_z);
    printf("Mag: X=%d Y=%d Z=%d\n", mag_x, mag_y, mag_z);
}

int main()
{
    const char *portname = "/dev/ttyUSB0";
	serial_fd = uart_init(portname);
	
	// 设置BNO055工作模式为: OPERATION_MODE_CONFIG 
	bno055_set_mode(OPERATION_MODE_CONFIG);
	
	bno055_read_chipId();

	// 设置BNO055工作模式为AMG模式: OPERATION_MODE_AMG
	bno055_set_mode(OPERATION_MODE_AMG);

    while (1) {
		//if(bno055_imu_ready() & 0x44){
			bno055_read_imu_data();  // 读取IMU数据
		//}
        usleep(100000);  // 每秒读取一次
    }
	
	
}