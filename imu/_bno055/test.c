#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "debug.h"

#define U8 unsigned char
#define S16 signed short

#define MAX_DEVICE_NAME 36
#define MAX_READ_SZ 1024

static int uart_fd=0;
char device[MAX_DEVICE_NAME];
static const speed_t invalid_speed_parameter = (speed_t)-1;

speed_t initial_speed = 115200;
int initial_bits=8;
char initial_event='N';
int initial_stop=1;

#define TEST 1
#if TEST
#define MAX_GBUF_SZ 100
int flag_w = 0;
int g_buffer_sz=0;
char g_buffer[MAX_GBUF_SZ+1] = "";
#endif


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch(nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}

	switch(nEvent)
	{
		case 'O':
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': 
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N':  
			newtio.c_cflag &= ~PARENB;
			break;
	}

	//already finish the speed translation
	//set speed directly here
	cfsetispeed(&newtio, nSpeed);
	cfsetospeed(&newtio, nSpeed);
	
	if( nStop == 1 ){
		newtio.c_cflag &=  ~CSTOPB;
	}
	else if ( nStop == 2 ){
		newtio.c_cflag |=  CSTOPB;
	}
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	return 0;
}

int init_uart(int flags, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct  termios pts;  /* termios settings on port */
    struct  termios sts;  /* termios settings on stdout/in */
	
	uart_fd = open(device, flags);
	if(uart_fd<0){
		APP_E("open device failed and exit : ");
		return errno;
	}
	
	if(!set_opt(uart_fd, nSpeed, nBits, nEvent, nStop))
		return 0;
	else
		return -1;
}

void show_buf(char *buf, int buf_sz)
{
	int i;
	for(i=0;i<buf_sz;i++){
		APP_DDD("0x%02x ", buf[i]);
	}
}

void usage(char *app_name)
{
	printf("Application Usage:\n");
	printf(" %s -D <device> -s <speed> -b <bits> -c <cts> -p <pts>\n", app_name);
	printf("ex:\n");
	printf(" %s -D /dev/ttyXRUSB0 -s 9600 -b8 -cN -p1\n", app_name);
	printf(" %s -D /dev/ttyXRUSB0 -s 115200 -b8 -cN -p1\n", app_name);
}

void signal_handle(int dunno)
{
	if((SIGINT == dunno) || (SIGKILL == dunno))
	{
		if(uart_fd){
			APP_I("close fd");
			close(uart_fd);
		}
	}
	exit(0);
}

#define S(x)    X(x,B##x,TO_STRING(x))
#define VALID_TERMINAL_SPEED_TABLE \
        S(0) \
        S(50) \
        S(75) \
        S(110) \
        S(134) \
        S(150) \
        S(200) \
        S(300) \
        S(600) \
        S(1200) \
        S(1800) \
        S(2400) \
        S(4800) \
        S(9600) \
        S(19200) \
        S(38400) \
        S(57600) \
        S(115200) \
        S(230400)

static inline speed_t check_speed_parameter(const char* value)
{
    speed_t speed = invalid_speed_parameter;
    const unsigned long int requested_speed = strtoul(value, NULL,0);
    printf("requested_speed = %d\n", requested_speed);
    
    switch(requested_speed) {
#define X(x,y,z) case x: return y;
        VALID_TERMINAL_SPEED_TABLE
    default:
        return invalid_speed_parameter;
#undef X
    }
}

static inline int parse_cmdLine(int argc, char *argv[])
{
    int error = EXIT_SUCCESS;
    int optc;

    while (((optc = getopt (argc, argv, "D:s:b:c:p:h")) != -1) && (EXIT_SUCCESS == error)) {
        //int param;
        switch (optc) {
        case 'D':
            strncpy(device, optarg, MAX_DEVICE_NAME);
            break;
        //case 't': {
        //    const int value = atoi(optarg);
        //    if (value >= 1) {
        //        timeout = value;
        //    } else {
        //        fprintf(stderr,"invalid argument (%d) to set initial time out value\n",value);
        //        error = EINVAL;
        //    }
        //    DEBUG_VAR(timeout,"%d");
        //}
        //break;
        case 's': {
            const speed_t requested_speed = check_speed_parameter(optarg);
            
            if (requested_speed != invalid_speed_parameter) {
                initial_speed = requested_speed;
            } else {
                APP_E("invalid speed parameters arguments");
            }
        }
        break;
		case 'b':
			initial_bits = atoi(optarg);
			break;
		case 'c':
			initial_event = 'N';
			break;
		case 'p':
			initial_stop = atoi(optarg);
			break;
		
        case 'h':
            usage(argv[0]);
            exit(error);
            break;
        default:
            error = EINVAL;
            fprintf(stderr,"invalid argument (%d)\n",optc);
            usage(argv[0]);
            exit(error); /* because of current design */
            break;
        } /* switch (optc) */

    } /* while (((optc = getopt_long (argc, argv, "D:S:Lt:fhv", longopts, NULL)) != -1) && (EXIT_SUCCESS == error)) */
    return error;
}

int build_fd_sets(fd_set *read_fds, fd_set *write_fds/*, fd_set *except_fds*/)
{
	FD_ZERO(read_fds);
	FD_SET(uart_fd, read_fds);

	FD_ZERO(write_fds);
#if TEST
	if(flag_w)
		FD_SET(uart_fd, write_fds);
#else
	FD_SET(uart_fd, write_fds);
#endif
	return 0;
}

int handle_uart_fd_read(char *buf)
{
	int read_sz = read(uart_fd, buf, MAX_READ_SZ);
	if(read_sz<=0){
		APP_E("read failed");
	}

	return read_sz;
}

int handle_uart_fd_write(void)
{
#if TEST
	return write(uart_fd, g_buffer, g_buffer_sz);
#else
	U8 write_buf[]="Hello, This is mipc-100!";
	return write(uart_fd, write_buf, sizeof(write_buf));
#endif
}

int main(int argc, char **argv)
{
	struct sigaction sact;
	if(argc<6){
		usage(argv[0]);
		exit(0);
	}
	
	if(0 != parse_cmdLine(argc,argv)){
		return errno;
	}
	
	APP_DDD("initial_speed %d", initial_speed);
	APP_DDD("initial_bits %d", initial_bits);
	APP_DDD("initial_event %c", initial_event);
	APP_DDD("initial_stop %d", initial_stop);
	
	if(0 != init_uart(O_RDWR | O_NOCTTY | O_SYNC, initial_speed, initial_bits, initial_event, initial_stop)){
		return errno;
	}

    sact.sa_handler = signal_handle;
    sigaction(SIGINT, &sact, NULL);
	sigaction(SIGKILL, &sact, NULL);

	fd_set read_fdset;
	fd_set write_fdset;
	//struct timeval tv;

	//while (1)
	{
        int ret_w=0, ret_r=0;
		int read_sz=0;
		int write_sz=0;
		U8 read_buf[1024+1];
		
		uint8_t read_chip_id[] = {0xAA, 0x01, 0x00, 0x01};
        uint8_t response[4];

        ret_w = write(uart_fd, read_chip_id, sizeof(read_chip_id));

	usleep ((4 + 25) * 100);
        ret_r = read(uart_fd, response, 4);
        
        if (response[0] == 0xBB && response[1] == 0x00) {
            printf("Chip ID: 0x%02X\n", response[2]);
        } else {
            printf("Read failed ret_w=%d,ret_r=%d \n", ret_w, ret_r);
        }
	}

	return 0;
}
