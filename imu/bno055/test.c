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

bno055_opmode_t op_mode = OPERATION_MODE_NDOF;

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

static int serialDataAvail(int fd)
{
    int result;
    if (ioctl (fd, FIONREAD, &result) == -1)
        return -1;

    return result;
}

static void serialFlush(const int fd)
{
    tcflush (fd, TCIOFLUSH);
}

#if 1

int send_serial_cmd(int serial_fp, uint8_t *cmd, int nCmdBytes, uint8_t *resp, bool ack)
{
    int nAttempts = 0;
    while (1)
    {
#ifdef DEBUG
        fprintf(stdout, "Serial Send: 0x");
        for (int i = 0; i < nCmdBytes; i++)
        {
            fprintf(stdout, "%02x", cmd[i]);
        }
        fprintf(stdout, "\n");
#endif

        // Flush the serial port
        serialFlush(serial_fp);

        // Send our command over the serial port
        if (write(serial_fp, cmd, nCmdBytes) == -1)
        {
            perror("send_serial_cmd() send error:");
            return -1;
        }

        usleep(3);//delay(3)

        // If we don't want an ack, then return
        if (!ack)
            return 1;

        // This is independent on read or write command,
        // we want to read the first two response bytes
        // and determine if there was a bus error or not.
        int respByteIdx = 0;
        while (serialDataAvail(serial_fp) && respByteIdx < 2)
        {
            if (read(serial_fp, &resp[respByteIdx], 1) == -1)
            {
                perror("send_serial_cmd() timeout waiting for ack:");
                return -1;
            }
            respByteIdx++;
        }

#ifdef DEBUG
        fprintf(stdout, "Serial Receive: 0x%02x%02x\n", resp[0], resp[1]);
#endif

        // If there is no bus error, return
        if (!(resp[0] == 0xEE && resp[1] == 0x07))
            return 1;

        // If there was a bus error, retry
        nAttempts += 1;
        if (nAttempts > MAX_CMD_SEND_ATTEMPTS)
        {
            fprintf(stderr, "Exceeded maximum attempts for acknowledgment\n");
            return -1;
        }
    }
}

int write_bytes(int fd, bno055_register_t addr, uint8_t *bytes, uint8_t nBytes, bool ack)
{
    // Build the write command with the following format:
    // Byte 1: Start Byte 0xAA
    // Byte 2: Write Command 0x00
    // Byte 3: Register Address
    // Byte 4: Number of bytes to be written
    // Byte 5 -> n + 4: Bytes to be written
    int cmdSize = 4 + nBytes;
    uint8_t cmd[cmdSize];

    cmd[0] = 0xAA;
    cmd[1] = 0x00;
    cmd[2] = addr & 0xFF;
    cmd[3] = nBytes & 0xFF;
    for (int i = 0; i < nBytes; i++)
    {
        cmd[4 + i] = bytes[i] & 0xFF;
    }

    // Send write command over serial, only allow for
    // 5 attempts (ignoring bus errors)
    uint8_t resp[2];
    if (send_serial_cmd(fd, cmd, cmdSize, resp, ack) == -1)
    {
        fprintf(stderr, "Error sending serial command\n");
        return -1;
    }

    // Check to make sure we received a valid response
    // if we are expecting an acknowledgment
    if (resp == NULL || (ack && (resp[0] != 0xEE || resp[1] != 0x01)))
    {
        fprintf(stderr, "Failed to write to register: 0x%02x%02x\n", resp[0], resp[1]);
        return -1;
    }

    return 1;
}

int write_byte(int fd, bno055_register_t addr, uint8_t byte, bool ack)
{
    uint8_t bytes[] = {(uint8_t)(byte & 0xFF)};
    if (write_bytes(fd, addr, bytes, 0x01, ack) == -1)
    {
        // wait and try again
        usleep(500);//delay(500);
        if (write_bytes(fd, addr, bytes, 0x01, ack) == -1)
        {
            return -1;
        }
    }

    return 1;
}

// Return 1 on success, -1 on error.
int read_bytes(int fd, bno055_register_t addr, uint8_t *bytes, uint8_t nBytes)
{
    // Build the read command with the following format:
    // Byte 1: Start Byte 0xAA
    // Byte 2: Read Command 0x01
    // Byte 3: Register Address
    // Byte 4: Number of bytes to be read
    printf("addr: 0x%02x, len: %d\n", addr, nBytes);
    uint8_t cmd[] = {0xAA, 0x01, (uint8_t)(addr & 0xFF), (uint8_t)(nBytes & 0xFF)};

    // Send read command over serial, only allow for
    // 5 attempts (ignoring bus errors)
    uint8_t resp[2];
    if (send_serial_cmd(fd, cmd, 4, resp, true) == -1)
    {
        fprintf(stderr, "Error sending serial command\n");
        return -1;
    }

    // Process the response we received
    if (resp[0] != 0xBB || resp[1] != nBytes)
    {
        fprintf(stderr, "Failed to read register: 0x%02x%02x\n", resp[0], resp[1]);
        return -1;
    }
    
    // Busy loop while we wait for data
    while (!serialDataAvail(fd)){}

    // Read nBytes of data
    if (read(fd, bytes, (int)nBytes) == -1)
    {
        perror("read_bytes() unable to read bytes:");
        return -1;
    }

    return 1;
}

uint8_t read_byte(int fd, bno055_register_t addr)
{
    uint8_t readByte[1];
    if (read_bytes(fd, addr, readByte, 0x01) == -1)
    {
        // Wait and try again
        usleep(500);//delay(500);
        if (read_bytes(fd, addr, readByte, 0x01) == -1)
        {
            fprintf(stderr, "Error reading single byte\n");
        }
    }

#ifdef DEBUG
    fprintf(stdout, "Byte read: 0x%02x\n", readByte[0]);
#endif

    // Ew no error checking
    return readByte[0];
}

int bno_get_calibration_status(int fd, uint8_t *cal)
{
    memset(cal, 0, 4);

    uint8_t calStatus = read_byte(fd, BNO055_CALIB_STAT_ADDR);

    cal[0] = (calStatus >> 6) & 0x03;
    cal[1] = (calStatus >> 4) & 0x03;
    cal[2] = (calStatus >> 2) & 0x03;
    cal[3] = calStatus & 0x03;

    return 1;
}

int bno_fully_calibrated(int fd)
{
    uint8_t cal[4];
    if (bno_get_calibration_status(fd, cal) == -1)
    {
        fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
        return -1;
    }

    switch (op_mode)
    {
    case OPERATION_MODE_ACCONLY:
        return (cal[2] == 3);
    case OPERATION_MODE_MAGONLY:
        return (cal[3] == 3);
    case OPERATION_MODE_GYRONLY:
    case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
        return (cal[1] == 3);
    case OPERATION_MODE_ACCMAG:
    case OPERATION_MODE_COMPASS:
        return (cal[2] == 3 && cal[3] == 3);
    case OPERATION_MODE_ACCGYRO:
    case OPERATION_MODE_IMUPLUS:
        return (cal[2] == 3 && cal[1] == 3);
    case OPERATION_MODE_MAGGYRO:
        return (cal[3] == 3 && cal[1] == 3);
    default:
        return (cal[0] == 3 && cal[1] == 3 && cal[2] == 3 && cal[3] == 3);
    }
}


int bno_set_mode(int fd, bno055_opmode_t mode)
{
    int success = write_byte(fd, BNO055_OPR_MODE_ADDR, mode & 0xFF, true);

    usleep(30);//delay(30);
    if (success == -1)
    {
        fprintf(stderr, "Error changing operating mode of BNO055\n");
        return -1;
    }

    return 1;
}



static int bno_get_calibration_data(int fd, bno055_offsets_t *offsets)
{
    uint8_t calData[22];
    
    if (bno_fully_calibrated(fd) != 1)
    {
        fprintf(stderr, "Unable to fetch calibration data, device is not fully calibrated\n");
        return -1;
    }

    memset(offsets, 0, sizeof(bno055_offsets_t));

    // Enter configuration mode
    if (bno_set_mode(fd, OPERATION_MODE_CONFIG) == -1)
    {
        fprintf(stderr, "Error changing op mode to config mode\n");
        return -1;
    }

    usleep(25);//delay(25);

    // Read the calibration data into a 22 byte array since the offset
    // data is stored at 22 contiguous bytes in memory
    if (read_bytes(fd, ACCEL_OFFSET_X_LSB_ADDR, calData, 22) == -1)
    {
        fprintf(stderr, "Unable to read calibration offset data\n");
        return -1;
    }

    // Must properly transfer data into bno055_offsets_t struct passed
    offsets->accel_offset_x = (((uint16_t)calData[1] << 8) | calData[0]) & 0xFFFF;
    offsets->accel_offset_y = (((uint16_t)calData[3] << 8) | calData[2]) & 0xFFFF;
    offsets->accel_offset_z = (((uint16_t)calData[5] << 8) | calData[4]) & 0xFFFF;
    offsets->mag_offset_x = (((uint16_t)calData[7] << 8) | calData[6]) & 0xFFFF;
    offsets->mag_offset_y = (((uint16_t)calData[9] << 8) | calData[8]) & 0xFFFF;
    offsets->mag_offset_z = (((uint16_t)calData[11] << 8) | calData[10]) & 0xFFFF;
    offsets->gyro_offset_x = (((uint16_t)calData[13] << 8) | calData[12]) & 0xFFFF;
    offsets->gyro_offset_y = (((uint16_t)calData[15] << 8) | calData[14]) & 0xFFFF;
    offsets->gyro_offset_z = (((uint16_t)calData[17] << 8) | calData[16]) & 0xFFFF;
    offsets->accel_radius = (((uint16_t)calData[19] << 8) | calData[18]) & 0xFFFF;
    offsets->mag_radius = (((uint16_t)calData[21] << 8) | calData[20]) & 0xFFFF;

    // Return to normal operation mode
    if (bno_set_mode(fd, op_mode) == -1)
    {
        fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
        return -1;
    }

    return 1;
}

static void bno_show_calibration_data(bno055_offsets_t *offsets)
{
    printf("acc_x: %u", offsets->accel_offset_x);
    printf("acc_y: %u", offsets->accel_offset_y);
    printf("acc_z: %u", offsets->accel_offset_z);
    
    printf("mag_x: %u", offsets->mag_offset_x);
    printf("mag_y: %u", offsets->mag_offset_y);
    printf("mag_z: %u", offsets->mag_offset_z);
    
    printf("gyro_x: %u", offsets->gyro_offset_x);
    printf("gyro_y: %u", offsets->gyro_offset_y);
    printf("gyro_z: %u", offsets->gyro_offset_z);
    
    printf("acc_rad: %u", offsets->accel_radius);
    printf("mag_rad: %u", offsets->mag_radius);
    
}
#endif

int main()
{
    const char *portname = "/dev/ttyUSB0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
            error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
            return 0;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking
    
    
#if 0
    int ret_w=0, ret_r=0;
    uint8_t read_chip_id[] = {0xAA, 0x01, 0x00, 0x01};
    uint8_t resp_h[2];
    uint8_t resp_data[1024];
    
    ret_w = write(fd, read_chip_id, sizeof(read_chip_id));
    ret_r = read(fd, resp_h, sizeof(resp_h));
    if(resp_h[0] == 0xBB){
        printf("ret_r=%d read len = %d\n", ret_r, resp_h[1]);
        read(fd, resp_data, resp_h[1]);
        printf("data: 0x%02X", resp_data[0]);
    }else{
        printf("fff\n");
    }
#else
    uint8_t cmd[1]={0x00};
    uint8_t len = 1;
    
    read_bytes(fd, BNO055_CHIP_ID_ADDR, cmd, 1);
    
    for(int i=0;i<len;i++){
        printf("%d ", cmd[i]);
    }
    
    bno055_offsets_t calOffsets;
    memset(&calOffsets, 0, sizeof(calOffsets));
    bno_get_calibration_data(fd, &calOffsets);
    bno_show_calibration_data(&calOffsets);
#endif
    
    return 0;
}
