#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <sys/stat.h>

typedef enum {
	NOHWFC = 'N',
	RTSCTS = 'R',
	DTRDSR = 'D',
	FULLFC = 'F'
} serial_flow_controll_t;

typedef enum {
	NOPARITY = 'N',
	PARITYODD = 'O',
	PARITYEVEN = 'E'
} serial_parity_t;

int serialfd = 0;
char *serialdev;

int openserial(char *devicename)
{
	int fd;
	if ((fd = open(devicename, O_RDWR)) == -1) {
		printf("openserial(): open(): %s\n", strerror(errno));
		return 0;
	}
	serialfd = fd;
	return fd;
}

void closeserial(int fd)
{
	tcdrain(fd);
//	tcsetattr(fd, TCSANOW, &oldterminfo);
	if (close(fd) < 0)
		printf("closeserial(): %s", strerror(errno));
}

speed_t serial_int_to_baudrate (int baudrate)
{
	switch (baudrate) {
	case 50     : return B50     ;
	case 75     : return B75     ;
	case 110    : return B110    ;
	case 134    : return B134    ;
	case 150    : return B150    ;
	case 200    : return B200    ;
	case 300    : return B300    ;
	case 600    : return B600    ;
	case 1200   : return B1200   ;
	case 1800   : return B1800   ;
	case 2400   : return B2400   ;
	case 4800   : return B4800   ;
	case 9600   : return B9600   ;
	case 19200  : return B19200  ;
	case 38400  : return B38400  ;
	case 57600  : return B57600  ;
	case 115200 : return B115200 ;
	case 230400 : return B230400 ;
	case 460800 : return B460800 ;
	case 500000 : return B500000 ;
	case 576000 : return B576000 ;
	case 921600 : return B921600 ;
	case 1000000: return B1000000;
	case 1152000: return B1152000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 2500000: return B2500000;
	case 3000000: return B3000000;
	case 3500000: return B3500000;
	case 4000000: return B4000000;
	}
	printf("serial_int_to_baudrate(): Error: not a valid baudrate!");
	return ~0;
}

int set_interface_attribs(int fd, int baudrate, int bits, int stopbits,
		serial_parity_t parity, serial_flow_controll_t hwfc, int vmin, int vtime)
{
    struct termios attr;
	printf(",%d,%d%c%d%c"
			,baudrate, bits, parity, stopbits, hwfc);
    if (tcgetattr(fd, &attr) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    speed_t br = serial_int_to_baudrate(baudrate);
    if (cfsetospeed(&attr, br) < 0){
    	printf("set_interface_attribs(): Error: not a valid baudrate!\n\r");
    	return -1;
    }
    if (cfsetispeed(&attr, br) < 0){
    	printf("set_interface_attribs(): Error: not a valid baudrate!\n\r");
		return -1;
    }

    attr.c_cflag |= (CLOCAL | CREAD);	//ignore modem controls
    attr.c_cflag &= ~CSIZE;
    switch (bits) {
		case 5: {attr.c_cflag |= CS5;}; break;// 5-bit characters
		case 6: {attr.c_cflag |= CS6;}; break;// 6-bit characters
		case 7: {attr.c_cflag |= CS7;}; break;// 7-bit characters
		case 8: {attr.c_cflag |= CS8;}; break;// 8-bit characters
		default: {printf("Error: not a valid bit length!"); return -1;}
    }
    switch (parity) {
		case NOPARITY: {attr.c_cflag &= ~PARENB;};break;
		case PARITYODD: {attr.c_cflag |= (PARENB | PARODD);};break;
		case PARITYEVEN: {attr.c_cflag |= PARENB; attr.c_cflag &= ~PARODD;};break;
		default: {
			printf("Error: not a valid Parity!");
			return -1;
		}
    }
    switch (stopbits) {
		case 1: {attr.c_cflag &= ~CSTOPB;};break;
		case 2: {attr.c_cflag |= CSTOPB;};break;
		default: {
			printf("Error: not a valid Stop bits number!");
			return -1;
		}
    }

    switch (hwfc) {
		case FULLFC:
		case RTSCTS: {attr.c_cflag |= CRTSCTS;}; break;
		case DTRDSR:
		case NOHWFC: {attr.c_cflag &= ~CRTSCTS;}; break;
		default: {
			printf("Error: not a valid Flow Control!");
			return -1;
		}
    }

    attr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    attr.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    attr.c_oflag &= ~OPOST;

    if ((vmin > 255) && (vmin < 0)) {
    	printf("Error: VMIN is out of range 0-255!");
    	return -1;
    }
    attr.c_cc[VMIN] = vmin;
    if ((vtime > 255) && (vtime < 0)) {
		printf("Error: VTIME is out of range 0-255!");
		return -1;
	}
    attr.c_cc[VTIME] = vtime;	//ten'ths of a second units (1->0.1seconds)
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &attr) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
	int fileok = 0, portok = 0;
	int result = -1;
	int argnum = 1;
	char *binfile = NULL;// = "binfile.txt";

	if (argc <= 2) {
		printf("usage:\r\nesp_boot -p /dev/ttyUSBx\n\r"); fflush(0);
		return 0;
	}
	while (argnum < argc) {
		if ( argv[argnum][0] == '-' ) {
			if (argv[argnum+1] == NULL) {
				printf("missing argument for option '-%c'\n\r", argv[argnum][1]);
				return -1;
			}
			switch (argv[argnum][1]) {
				case 'p': {
					serialdev = argv[argnum+1];
					printf("Port '%s'\n", serialdev); fflush(0);
					serialfd = openserial(serialdev);
					if (!serialfd) {
						printf("Error while initializing %s.\r\n", serialdev);
						return -1;
					}
//					set_interface_attribs(serialfd,
//							115200, 8, 1, NOPARITY, NOHWFC, 0, 1/*100mSeconds*/);
//					printf(" OK\r\n"); fflush(0);
					portok = 1;
				};
				break;
				default: {
					printf("wrong option\n\r");
					return -1;
				}
			}//switch (argv[1][1])
		} else {
			fprintf(stderr, "\r\nError: wrong parameter '%s'\n\r", argv[argnum]);
		}
//		printf("\r\nargnum = %d", argnum); fflush(0);
		argnum+=2;
	} // while

	usleep(10000);
	int DTR_flag = TIOCM_DTR;
	int RTS_flag = TIOCM_RTS;
	int flags = TIOCM_DTR | TIOCM_RTS;

	ioctl(serialfd, TIOCMBIC, &DTR_flag);
	usleep(10000);
	flags = TIOCM_DTR;
	ioctl(serialfd, TIOCMSET, &flags);

	close(serialfd);

	printf("Now you can 'cd your_project_name/build' and run:\n"
			"esptool.py --chip esp32 -p /dev/ttyUSB0 -b 460800 --before=no_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin 0x1000 bootloader/bootloader.bin 0x130000 your_project_name.bin\n");
	return 0;
}

