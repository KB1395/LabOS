#include "stdio.h"
#include "stdlib.h"
#include "sys/ioctl.h"
#include "sys/fcntl.h"
#include "sys/termios.h"

int main (void) {
    /* open serial port */
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n", fd);

    /* wait for the Arduino to reboot */
    usleep(3500000);

    /* Set up the control structure */
    struct termios toptions;

    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 9600 baud both ways */
    cfsetispeed(&toptions, B9600);
    cfsetospeed(&toptions, B9600);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    char buf[10];

    while(1) {
        /* Send byte to trigger Arduino to send string back */
        write(fd, "0", 1);
        /* Receive string from Arduino */
        memset(buf, '\0', 10);
        char n = read(fd, buf, 10);
        /* insert terminating zero in the string */
        buf[n] = '\0';

        printf("%i bytes read, buffer contains: %s", n, buf);
        usleep(500000);
    }
    
}
