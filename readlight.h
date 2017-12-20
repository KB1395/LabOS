/*
    DOCUMENTATION
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/termios.h>
#include <sys/random.h>
#include <string.h>

int readlight(char* port, speed_t baud) {
    int fd = open(port, O_RDWR | O_NOCTTY);

    /* Set up the control structure */
    struct termios toptions;

    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 9600 baud both ways */
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* enable receiver */
    toptions.c_cflag |= CREAD | CLOCAL;
    /* Setting Time outs */                                       
    //toptions.c_cc[VMIN]  = 3; /* Read 10 characters */  
    //toptions.c_cc[VTIME] = 0;  /* Wait indefinitely */ 

    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);


    char buf[10] = '\0';

    /* Send byte to trigger Arduino to send string back */
    write(fd, "0", 1);
    /* Receive string from Arduino */
    char n = read(fd, buf, 10);
    /* insert terminating zero in the string */
    buf[n] = '\0';

    int value = strtonum(&buf, 0, 1023);

    return value;
}

int readlightmock(char* port, speed_t baud) {
    char buf[4];
    getrandom(&buf, 4, );

    return ((u_int)*buf) % 1024;
}