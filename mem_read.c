/** @file mem_read.c
 *
 * @author marco corvi <marco_corvi@geocities.com>
 * @date   mar 2003
 *
 * \brief read test for the mem_tty driver
 *
 */ 

#include <stdio.h>      // printf
#include <stdlib.h>     // exit
#include <sys/types.h>  // open
#include <sys/stat.h>   // open
#include <fcntl.h>      // open
#include <unistd.h>     // read close
#include <errno.h>      // errno
#include <sys/ioctl.h>  // ioctl


int 
main( int argc, char ** argv )
{
  int fd;
  char ch;
  int i;
  int ld;

  if ( argc <= 1 ) {
    fprintf(stderr, "Usage: %s <device>\n", argv[0] );
    exit(1);
  }

  fd = open( argv[1], O_RDONLY );
  if ( fd < 0 ) {
    fprintf(stderr, "Error: unable to open %s (errno %d)\n", argv[1], errno);
    exit( 1 );
  }

  ioctl( fd, TIOCGETD, &ld);
  printf("Line discipline %d \n", ld );

  for (i=0; i<1024; i++) {
    if ( read( fd, &ch, 1 ) <= 0) 
      break;
    fprintf(stderr, "%c", ch );
  }
  close( fd );
  exit( 0 );
}
