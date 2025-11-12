#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc,char **argv)
{
	int fd;
	unsigned char buf[1];

	if(argc != 2){
		printf("the argc is %d\n",argc);
		return -1;
	}
	fd = open("/dev/led",O_RDWR);
	if(fd == -1){
		printf("can not find the /dev/led\n");
		return -1;
	}

	buf[0] = atoi(argv[1]);
	printf("%c\n",buf[0]);
	
	write(fd,buf,1);

	close(fd);
	return 0;
	

}




