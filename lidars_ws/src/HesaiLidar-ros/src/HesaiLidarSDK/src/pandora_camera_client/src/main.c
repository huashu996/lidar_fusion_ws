#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include "pandora_client.h"

int imgCounter = 0;

int callback(void* handle , int cmd , void* param , void* userp)
{
	if(!handle  || ! userp || !param)
	{
		printf(" Bad Parameter\n");
		return -1;
	}

	PandoraPic *pic = (PandoraPic*)param;
       printf("frame[%d] pos: Y:%d M:%d D:%d H:%d M:%d S:%d usec: %d \n", 
       pic->header.pic_id,
       pic->header.UTC_Time.UTC_Year , 
       pic->header.UTC_Time.UTC_Month ,
       pic->header.UTC_Time.UTC_Day , 
       pic->header.UTC_Time.UTC_Hour , 
       pic->header.UTC_Time.UTC_Minute , 
       pic->header.UTC_Time.UTC_Second , 
       pic->header.timestamp);


       // if(pic->header.pic_id != 0)
       // {
       //      // printf("frame[%d] pos: %d and %d of %d time :%d \n", pic->header.pic_id , pic->header.position ,pic->header.len , pic->header.totalLen , pic->header.timestamp);
       //      // printf("OK , There is a picture , %d\n" , pic->header.len);
       //      char filename[256];
       //      sprintf(filename, "%d.jpg", ++imgCounter);
       //      int fd = open(filename, O_RDWR | O_CREAT, 0666);
       //      // printf("tail: %02x\n", (char)((char*)pic->yuv)[pic->header.len - 1]);
       //      write(fd , pic->yuv, pic->header.len );
       //      close(fd);
       // }

	
	free(pic->yuv);
	free(pic);
}


int main(int argc , char**argv)
{
	struct sigaction sa;
	sa.sa_handler = SIG_IGN;//设定接受到指定信号后的动作为忽略
	sa.sa_flags = 0;
	if (sigemptyset(&sa.sa_mask) == -1 ||  
		sigaction(SIGPIPE, &sa, 0) == -1) {  
		perror("failed to ignore SIGPIPE; sigaction");
		exit(-1);
	}

	void* handle = PandoraClientNew(argv[1] , atoi(argv[2]) , callback , (void*)1);
	if(!handle)
	{
		printf("Client Create Failed\n");
		return -1;
	}

	while(1)
	{
		sleep(1);
	}
}