#include "pandora_client.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h> 
#include <setjmp.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <pthread.h>

enum {
	WAIT_FOR_READ,
	WAIT_FOR_WRITE,
	WAIT_FOR_CONN
};

#define DEFAULT_TIMEOUT	10	/*secondes waitting for read/write*/

#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <strings.h>

int sys_readn(int fd, void *vptr, int n)
{
		// printf("start sys_readn: %d....\n", n);
    int nleft, nread;
    char *ptr;

    ptr = vptr;
    nleft = n;
    while (nleft > 0) {
				// printf("start read\n");
        if ((nread = read(fd, ptr, nleft)) < 0) {
            if (errno == EINTR)
                nread = 0;
            else
                return -1;
        }
        else if (nread == 0)
            break;
				// printf("end read, read: %d\n", nread);
        nleft -= nread;
        ptr += nread;
    }
		// printf("stop sys_readn....\n");
		
    return n - nleft;
}

int sys_writen(int fd, const void *vptr, int n)
{
    int nleft;
    int nwritten;
    const char *ptr;

    ptr = vptr;
    nleft = n;
    while (nleft > 0) {
        if ( (nwritten = write(fd, ptr, nleft)) <= 0) {
            if (nwritten < 0 && errno == EINTR)
                nwritten = 0;   /* and call write() again */
            else
                return (-1);    /* error */
        }

        nleft -= nwritten;
        ptr += nwritten;
    }

    return n;
}


int tcp_open(const char *ipaddr, int port)
{
    int sockfd;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        return -1;

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, ipaddr, &servaddr.sin_addr) <= 0) {
        close(sockfd);
        return -1;
    }

    if (connect(sockfd, (struct sockaddr *)&servaddr, 
                sizeof(servaddr)) == -1) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

/**
 *Description:check the socket  state
 *
 * @param 
 * fd: socket
 * timeout:the time out of select
 * wait_for:socket state(r,w,conn)
 *
 * @return 1 if everything was ok, 0 otherwise
 */
static int select_fd (int fd, int timeout, int wait_for)
{
	fd_set fdset;
	fd_set *rd = NULL, *wr = NULL;
	struct timeval tmo;
	int result;

	FD_ZERO (&fdset);
	FD_SET (fd, &fdset);
	if (wait_for == WAIT_FOR_READ)
	{
		rd = &fdset;
	}
	if (wait_for == WAIT_FOR_WRITE)
	{
		wr = &fdset;
	}
	if(wait_for == WAIT_FOR_CONN)
	{
		rd = &fdset;
		wr = &fdset;
	}

	tmo.tv_sec =  timeout;
	tmo.tv_usec = 0;
	do {
		result = select (fd + 1, rd, wr, NULL, &tmo);
	} while (result < 0 && errno == EINTR);
	
	return result;
}

typedef struct _PandoraClient_s{
	pthread_mutex_t		cliSocketLock;
	int					cliSocket;

	pthread_t 			receiveTask;
	pthread_t 			heartBeatTask;

	int 				exit;
	char * ip;
	int port;

	CallBack 			callback;
	void*				userp;

	unsigned int 		position[PANDORA_CAMERA_UNIT];
	unsigned int 		startTimestamp[PANDORA_CAMERA_UNIT];
	PandoraPic* 		pics[PANDORA_CAMERA_UNIT]; 
}PandoraClient;

void PandoraClientTask(void* handle);
void PandoraClientHeartBeatTask(void* handle);
void parseHeader(char* header , int len , PandoraPicHeader* picHeader)
{
	int index = 0;
	picHeader->SOP[0] = header[index];
	picHeader->SOP[1] = header[index +1];
	index += 2;

	picHeader->pic_id = header[index];
	picHeader->type = header[index+1];
	index += 2;

	picHeader->width = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;
	index += 4;

	picHeader->height = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;
	index += 4;

	picHeader->timestamp = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;
	index += 4;

	picHeader->len = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;
	index += 4;

	picHeader->totalLen = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;

	index += 4;

	picHeader->position = (header[index + 0] & 0xff) << 24 | \
					(header[index + 1] & 0xff) <<  16  | \
					(header[index + 2] & 0xff) <<  8  | \
					(header[index + 3] & 0xff) <<  0  ;
	#ifdef UTC_TIME
	index+=4;
	picHeader->UTC_Time.UTC_Year=header[index++];
	picHeader->UTC_Time.UTC_Month=header[index++];
	picHeader->UTC_Time.UTC_Day=header[index++];
	picHeader->UTC_Time.UTC_Hour=header[index++];
	picHeader->UTC_Time.UTC_Minute=header[index++];
	picHeader->UTC_Time.UTC_Second=header[index];
	#endif
}
void* PandoraClientNew(const char* ip , int port , CallBack callback , void* userp)
{
	if(!ip || !callback || !userp )
	{
		printf("Bad Parameter\n");
		return NULL;
	}

	int ret = 0;

	PandoraClient *client = (PandoraClient*)malloc(sizeof(PandoraClient));
	if(!client)
	{
		printf("No Memory\n");
		return NULL;
	}

	memset(client , 0 , sizeof(PandoraClient));
	client->callback = callback;
	client->userp = userp;
	client->cliSocket = -1;
	client->ip = strdup(ip);
	client->port = port;
	
	pthread_mutex_init(&client->cliSocketLock , NULL);

	ret = pthread_create(&client->receiveTask , NULL , (void*)PandoraClientTask , (void*)client );
	if(ret != 0)
	{
		printf("Create Task Failed\n");
		free(client);
		return NULL;
	}

	ret = pthread_create(&client->heartBeatTask , NULL , (void*)PandoraClientHeartBeatTask , (void*)client );
	if(ret != 0)
	{
		printf("Create heart beat Task Failed\n");
		client->exit = 1;
		pthread_join(client->receiveTask , NULL);
		free(client);
		return NULL;
	}
	return (void*)client;
}

void PandoraClientDestroy(void* handle)
{
	PandoraClient *client = (PandoraClient*)handle;
	if(!client)
	{
		printf("Bad Parameter\n");
		return;
	}

	client->exit = 1;
	pthread_join(client->heartBeatTask , NULL);
	pthread_join(client->receiveTask, NULL);
	close(client->cliSocket);
	free(client);
}

void PandoraClientHeartBeatTask(void* handle)
{
	PandoraClient *client = (PandoraClient*)handle;
	if(!client)
	{
		printf("Bad Parameter\n");
		return;
	}
	int ret = 0;
	
	while(!client->exit)
	{
		ret = select_fd(client->cliSocket, 1, WAIT_FOR_WRITE);
		if (ret > 0)
		{
			ret = write(client->cliSocket , "HEARTBEAT" , strlen("HEARTBEAT"));
			if(ret < 0)
			{
				printf("Write Error\n");
			}

		}

		sleep(1);
	}
}



void PandoraClientTask(void* handle)
{
	PandoraClient *client = (PandoraClient*)handle;
	if(!client)
	{
		printf("Bad Parameter\n");
		return;
	}

	int connfd = client->cliSocket;

	while(!client->exit)
	{
		if (client->cliSocket == -1) {
			printf("Camera: Connecting......\n");
			connfd = tcp_open(client->ip ,client->port);
			if(connfd < 0)
			{
				printf("Connect to server failed\n");
				sleep(1);
				continue;
			}
			pthread_mutex_lock(&client->cliSocketLock);
            client->cliSocket = connfd;
			pthread_mutex_unlock(&client->cliSocketLock);
			printf("Camera: connect to server successfully!\n");
			struct timeval tv;
			tv.tv_sec = 5;  /* 5 Secs Timeout */
			tv.tv_usec = 0;  // Not init'ing this can cause strange errors
			setsockopt(client->cliSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));
			connfd = client->cliSocket;
		}


		int ret = 0;
		// printf("start select_fd for read\n");
		ret = select_fd(connfd, 1, WAIT_FOR_READ);
		if(ret == 0)
		{
			printf("select for read wrong\n");
			pthread_mutex_lock(&client->cliSocketLock);
			close(client->cliSocket);
			client->cliSocket = -1;
			pthread_mutex_unlock(&client->cliSocketLock);
			sleep(5);
			continue;
		}
		else if(ret > 0)
		{
			char header[64];
			int n = sys_readn(connfd , header , 2);
			if (n < 0)
			{
				printf("read header wrong!\n");
				pthread_mutex_lock(&client->cliSocketLock);
				close(client->cliSocket);
				client->cliSocket = -1;
				pthread_mutex_unlock(&client->cliSocketLock);
				continue;
			}

			if(header[0] != 0x47 || header[1] != 0x74)
			{
				printf("InValid Header SOP\n");
				printf("%02x %02x \n", header[0] , header[1]);
				exit(0);
				continue;
			}

			n = sys_readn(connfd , header + 2 , PANDORA_CLIENT_HEADER_SIZE - 2);
			if (n <= 0)
			{
				printf("read header2 wrong!\n");				
				pthread_mutex_lock(&client->cliSocketLock);
				close(client->cliSocket);
				client->cliSocket = -1;
				pthread_mutex_unlock(&client->cliSocketLock);
				continue;
			}
			PandoraPic* pic = (PandoraPic*)malloc(sizeof(PandoraPic));
			if(!pic)
			{
				printf("No Memory\n");
				continue;
			}

			parseHeader(header , n+2 , &pic->header);

			pic->yuv = malloc(pic->header.len);
			if(!pic->yuv)
			{
				printf("No Memory\n");
				free(pic);
				continue;
			}

			n = sys_readn(connfd , pic->yuv , pic->header.len);
			if (n <= 0)
			{
				printf("read pic_yuv wrong!\n");
				pthread_mutex_lock(&client->cliSocketLock);
				close(client->cliSocket);
				client->cliSocket = -1;
				pthread_mutex_unlock(&client->cliSocketLock);
				continue;
			}
			if(n != pic->header.len)
			{
				printf("picLength wrong\n");
				free(pic->yuv);
				free(pic);
				continue;
			}

			// check frame id 
			if(pic->header.pic_id < 0 || pic->header.pic_id >= PANDORA_CAMERA_UNIT)
			{
				free(pic->yuv);
				free(pic);
				// printf("Camera %d drop \n" , pic->header.pic_id);
				continue;
			}

			// check the desired frame postion packet.
			if(pic->header.position != client->position[pic->header.pic_id])
			{
				client->position[pic->header.pic_id] = 0;
				free(pic->yuv);
				free(pic);
				// printf("Camera %d drop \n" , pic->header.pic_id);
				continue;
			}

			if(!client->pics[pic->header.pic_id])
			{
				client->pics[pic->header.pic_id] = malloc(sizeof(PandoraPic));
				memcpy(client->pics[pic->header.pic_id] , pic , sizeof(PandoraPic));
				client->pics[pic->header.pic_id]->yuv = malloc(pic->header.totalLen);
			}

			memcpy(client->pics[pic->header.pic_id]->yuv + client->position[pic->header.pic_id] , 
					pic->yuv, pic->header.len);

			if(client->position[pic->header.pic_id] == 0)
			{
				client->startTimestamp[pic->header.pic_id] = pic->header.timestamp;
												
			}

			client->position[pic->header.pic_id] += pic->header.len;

			

			if(client->position[pic->header.pic_id] == pic->header.totalLen)
			{
				client->pics[pic->header.pic_id]->header.len = pic->header.totalLen;
				client->pics[pic->header.pic_id]->header.timestamp = pic->header.timestamp;
				client->pics[pic->header.pic_id]->header.UTC_Time = pic->header.UTC_Time;
				// a whole frame
				if(client->callback)
					client->callback(client , 0 , client->pics[pic->header.pic_id] , client->userp);
				// printf("camera[%d] write cost : %d-%d=%d us size %d \n", 
				// 	pic->header.pic_id ,pic->header.timestamp,client->startTimestamp[pic->header.pic_id] ,
				// 	 pic->header.timestamp - client->startTimestamp[pic->header.pic_id]
				// 	 , pic->header.len);
				client->pics[pic->header.pic_id] = NULL;
				client->position[pic->header.pic_id] = 0;
			}
			free(pic->yuv);
			free(pic);
		
		}
		else
		{
			printf("Read Error\n");
			pthread_mutex_lock(&client->cliSocketLock);
			close(client->cliSocket);
			client->cliSocket = -1;
			pthread_mutex_unlock(&client->cliSocketLock);
			continue;
		}
	}
}
