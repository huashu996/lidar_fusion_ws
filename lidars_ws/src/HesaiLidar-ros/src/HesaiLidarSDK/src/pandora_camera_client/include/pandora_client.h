#ifndef _PANDORA_CLIENT_H
#define _PANDORA_CLIENT_H

#ifdef __cplusplus
extern "C"
{
#endif

#define PANDORA_CAMERA_UNIT (5)

#define UTC_TIME
#ifdef UTC_TIME
typedef struct{
unsigned char UTC_Year;
unsigned char UTC_Month;
unsigned char UTC_Day;
unsigned char UTC_Hour;
unsigned char UTC_Minute;
unsigned char UTC_Second;
}UTC_Time_T; 
#endif
typedef struct _PandoraPicHeader_s{
	char SOP[2];
	unsigned char pic_id;
	unsigned char type;
	unsigned int width;
	unsigned int height;
	unsigned timestamp;
	unsigned len;
	unsigned int totalLen;
	unsigned int position;
	#ifdef UTC_TIME
	UTC_Time_T UTC_Time;
	#endif 
}PandoraPicHeader;

typedef struct _PandoraPic{
	PandoraPicHeader header;
	void* yuv;
}PandoraPic;
	#ifdef UTC_TIME
	#define PANDORA_CLIENT_HEADER_SIZE (34)
	#else
	#define PANDORA_CLIENT_HEADER_SIZE (28)
	#endif 


typedef int (*CallBack)(void* handle , int cmd , void* param , void* userp);

void* PandoraClientNew(const char* ip , int port , CallBack callback , void* userp);
void PandoraClientDestroy(void* handle);



#ifdef __cplusplus
}
#endif

#endif