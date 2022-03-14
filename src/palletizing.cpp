#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string>    
#include <iostream> 
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <std_msgs/Int32.h>
#include <pthread.h>
#include "std_msgs/Float32MultiArray.h"  
  
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <mosquitto.h>
#include "cJSON.h"

#include <termios.h> 
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#define  Header_Frame 	0X7B	//	帧头
#define  Tail_Frame 	0X0A	//	帧尾

#define DEBUG		0
#define DEBUG_R		0

#define KEEP_ALIVE 60

std:: string usart_port;	//	通信端口
std:: string mqtt_host;	//	服务器地址
int mqtt_port;			//	MQTT端口号
float clip_pulse;	//	夹爪力度控制接口


//	MQTT信息对象
const char *mqtt_body[]={"name","dir","cmd_type","pos","Grasp_Status","EmStop_Status","Stop_Status","Reset_Status","Reset_x","Reset_y","error"};

/*
RS232通信报文
			帧头+数据帧+校验位+帧尾
Header_Frame	 1
CMD 		 1
X                4
Y                4
Z                4
U                4
CRC 		 1
Tail_Frame  	 1	
*/

 /*X轴数据*/
union X_DATA
{
    float x;
    uint8_t x_byte[4];
};

 /*Y轴数据*/
union Y_DATA
{
    float y;
    uint8_t y_byte[4];
};

 /*Z轴数据*/
union Z_DATA
{
	float z;
	uint8_t z_byte[4];
};

 /*U轴数据  未使用*/
union U_DATA
{
	float u;
	uint8_t u_byte[4];
};

 /*数据报文结构体*/
struct  palletizing
{
	unsigned char send_data[20];
    unsigned char cmd_type; 
	X_DATA x_data;
	Y_DATA y_data;
	Z_DATA z_data;
	U_DATA u_data;

}palletizing_data;

 /*接收报文结构体*/
struct  palletizing_recei
{
	unsigned char send_data[20];
        unsigned char cmd_type; 
	X_DATA x_data_curr;
	Y_DATA y_data_curr;
	Z_DATA z_data_curr;
	U_DATA u_data_curr;
	float X_Curr;
	float Y_Curr;
	float Z_Curr;
	float U_Curr;
	uint8_t GRAB_STATUS;
	uint8_t STOP_STATUS;

}recei_pall_data;

 /*MQTT通信数据结构体*/
typedef struct  MQTT
{
//	塔吊 岸吊
    char name[50];
    char dir[10];
    uint8_t cmd_type;
    float pos[4];
    uint8_t Grasp_Status;
    uint8_t EmStop_Status;
    uint8_t Stop_Status;
    uint8_t Reset_Status;
    uint8_t Reset_x;
    uint8_t Reset_y;
    char error[50];

}MQTT_DATA;

//	预留
typedef struct  MQTT_CAR
{
// AGV AUTOWAER
    char name[50];
    char dir[10];
    uint8_t ation;
    char error[50];

}CAR_MQTT;


void SEND_PALL_DATA(void);

//	ROS相关话题发布定义
ros::Publisher pos_publisher;
ros::Publisher grab_publisher;  //GRAB
serial::Serial Pall_Serial;

//	线程运行控制
int key_runing = 1;
char key = 0;
int Pall_recei_runing=1;
static struct termios initial_settings, new_settings;
static int peek_character = -1; 

//	RS232下发相关函数与变量定义
unsigned char receri_data[50]={0};
uint8_t CRC_Calculate(uint8_t *addr,uint8_t datalen);
void hg_car_ack(int action);
void pall_movie_limit(void);	// 对xyz进行软限位

//	MQTT通信相关变量定义
bool session = true;
int mqtt_sub_runing=0;

struct mosquitto *mosq = NULL;

MQTT_DATA    MQTT_PALL;
CAR_MQTT     CAR_MQTT_DATA;
float temp_pos[3]={0};

 /********************************
	功能：测试是否有键盘按键按下，若有则返回非零值，否则返回零
 ********************************/
int kbhit()
{
	char ch;
    int nread;
    if ( peek_character != -1 )
        return(1);
	//	TIME = 0
    new_settings.c_cc[VMIN] = 0;
    tcsetattr( 0, TCSANOW, &new_settings );
	//	TIME=0, MIN=0, read立即返回,如果有待处理的字符,它们就会被返回,如果没有,read调用返回0,且不读取任何字符
    nread = read( 0, &ch, 1 );

    new_settings.c_cc[VMIN] = 1;
	//	TIME=0, MIN=1,read一直等待,直到有1个字符可以读取,返回值是字符的数量.到达文件尾时返回0
    tcsetattr( 0, TCSANOW, &new_settings );
	//	成功捕获到输入
    if ( nread == 1 )
    {
        peek_character = ch;
        return(1);
    }

    return(0);
}

 /********************************
	功能：返回检测到的按键
 ********************************/
int readch()
{
    char ch;
    if ( peek_character != -1 )
    {
        ch = peek_character;
        peek_character = -1;
        return(ch);
    }

    read( 0, &ch, 1 );
    return(ch);
}

 /********************************
	功能：关闭键盘捕获
 ********************************/
void close_keyboard()
{
    tcsetattr( 0, TCSANOW, &initial_settings );
}

 /********************************
	功能：开启线程，监听键盘事件
 ********************************/
void *capture_keyvalue(void*)
{

   // 将自己设置为分离状态
   pthread_detach(pthread_self());  
   while(key_runing)
   {
        if (kbhit())
        {
            key = readch(); 
	    if(key == 3)
	    {
			close_keyboard();
			key_runing = 0;
  	    }
//	    else
//            	printf( "You put %c(%d)\n", key,key);

        }
 	ros::spinOnce(); 
	usleep(1000*5); 
    }	

     close_keyboard();	
     pthread_exit(NULL);
}

 /********************************
	功能：解析RS232报文数据，读取上传的坐标
 ********************************/
float Data_Analysis(unsigned char *buf, int axis)
{
	float data = 0.0f;
	//	解析X轴数据，从2字节开始，共四个字节
	if(axis == 1)   	//X
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.x_data_curr.x_byte[i] = buf[2+i];
		}
		data = recei_pall_data.x_data_curr.x;
	}
	//	解析Y轴数据，从6字节开始，共四个字节
	else if(axis == 2)	//Y
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.y_data_curr.y_byte[i] = buf[6+i];
		}
		data = recei_pall_data.y_data_curr.y;
	}	
	//	解析Z轴数据，从10字节开始，共四个字节
	else if(axis == 3)	//Z
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.z_data_curr.z_byte[i] = buf[10+i];
		}
		data = recei_pall_data.z_data_curr.z;
	}
	// U轴
	else if(axis == 4)  	//U
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.u_data_curr.u_byte[i] = buf[14+i];
		}
		data = recei_pall_data.u_data_curr.u;
	}

	return data;
}

 /********************************
	功能：开启线程不断读取RS232的数据
 ********************************/
void* read_data_thread(void *parameter)
{
		// 将线程状态改为unjoinable状态，确保资源的释放
        pthread_detach(pthread_self()); 
        std::string data;
        uint8_t ch[50]={0};
	
        Pall_Serial.flush();	//	等待串口数据发送结束
        while(Pall_recei_runing)
        {   
			if(Pall_Serial.available())
            {
	            usleep(10*1000); 
                data=Pall_Serial.read(Pall_Serial.available());
				int len = data.length();
	//			printf("\nlen:%d\r\n",data.length());

				if(len<5 || len>30)
				{
					Pall_Serial.flush();
					data.clear();
					memset(receri_data,0,50);	
					continue;	
				}

				for(int i=0; i<data.length(); i++)
					receri_data[i] = data[i];
				unsigned char crc =  CRC_Calculate(receri_data,data.length() -2 );
				if(crc != receri_data[data.length() -2] && data.length()>5 && data.length()<25)
				{
					//Pall_Serial.flush();
					data.clear();
					memset(receri_data,0,50);	
					continue;	
				}		

				recei_pall_data.cmd_type = receri_data[1];
				if(recei_pall_data.cmd_type  == 0x01)
				{

					std_msgs::Float32MultiArray PALL_POS;
					std_msgs::Int32 GRAB_STATUS;
					recei_pall_data.X_Curr = Data_Analysis(receri_data,1);
					recei_pall_data.Y_Curr = Data_Analysis(receri_data,2);
					recei_pall_data.Z_Curr = Data_Analysis(receri_data,3);
					recei_pall_data.U_Curr = Data_Analysis(receri_data,4);	
					
					recei_pall_data.GRAB_STATUS=receri_data[14];
					recei_pall_data.STOP_STATUS=receri_data[15];				
		
					temp_pos[0]=recei_pall_data.X_Curr;
					temp_pos[1]=recei_pall_data.Y_Curr;
					temp_pos[2]=recei_pall_data.Z_Curr;
										
					PALL_POS.data.push_back(recei_pall_data.X_Curr);
					PALL_POS.data.push_back(recei_pall_data.Y_Curr);
					PALL_POS.data.push_back(recei_pall_data.Z_Curr);
					pos_publisher.publish(PALL_POS);
					
					GRAB_STATUS.data = recei_pall_data.GRAB_STATUS;
					grab_publisher.publish(GRAB_STATUS);
					SEND_PALL_DATA();			//发布当前位置数据
					printf("\r实时坐标:X:%.2f Y:%.2f Z:%.2f GRAB_STATUS:%d",  
					recei_pall_data.X_Curr,recei_pall_data.Y_Curr,recei_pall_data.Z_Curr,GRAB_STATUS.data);
					fflush(stdout);
				}
#if DEBUG_R
					printf("------------------------------------------------------\r\n");
					for(int i=0; i<data.length(); i++)
					{
						printf("%#02X ",receri_data[i]);
					}	
					printf("\n------------------------------------------------------\r\n");
					
#endif
					data.clear();
					memset(receri_data,0,50);

			}
			//Pall_Serial.flush();
			usleep(10*1000); 
		}
}

 /********************************
	功能：CRC校验计算
	备注：各个字节异或之和，取低8位
 ********************************/
uint8_t CRC_Calculate(uint8_t *addr,uint8_t datalen)
{
    uint8_t i;
    uint8_t *p = addr; 	
    uint8_t crc = 0;
    for(i = 0; i < datalen; i++)
    {
        crc ^= *p++;
    }
    return crc;
}

/********************************
	功能：向RS232下发报文
	备注：无
********************************/
void send_pall_cmd()
{
	// XYZ限位
    pall_movie_limit();

    palletizing_data.send_data[1] = palletizing_data.cmd_type;
	//	赋值X数据，从2字节开始，共四个字节
    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+2] = palletizing_data.x_data.x_byte[i];  //X
    }
	//	赋值Y数据，从6字节开始，共四个字节
    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+6] = palletizing_data.y_data.y_byte[i];  //Y
    }
	//	赋值Z数据，从10字节开始，共四个字节
    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+10] = palletizing_data.z_data.z_byte[i];  //Z
    }  
	//	预留U轴
    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+14] = palletizing_data.u_data.u_byte[i];  //U
    }

    palletizing_data.send_data[18] = CRC_Calculate(palletizing_data.send_data,18);

    printf("\nset pos: x:%f y:%f z:%f\r\n",palletizing_data.x_data.x,palletizing_data.y_data.y,palletizing_data.z_data.z);
	
#if DEBUG
    printf("------------------------------------------------------\n");
    printf("send:");
    for(int i=0; i<20;i++)
    {
    	printf("%02X ",palletizing_data.send_data[i]);
    }  
    printf("\n------------------------------------------------------\n");
#endif

    Pall_Serial.write(palletizing_data.send_data,20);
}

/********************************
	功能：电爪   	1：闭合 0：松开
	备注：ROS接口回调函数
********************************/
void Grasp_Status_Back(const std_msgs::Int32::ConstPtr &msg)
{
	if(msg->data == 1)  		//吸
        palletizing_data.cmd_type = 0x05;
	else if(msg->data == 0)		//放
		palletizing_data.cmd_type = 0x06;
    send_pall_cmd();	
}

/********************************
	功能：1：开始 0：暂停
	备注：ROS接口回调函数
********************************/
void Pall_Running_Back(const std_msgs::Int32::ConstPtr &msg)
{
	if(msg->data == 1)  		//开始
		palletizing_data.cmd_type = 0x02;
	else if(msg->data == 0)		//暂停
		palletizing_data.cmd_type = 0x03;
    send_pall_cmd();	
}

/********************************
	功能：复位   1：复位
	备注：ROS接口回调函数
********************************/
void Pall_Reset_Back(const std_msgs::Int32::ConstPtr &msg)
{
	if(msg->data == 1)  		//回原点
        palletizing_data.cmd_type = 0x04;
    send_pall_cmd();
}

/********************************
	功能：坐标点设置
	备注：ROS接口回调函数
********************************/
void Pall_POS_Back(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	palletizing_data.x_data.x =  msg->data[0];        
	palletizing_data.y_data.y =  msg->data[1]; 
	palletizing_data.z_data.z =  msg->data[2]; 
	palletizing_data.u_data.u =  msg->data[3]; 

    palletizing_data.cmd_type = 0x00;
    send_pall_cmd();
}

/* 配置终端函数 */
void init_keyboard()
{
	// 初始化终端对应的termios结构,把当前终端接口变量的值写入initial_settings参数指向的结构
    tcgetattr( 0, &initial_settings );	 
    new_settings = initial_settings;

    new_settings.c_lflag &= ~ICANON;	//	允许canonical模式 
    new_settings.c_lflag &= ~ECHO;	// 回显所输入的字符 
    new_settings.c_lflag &= ~ISIG;	//	当接收到INTR/QUIT/SUSP/DSUSP字符，生成一个相应的信号 
    new_settings.c_cc[VMIN] = 1;	//	非canonical模式读操作的最少字符数 
    new_settings.c_cc[VTIME] = 0;	//	非canonical模式读操作超时（单位为1/10秒) 
	//	重新配置终端接口变量
    tcsetattr( 0, TCSANOW, &new_settings );	// 更改终端设置（立刻对值进行修改 ）
}

/********************************
	功能：向中控系统发送数据函数
	备注：MQTT通信
********************************/
void SEND_PALL_DATA(void)
{

	cJSON * root =  cJSON_CreateObject();
    cJSON * pall_pos =  cJSON_CreateObject();
	char *message = NULL;

	//CAR_Serial.write(); 向下位机发送指令 
	cJSON_AddItemToObject(root, "name", cJSON_CreateString("AD"));
	cJSON_AddItemToObject(root, "dir", cJSON_CreateString("ZK"));
	cJSON_AddItemToObject(root, "cmd_type", cJSON_CreateNumber(MQTT_PALL.cmd_type));

	pall_pos=cJSON_CreateFloatArray(temp_pos,3);
	cJSON_AddItemToObject(root, "pos", pall_pos);

	//cJSON_AddItemToObject(pos, "x", cJSON_CreateNumber(0));
	//cJSON_AddItemToObject(pos, "y", cJSON_CreateNumber(0));
	//cJSON_AddItemToObject(pos, "z", cJSON_CreateNumber(0));

	cJSON_AddItemToObject(root, "Grasp_Status", cJSON_CreateNumber(MQTT_PALL.Grasp_Status));
	cJSON_AddItemToObject(root, "EmStop_Status", cJSON_CreateNumber(MQTT_PALL.EmStop_Status));
	cJSON_AddItemToObject(root, "Stop_Status", cJSON_CreateNumber(MQTT_PALL.Stop_Status));
    cJSON_AddItemToObject(root, "Reset_Status", cJSON_CreateNumber(MQTT_PALL.Reset_Status));
	cJSON_AddItemToObject(root, "Reset_x", cJSON_CreateNumber(MQTT_PALL.Reset_x));
	cJSON_AddItemToObject(root, "Reset_y", cJSON_CreateNumber(MQTT_PALL.Reset_y));

	cJSON_AddItemToObject(root, "error", cJSON_CreateString("null"));
	message = cJSON_PrintUnformatted(root);
	//printf("message:%s\r\n",message);

	//	发布mqtt消息
	mosquitto_publish(mosq,NULL,"/HG_DEV/AD_MSG",strlen(message),message,0,0); 

	cJSON_Delete(root);	
	if(message != NULL)
	{	
	    //cJSON_free(message);
	}

}

/********************************
	功能：解析中控系统数据
	备注：MQTT通信
********************************/
//	以递归的方式打印json的最内层键值对
void ZK_MSG_DATA(cJSON *mqtt_root)
{
	cJSON * root = mqtt_root;
    cJSON *Item = NULL;
    char *msg = NULL;
/*
        for(int i=0; i<3;i++)  //数据完整性校验
        {
	        Item = cJSON_GetObjectItem(root,mqtt_body[i]);
	        if(Item == NULL)
	                 goto skip;	
        }       
*/
	Item = cJSON_GetObjectItem(root,"name");
	strcpy(MQTT_PALL.name,Item->valuestring);   //执行响应的动作

	Item = cJSON_GetObjectItem(root,"dir");
	strcpy(MQTT_PALL.dir,Item->valuestring);

	Item = cJSON_GetObjectItem(root,"cmd_type");
	MQTT_PALL.cmd_type=Item->valueint;
	//	根据指令类型，做相关处理
    if(MQTT_PALL.cmd_type ==  0)
    {
		Item = cJSON_GetObjectItem(root,"pos");
        if(Item != NULL)
        {               
			cJSON *mqtt_pos = NULL;
            int  array_size  = cJSON_GetArraySize (Item);
			//	遍历最外层json键值对
            for(int i = 0;  i<array_size; i++)
            {
				mqtt_pos=cJSON_GetArrayItem(Item,i);
                MQTT_PALL.pos[i] = mqtt_pos->valuedouble;
            }	
        }
        palletizing_data.x_data.x = MQTT_PALL.pos[0];
        palletizing_data.y_data.y = MQTT_PALL.pos[1];
        palletizing_data.z_data .z= MQTT_PALL.pos[2];
        palletizing_data.cmd_type = 0x00;
        send_pall_cmd();
    }

    else  if(MQTT_PALL.cmd_type ==  1)
    {
        Item = cJSON_GetObjectItem(root,"Grasp_Status");
        MQTT_PALL.Grasp_Status = Item->valueint;
        if( MQTT_PALL.Grasp_Status == 1)
        {
			palletizing_data.cmd_type = 0x05;
            send_pall_cmd();     
        }
		else if(MQTT_PALL.Grasp_Status == 0) 
        {
			palletizing_data.cmd_type = 0x06;
            send_pall_cmd();    
        }
    }
    else  if(MQTT_PALL.cmd_type ==  2)
    {
		Item = cJSON_GetObjectItem(root,"EmStop_Status");
        MQTT_PALL.EmStop_Status = Item->valueint;
        if(MQTT_PALL.EmStop_Status == 1)
        {
			MQTT_PALL.EmStop_Status = Item->valueint;
        }
		else if(MQTT_PALL.EmStop_Status == 0)
        {
			MQTT_PALL.EmStop_Status = Item->valueint;
        }
    }
	//	暂停与开始指令
	else  if(MQTT_PALL.cmd_type ==  3)
    {
		Item = cJSON_GetObjectItem(root,"Stop_Status");
		MQTT_PALL.Reset_Status = Item->valueint;
		if(MQTT_PALL.Reset_Status == 1)  		//开始
		{
			MQTT_PALL.Reset_Status = 1; 
            palletizing_data.cmd_type = 0x02;
            send_pall_cmd();       
        }
        else if(MQTT_PALL.Reset_Status == 0)		//暂停
        {
			MQTT_PALL.Reset_Status = 0; 
            palletizing_data.cmd_type = 0x03;
            send_pall_cmd();       
        }
    } 
	else  if(MQTT_PALL.cmd_type ==  4)
    {
		Item = cJSON_GetObjectItem(root,"Reset_Status");
		MQTT_PALL.Reset_Status = Item->valueint;
		if(MQTT_PALL.Reset_Status == 1)  		//回原点
		{
			palletizing_data.cmd_type = 0x04;
			palletizing_data.x_data.x = 0.0; 
			palletizing_data.y_data.y = 0.0;
			palletizing_data.z_data.z = 0.0;                    
			send_pall_cmd();
        }
	} 
	else  if(MQTT_PALL.cmd_type ==  5)
    {
		Item = cJSON_GetObjectItem(root,"Reset_x");
		MQTT_PALL.Reset_x = Item->valueint;
		if(MQTT_PALL.Reset_x  == 1)
		{
			palletizing_data.cmd_type = 0x07;
			palletizing_data.x_data.x = 0.0; 	
			send_pall_cmd();
        }
	} 
    else  if(MQTT_PALL.cmd_type ==  6)
    {
		Item = cJSON_GetObjectItem(root,"Reset_y");
		MQTT_PALL.Reset_y = Item->valueint;       
        if(MQTT_PALL.Reset_y == 1)
        {
			palletizing_data.cmd_type = 0x08;	
            palletizing_data.y_data.y = 0.0; 
            send_pall_cmd(); 
        }
    }
                
    Item = cJSON_GetObjectItem(root,"error");
	strcpy(MQTT_PALL.error,Item->valuestring);
	msg = cJSON_Print(root);
	if(msg != NULL)
	{	
		printf("\r\nmessage:%s\r\n",msg);    
		cJSON_free(msg);
	}
	//返回数据
//skip:
}

/********************************
	功能：与AGV通信 预留
	备注：MQTT通信
********************************/
void AGV_MSG_DATA(cJSON *mqtt_root)
{
    cJSON * root = mqtt_root;
    char action[50]={0};
   	cJSON *Item = NULL;
    char *msg = NULL;

	Item = cJSON_GetObjectItem(root,"ation");
	CAR_MQTT_DATA.ation=Item->valueint;

	Item = cJSON_GetObjectItem(root,"error");
	strcpy(CAR_MQTT_DATA.error,Item->valuestring);

	msg=cJSON_Print(root);
	printf("\r\nmessage:%s\r\n",msg); 

	if(CAR_MQTT_DATA.ation == 1 )	//上料
	{
   	    sleep(3);
	    hg_car_ack(CAR_MQTT_DATA.ation);
	
	}
	else if(CAR_MQTT_DATA.ation == 0)   //下料
	{
  	    sleep(3);
	    hg_car_ack(CAR_MQTT_DATA.ation);
	}
			
	cJSON_Delete(root);
	cJSON_free(msg);

}

/********************************
	功能：与新能源小车通信 预留
	备注：MQTT通信
********************************/
void AUTO_MSG_DATA(cJSON *mqtt_root)
{
    cJSON * root = mqtt_root;
    cJSON *Item = NULL;
    char *msg = NULL;

	cJSON_Delete(root);
	cJSON_free(msg);
}

/********************************
	功能：获取MQTT数据信息
	备注：MQTT通信
********************************/
void car_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char name[50]={0};
    char dir[50]={0};
    cJSON *root = NULL;
    cJSON *Item = NULL;

    if(message->payloadlen)
    {	
		root=cJSON_Parse((char *)message->payload);
		Item = cJSON_GetObjectItem(root,"name");
		if(Item == NULL)
		   goto skip;

		strcpy(name,Item->valuestring);
		if(!strcmp(name,"TD"))
		   goto skip;
		
		Item = cJSON_GetObjectItem(root,"dir");
		if(Item == NULL)
		   goto skip;

		strcpy(dir,Item->valuestring);
		
		// 当dir字符串与"AD"相同时，strcmp（）函数返回零，取反为1
		if(!strcmp(dir,"AD"))
		{
			if(!strcmp(name,"ZK"))	//来自中控系统消息
			{
				//	解析中控系统数据
				ZK_MSG_DATA(root);
			}
	/*
			else if(!strcmp(name,"AGV"))
			{
				AGV_MSG_DATA(root);
			}else if(!strcmp(name,"Auto"))
			{
				AUTO_MSG_DATA(root);

			}
	*/
		}
		
		skip:
			cJSON_Delete(root);
    }
    else
    {
        printf("%s (null)\n", message->topic);
    }
	
}
 
 /********************************
	功能：AGV小车通信赋值(预留)
	备注：MQTT通信
********************************/
void hg_car_ack(int action)
{

	cJSON * root =  cJSON_CreateObject();
	char *message = NULL;

	cJSON_AddItemToObject(root, "name", cJSON_CreateString("TD"));
	cJSON_AddItemToObject(root, "dir", cJSON_CreateString("AGV"));
	cJSON_AddItemToObject(root, "ation", cJSON_CreateNumber(action));
	cJSON_AddItemToObject(root, "error", cJSON_CreateString("null"));
	message = cJSON_PrintUnformatted(root);
	//printf("message:%s\r\n",message);
	mosquitto_publish(mosq,NULL,"/HG_CAR/CAR_MSG",strlen(message),message,1,0); 

	cJSON_Delete(root);
	cJSON_free(message);
}

 /********************************
	功能：订阅岸吊MQTT数据话题
	备注：MQTT通信
********************************/
void car_connect(struct mosquitto *mosq, void *userdata, int result)
{
    int mid=0;
    if(!result)
    {
        mosquitto_subscribe(mosq, NULL, "/HG_DEV/AD_MSG", 1); //topic 主题："/HG_DEV/AD_MSG"
       // mosquitto_subscribe(mosq, NULL, "/HG_CAR/CAR_MSG", 1); //topic 主题："/HG_CAR/CAR_MSG"
    }
    else
    {
        fprintf(stderr, "\n MQTT请求连接失败,请检查服务器状态...\n");
    }
}
 
/********************************
	功能：MQTT回调函数
	备注：MQTT通信（没有用上）
********************************/
void subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;
    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}
 
 /********************************
	功能：MQTT日志回调函数
	备注：MQTT通信（预留）
********************************/
void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    /* Pring all log messages regardless of level. */
    printf("%s\n", str);
}
 
/********************************
	功能：开启一个线程，循环处理网络消息
	备注：MQTT通信
********************************/ 
void* mqtt_data_thread(void *parameter)
{
		pthread_detach(pthread_self()); 
        while(mqtt_sub_runing)
        {
	        //	无限阻塞循环调用loop（），mosq:mosq客户端，timeout：-1为使用默认值为1000ms，max_packets：1为未使用
	    	mosquitto_loop_forever(mosq, -1, 1);
	       //	网络事件循环处理函数，通过创建新的线程不断调用mosquitto_loop() 函数处理网络事件，不阻塞
	       int loop = mosquitto_loop_start(mosq);	// 成功时返回MOSQ_ERR_SUCCESS 
	       if(loop != MOSQ_ERR_SUCCESS)
	       {
			printf("mosquitto loop error\n");
	       }
		ros::spinOnce();
	    usleep(1000*5);
        }
}

 /********************************
	功能：向232发送数据进行软限位
	备注：安全配置
********************************/
void pall_movie_limit(void)
{

	if( palletizing_data.x_data.x >750 )  
	    palletizing_data.x_data.x = 750;
	else if( palletizing_data.x_data.x < -100)
            palletizing_data.x_data.x = -100;

	if(palletizing_data.y_data.y > 750) 
	    palletizing_data.y_data.y = 750;
	else if(palletizing_data.y_data.y < -200)
	    palletizing_data.y_data.y = -200;

	if(palletizing_data.z_data.z > 850) 
	    palletizing_data.z_data.z = 850;
	else if(palletizing_data.z_data.z < 0)
	    palletizing_data.z_data.z = 0;
}

 /********************************
	功能：主函数
	备注：无
********************************/
int main(int argc,char **argv)
{
        ros::init(argc, argv, "Palletizing_NODE"); 

        static int gruab = 1;
        ros::NodeHandle n;

        ros::NodeHandle nh("~");        
        std:: string usart_port;

        grab_publisher = n.advertise<std_msgs::Int32>("/Pall_GRAB_STATUS", 1);  //	发布抓取状态
        pos_publisher = n.advertise<std_msgs::Float32MultiArray>("/Pall_CURR_POS", 1);  //	发布当前位姿话题
        ros::Subscriber RUNNING_SUB = n.subscribe("/Pall_Running_Topic", 5,&Pall_Running_Back);	// 开始和暂停控制话题
        ros::Subscriber Grasp_SUB = n.subscribe("/Pall_Grasp_Topic", 5,&Grasp_Status_Back);
        ros::Subscriber RESET_SUB = n.subscribe("/Pall_Reset_Topic", 5,&Pall_Reset_Back);
        ros::Subscriber PALL_POS_SUB = n.subscribe("/Pall_POS_SET", 5,&Pall_POS_Back);	//topic


        nh.param<std::string>("usart_port", usart_port,  "/dev/ttyUSB0");
        nh.param<std::string>("mqtt_host",  mqtt_host,  "192.168.10.124");
        nh.param<int>("mqtt_port",  mqtt_port,  50002);

        nh.param<float>("clip_pulse",  clip_pulse,  2000);  

		//	串口连接
        try
        {
                Pall_Serial.setPort(usart_port);
                Pall_Serial.setBaudrate(115200);

                serial::Timeout timeout=serial::Timeout::simpleTimeout(2000);
                Pall_Serial.setTimeout(timeout);
                Pall_Serial.open();
        }
        catch (serial::IOException& e)
        {
                 ROS_ERROR_STREAM("open Palletizing falied!"); 
        }
	
		init_keyboard();

		//	新创建的线程ID指向的内存单元
        pthread_t 	recei_thread,input_key,mqtt_thread;

		//	创建线程不断读取RS232的数据
        pthread_create(&recei_thread, NULL, read_data_thread, NULL);

		//	创建线程线程监听键盘事件
		pthread_create(&input_key, NULL, capture_keyvalue, NULL);

		// 初始化mosquitto库函数
    	mosquitto_lib_init();  							//libmosquitto 库初始化   	
    	// 新建mosquitto客户端，生成一个随机客户端ID
		mosq = mosquitto_new(NULL,session,NULL);  				//创建mosquitto客户端
    	if(!mosq)
		{
        	printf("创建 MQTT 连接失败..\n");
        	mosquitto_lib_cleanup();
        	return 1;
    	}
    	
    	//mosquitto_log_callback_set(mosq, my_log_callback);  			//设置回调函数，需要时可使用
 
		//	连接确认car_connect回调函数，订阅岸吊mqtt消息
    	mosquitto_connect_callback_set(mosq, car_connect);

		//	连接确认car_message回调函数，处理mosq客户端数据
    	mosquitto_message_callback_set(mosq, car_message);

    	//mosquitto_subscribe_callback_set(mosq, subscribe_callback);
		//	连接mqtt
    	if(mosquitto_connect(mosq, mqtt_host.c_str(), mqtt_port, KEEP_ALIVE))  			//客户端连接服务器
        {   
        	fprintf(stderr, "\nUnable to connect.\n");
        	printf("MQTT 连接失败...\n");
        	return 1;
    	}
	
		mqtt_sub_runing = 1;	//	循环处理mqtt数据标志

		//	创建线程循环处理网络消息
        pthread_create(&mqtt_thread, NULL, mqtt_data_thread, NULL); 

		palletizing_data.send_data[0] = Header_Frame;	// 帧头
		palletizing_data.send_data[19] = Tail_Frame;	// 帧尾
 
		palletizing_data.x_data.x = clip_pulse;	//	夹爪力度控制接口
        palletizing_data.cmd_type = 0x01;
        send_pall_cmd();	
		usleep(1000*500);
		palletizing_data.x_data.x = recei_pall_data.X_Curr;        
		palletizing_data.y_data.y = recei_pall_data.Y_Curr;
		palletizing_data.z_data.z = recei_pall_data.Z_Curr;
		palletizing_data.u_data.u = recei_pall_data.U_Curr;

        unsigned int curr_count = 0;
		while(ros::ok())
        {
			/**************************************************
			a: x-
			d: x+
			w: y+
			s: y-
			q: z+
			e: z-
			f: 吸/放
			1： x复位
			2： y复位
			3： 退出键盘控制
			***************************************************/
			switch(key)
			{
				case 'a': 	
							palletizing_data.x_data.x -= 1.30;
        		    		palletizing_data.cmd_type = 0x00;
					  		send_pall_cmd();
					  		break;

				case 'd':   
							palletizing_data.x_data.x += 1.30;
			       			palletizing_data.cmd_type = 0x00;   
                            send_pall_cmd();	
			       			break;

		  		case 'w':   
				  			palletizing_data.y_data.y += 1.30;
 			       			palletizing_data.cmd_type = 0x00;  
                            send_pall_cmd();  	
			       			break;

		  		case 's':   
				  			palletizing_data.y_data.y -= 1.30;  
			      			palletizing_data.cmd_type = 0x00;	
                            send_pall_cmd();  
			      			break;

		  		case 'q':   
				  			palletizing_data.z_data.z += 1.5;
 			       			palletizing_data.cmd_type = 0x00;  
                            send_pall_cmd();  	
			       			break;

		  		case 'e':   
				  			palletizing_data.z_data.z -= 1.5; 
			      			palletizing_data.cmd_type = 0x00;	
                            send_pall_cmd();  
			      			break;

		  		case 'f':   
				  			if(gruab)
			      				palletizing_data.cmd_type = 0x05;
			      			else 
		      	        		palletizing_data.cmd_type = 0x06;
			      			gruab = !gruab;		
                            send_pall_cmd();  
			      			break;

		  		case '1':   
				  			palletizing_data.cmd_type = 0x07;	
                            send_pall_cmd();
			      			palletizing_data.x_data.x = 0.0;  
			      			break;

		  		case '2':   
				  			palletizing_data.cmd_type = 0x08;	
                            send_pall_cmd(); 
                            palletizing_data.y_data.y = 0.0;   
			      			break;

		  		case  3: 
				  			goto exit;break;
		 
		  		default:
/*
							palletizing_data.x_data.x = recei_pall_data.X_Curr;        
							palletizing_data.y_data.y = recei_pall_data.Y_Curr;
							palletizing_data.z_data.z = recei_pall_data.Z_Curr;
							palletizing_data.u_data.u = recei_pall_data.U_Curr; 
*/
							break;

	     	}
/*		//将坐标做负反馈

           if(key == 0)
           {
	     if(curr_count % 10 == 0)  //20x50ms = 1s
	     {
			 palletizing_data.x_data.x = recei_pall_data.X_Curr;        
		         palletizing_data.y_data.y = recei_pall_data.Y_Curr;
		         palletizing_data.z_data.z = recei_pall_data.Z_Curr;
		         palletizing_data.u_data.u = recei_pall_data.U_Curr; 
	     }
             curr_count++;
           }
	   else curr_count = -1;		
 */

            key = 0;
			usleep(1000*45);
			ros::spinOnce();
        }

//资源回收	
exit:
        Pall_recei_runing = 0;
        key_runing = 0;
		// 释放mqtt客户端
		mosquitto_destroy(mosq);
    	mosquitto_lib_cleanup();
    	close_keyboard();
		Pall_Serial.close(); 
        return 0;
}



