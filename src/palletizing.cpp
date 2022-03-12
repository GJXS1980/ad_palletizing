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

#define  Header_Frame 	0X7B
#define  Tail_Frame 	0X0A

#define DEBUG		0
#define DEBUG_R		0

#define KEEP_ALIVE 60

std:: string usart_port;
std:: string mqtt_host;
int mqtt_port;
float clip_pulse;

const char *mqtt_body[]={"name","dir","cmd_type","pos","Grasp_Status","EmStop_Status","Stop_Status","Reset_Status","Reset_x","Reset_y","error"};

/*
Header_Frame	 1
CMD 		 1
X                4
Y                4
Z                4
U                4
CRC 		 1
Tail_Frame  	 1	
*/
 
union X_DATA
{
    float x;
    uint8_t x_byte[4];
};

union Y_DATA
{
    float y;
    uint8_t y_byte[4];
};
union Z_DATA
{
	float z;
	uint8_t z_byte[4];
};
union U_DATA
{
	float u;
	uint8_t u_byte[4];
};

struct  palletizing
{
	unsigned char send_data[20];
        unsigned char cmd_type; 
	X_DATA x_data;
	Y_DATA y_data;
	Z_DATA z_data;
	U_DATA u_data;

}palletizing_data;

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

typedef struct  MQTT
{
//塔吊 岸吊
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

typedef struct  MQTT_CAR
{
// AGV AUTOWAER
    char name[50];
    char dir[10];
    uint8_t ation;
    char error[50];

}CAR_MQTT;


void SEND_PALL_DATA(void);

ros::Publisher pos_publisher;
ros::Publisher grab_publisher;  //GRAB
serial::Serial Pall_Serial;
int key_runing = 1;
char key = 0;
int Pall_recei_runing=1;
static struct termios initial_settings, new_settings;
static int peek_character = -1; 

unsigned char receri_data[50]={0};
uint8_t CRC_Calculate(uint8_t *addr,uint8_t datalen);
void hg_car_ack(int action);
void pall_movie_limit(void);

bool session = true;
int mqtt_sub_runing=0;

struct mosquitto *mosq = NULL;

MQTT_DATA    MQTT_PALL;
CAR_MQTT     CAR_MQTT_DATA;
float temp_pos[3]={0};

int kbhit()
{
    char ch;

    int nread;

    if ( peek_character != -1 )

        return(1);

    new_settings.c_cc[VMIN] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );

    nread = read( 0, &ch, 1 );

    new_settings.c_cc[VMIN] = 1;

    tcsetattr( 0, TCSANOW, &new_settings );

    if ( nread == 1 )
    {
        peek_character = ch;

        return(1);
    }

    return(0);
}

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

void close_keyboard()
{
    tcsetattr( 0, TCSANOW, &initial_settings );
}

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

float Data_Analysis(unsigned char *buf, int axis)
{
	
	float data = 0.0f;

	if(axis == 1)   	//X
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.x_data_curr.x_byte[i] = buf[2+i];
		}
		data = recei_pall_data.x_data_curr.x;
	}
	else if(axis == 2)	//Y
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.y_data_curr.y_byte[i] = buf[6+i];
		}
		data = recei_pall_data.y_data_curr.y;
	}	
	else if(axis == 3)	//Z
	{
		for(int i = 0; i<4; i++)
		{
			recei_pall_data.z_data_curr.z_byte[i] = buf[10+i];
		}
		data = recei_pall_data.z_data_curr.z;
	}
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


void* read_data_thread(void *parameter)
{
        pthread_detach(pthread_self()); 
        std::string data;
        uint8_t ch[50]={0};
	
        Pall_Serial.flush();
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
			}	printf("\n------------------------------------------------------\r\n");
			
#endif
			data.clear();
			memset(receri_data,0,50);

                 }
	      //Pall_Serial.flush();
	      usleep(10*1000); 
        }
}

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

void send_pall_cmd()
{
    pall_movie_limit();

    palletizing_data.send_data[1] = palletizing_data.cmd_type;
    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+2] = palletizing_data.x_data.x_byte[i];  //X
    }

    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+6] = palletizing_data.y_data.y_byte[i];  //Y
    }

    for(int i = 0; i<4;i++)
    {
    	palletizing_data.send_data[i+10] = palletizing_data.z_data.z_byte[i];  //Z
    }  

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

void Grasp_Status_Back(const std_msgs::Int32::ConstPtr &msg)
{
	
	if(msg->data == 1)  		//吸
        	palletizing_data.cmd_type = 0x05;
	else if(msg->data == 0)		//放
		palletizing_data.cmd_type = 0x06;

         send_pall_cmd();	
}

void Pall_Running_Back(const std_msgs::Int32::ConstPtr &msg)
{
	
	if(msg->data == 1)  		//开始
        	palletizing_data.cmd_type = 0x02;
	else if(msg->data == 0)		//暂停
		palletizing_data.cmd_type = 0x03;

         send_pall_cmd();	
}

void Pall_Reset_Back(const std_msgs::Int32::ConstPtr &msg)
{
	if(msg->data == 1)  		//回原点
        palletizing_data.cmd_type = 0x04;

         send_pall_cmd();
}

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
    tcgetattr( 0, &initial_settings );

    new_settings = initial_settings;

    new_settings.c_lflag &= ~ICANON;

    new_settings.c_lflag &= ~ECHO;

    new_settings.c_lflag &= ~ISIG;

    new_settings.c_cc[VMIN] = 1;

    new_settings.c_cc[VTIME] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );
}

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
	mosquitto_publish(mosq,NULL,"/HG_DEV/AD_MSG",strlen(message),message,0,0); 

	cJSON_Delete(root);	
	if(message != NULL)
	{	
	    //cJSON_free(message);
	}

}


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

        if(MQTT_PALL.cmd_type ==  0)
        {
                Item = cJSON_GetObjectItem(root,"pos");
                if(Item != NULL)
                {               cJSON *mqtt_pos = NULL;
                                int  array_size  = cJSON_GetArraySize (Item);
                                for(int i = 0;  i<array_size;i++)
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
                }else if(MQTT_PALL.EmStop_Status == 0)
                {

                         MQTT_PALL.EmStop_Status = Item->valueint;
                }
        }
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
	msg=cJSON_Print(root);
	if(msg != NULL)
	{	
		printf("\r\nmessage:%s\r\n",msg);    
		cJSON_free(msg);
	}
	//返回数据

//skip:

}

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
void AUTO_MSG_DATA(cJSON *mqtt_root)
{
    cJSON * root = mqtt_root;
    cJSON *Item = NULL;
    char *msg = NULL;


	cJSON_Delete(root);
	cJSON_free(msg);
}

//获取MQTT消息
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
		if(!strcmp(dir,"AD"))
		{
			if(!strcmp(name,"ZK"))
			{
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
 
void subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;
    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}
 
void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    /* Pring all log messages regardless of level. */
    printf("%s\n", str);
}
 
void* mqtt_data_thread(void *parameter)
{
	pthread_detach(pthread_self()); 
        while(mqtt_sub_runing)
        {
	        //循环处理网络消息
	    	mosquitto_loop_forever(mosq, -1, 1);
	       //开启一个线程，在线程里不停的调用 mosquitto_loop() 来处理网络信息
	       int loop = mosquitto_loop_start(mosq);
	       if(loop != MOSQ_ERR_SUCCESS)
	       {
		  printf("mosquitto loop error\n");
	       }
		ros::spinOnce();
	    usleep(1000*5);
        }
}

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

int main(int argc,char **argv)
{

        ros::init(argc, argv, "Palletizing_NODE"); 
        static int gruab = 1;
        ros::NodeHandle n;

        ros::NodeHandle nh("~");        
        std:: string usart_port;

        grab_publisher = n.advertise<std_msgs::Int32>("/Pall_GRAB_STATUS", 1);  //grab_publisher
        pos_publisher = n.advertise<std_msgs::Float32MultiArray>("/Pall_CURR_POS", 1);  //
        ros::Subscriber RUNNING_SUB = n.subscribe("/Pall_Running_Topic", 5,&Pall_Running_Back);
        ros::Subscriber Grasp_SUB = n.subscribe("/Pall_Grasp_Topic", 5,&Grasp_Status_Back);
        ros::Subscriber RESET_SUB = n.subscribe("/Pall_Reset_Topic", 5,&Pall_Reset_Back);
        ros::Subscriber PALL_POS_SUB = n.subscribe("/Pall_POS_SET", 5,&Pall_POS_Back);


        nh.param<std::string>("usart_port", usart_port,  "/dev/ttyUSB0");
        nh.param<std::string>("mqtt_host",  mqtt_host,  "192.168.10.124");
        nh.param<int>("mqtt_port",  mqtt_port,  50002);
        nh.param<float>("clip_pulse",  clip_pulse,  2000);

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

        pthread_t 	recei_thread,input_key,mqtt_thread;

        pthread_create(&recei_thread, NULL, read_data_thread, NULL);
	pthread_create(&input_key, NULL, capture_keyvalue, NULL);

    	mosquitto_lib_init();  							//libmosquitto 库初始化   	
    	mosq = mosquitto_new(NULL,session,NULL);  				//创建mosquitto客户端
    	if(!mosq)
	{
        	printf("创建 MQTT 连接失败..\n");
        	mosquitto_lib_cleanup();
        	return 1;
    	}
    	
    	//mosquitto_log_callback_set(mosq, my_log_callback);  			//设置回调函数，需要时可使用
 
    	mosquitto_connect_callback_set(mosq, car_connect);
    	mosquitto_message_callback_set(mosq, car_message);

    	//mosquitto_subscribe_callback_set(mosq, subscribe_callback);
    	if(mosquitto_connect(mosq, mqtt_host.c_str(), mqtt_port, KEEP_ALIVE))  			//客户端连接服务器
        {   
        	fprintf(stderr, "\nUnable to connect.\n");
        	printf("MQTT 连接失败...\n");
        	return 1;
    	}
	
	mqtt_sub_runing = 1;
        pthread_create(&mqtt_thread, NULL, mqtt_data_thread, NULL); 

	palletizing_data.send_data[0] = Header_Frame;
	palletizing_data.send_data[19] = Tail_Frame;
 
	palletizing_data.x_data.x = clip_pulse;
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
 	      switch(key)
              {
  
		   case 'a':  palletizing_data.x_data.x -= 1.30;
        		      palletizing_data.cmd_type = 0x00;
                              send_pall_cmd();
			      break;

		  case 'd':    palletizing_data.x_data.x += 1.30;
			       palletizing_data.cmd_type = 0x00;   
                               send_pall_cmd();	
			       break;

		  case 'w':    palletizing_data.y_data.y += 1.30;
 			       palletizing_data.cmd_type = 0x00;  
                               send_pall_cmd();  	
			       break;

		  case 's':   palletizing_data.y_data.y -= 1.30;  
			      palletizing_data.cmd_type = 0x00;	
                              send_pall_cmd();  
			      break;

		  case 'q':    palletizing_data.z_data.z += 1.5;
 			       palletizing_data.cmd_type = 0x00;  
                               send_pall_cmd();  	
			       break;

		  case 'e':   palletizing_data.z_data.z -= 1.5; 
			      palletizing_data.cmd_type = 0x00;	
                              send_pall_cmd();  
			      break;



		  case 'f':   if(gruab)
			      	palletizing_data.cmd_type = 0x05;
			      else 
		      	        palletizing_data.cmd_type = 0x06;
			      gruab = !gruab;		
                              send_pall_cmd();  
			      break;

		  case '1':   palletizing_data.cmd_type = 0x07;	
                              send_pall_cmd();
			      palletizing_data.x_data.x = 0.0;  
			      break;

		  case '2':   palletizing_data.cmd_type = 0x08;	
                              send_pall_cmd(); 
                              palletizing_data.y_data.y = 0.0;   
			      break;

		  case  3: goto exit;break;
		 
		  default:
/*
			 palletizing_data.x_data.x = recei_pall_data.X_Curr;        
		         palletizing_data.y_data.y = recei_pall_data.Y_Curr;
		         palletizing_data.z_data.z = recei_pall_data.Z_Curr;
		         palletizing_data.u_data.u = recei_pall_data.U_Curr; 
*/
			break;

	     }
/*

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
exit:
        Pall_recei_runing = 0;
        key_runing = 0;
 	mosquitto_destroy(mosq);
    	mosquitto_lib_cleanup();
    	close_keyboard();
 	Pall_Serial.close(); 

        return 0;
}



