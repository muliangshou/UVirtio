
/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
* @file:    ota.c
* @brief:   a application ota task of system
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2021/11/3
*
*/
#include <transform.h>
#include <adapter.h>

extern int main(void);

struct ota_header_t
{
    int16 frame_flag;          ///< frame start flag 2 Bytes
    uint8 dev_type;            ///< device type
    uint8 burn_mode;           ///< data burn way
    uint32 total_len;          ///< send data total length caculated from each frame_len 
    uint32 dev_hid;            ///< device hardware version
    uint32 dev_sid;            ///< device software version
    char resv[8];             ///< reserve
};

struct ota_frame_t
{
    uint32 frame_id;           ///< Current frame id
    uint32 frame_len;          ///< Current frame data length
    char   frame_data[64];    ///< Current frame data,max length 64
    uint32 crc;                ///< Current frame data crc
};

struct ota_data
{
    struct ota_header_t header;
    struct ota_frame_t frame;
    char end[4];
};

pthread_t ota_task;
pthread_t restart_main;

/**
 * @description: CRC16 check
 * @param data data buffer
 * @param length data length
 * @return check code
 */
uint32_t OtaCrc16(uint8_t * data, uint32_t length)
{
    int j;
    unsigned int reg_crc=0xFFFF;
    printf("crc data length[%d] Bytes,",length);

    while (length--) {
        reg_crc ^= *data++;
        for (j=0;j<8;j++) {
            if(reg_crc & 0x01)
                reg_crc=reg_crc >>1 ^ 0xA001;
            else
                reg_crc=reg_crc >>1;
        }
    }
    printf(" crc = [0x%x]\n",reg_crc);
    return reg_crc;
}

uint32_t FileCrc16(uint8_t * data, uint32_t length, unsigned int last_crc)
{
    int j;
    //printf("crc data length[%d] Bytes,",length);
 
    while (length--) {
        last_crc ^= *data++;
        for (j=0;j<8;j++) {
            if(last_crc & 0x01)
                last_crc = last_crc >>1 ^ 0xA001;
            else
                last_crc = last_crc >>1;
        }
    }

    //printf(" crc = [0x%x]\n",last_crc);

    return last_crc;
}


static int SaveAppBin(int fd, char* buf, int len)
{
    int ret = 0;
    int fd_t = 0;
    fd_t = open( BOARD_APP_NAME, O_RDWR | O_APPEND);
    ret = write(fd, buf, len);
    if(ret < 0){
        printf("fd = %d write buf len[%d] failed.ret = %d\n",fd_t,len,ret);
    }
    else
    {
        printf("fd[%d] write buf length[%d] done.\n",fd_t,ret);
    }
    
    close(fd_t);
}

static int CrcFileCheck(uint32 crc_check, unsigned long total_len)
{
    int ret = 0;
    int fd = 0;
    int len = 0;
    char *buf = NULL;
    unsigned int last_crc = 0xffff;
    unsigned long already_crc_length = 0;
    
    fd = open( BOARD_APP_NAME, O_RDONLY );
    if(fd < 0){
        printf("open %s bin failed.\n",BOARD_APP_NAME);
        return -1;
    }

    buf = PrivMalloc(128);
    if(NULL == buf)
    {
        printf("malloc failed.\n");
        close(fd);
        return 0;
    }

    /* crc check every 1024 Bytes until crc all the total file */
    while(already_crc_length != total_len)
    {
        memset(buf , 0 , 128);
        len = read(fd, buf, 128);
        if(len < 0)
        {
            printf("file read failed.ret = %d\n",len);
            ret = -1;
            break;
        }
        
        last_crc = FileCrc16(buf, len, last_crc);
        already_crc_length += len;
        printf("read len[%d] Bytes,already_crc_length[%d]\n",len,already_crc_length);
    }
    

    if (last_crc != crc_check)
    {
       printf("file crc error!!! last crc[%x] != check[%x]\n",last_crc,crc_check);
       ret =-1;
    }

    PrivFree(buf);
    close(fd);

    return ret;
}

static void RestartApplication(void)
{
    pthread_attr_t attr;
    attr.schedparam.sched_priority = 10;
    attr.stacksize = 2048;

    while(1) 
    {
        unsigned long pid = PrivUserTaskSearch();
        if ((pid > 0) && (pid != pthread_self())) 
        {
            printf("kill usertask pid[%d]\n",pid);
            PrivTaskDelete(pid, 0);
            PrivTaskDelay(1000); /* NOTE:this delay will make a schedule and recycle all user task */
        } 
        else 
        {
            break;
        }
    }
    printf("restart main.\n");
    PrivTaskCreate(&restart_main, &attr, (void *)main, NULL);
}
static int OtaDataRecv(struct Adapter* adapter)
{
    struct ota_data recv_msg;
    char reply[16] = {0};
    int ret = 0;
    int try_times = 10;
    int fd = 0;
    int frame_cnt = 0;

    fd = open( BOARD_APP_NAME, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    if(fd < 0)
    {
        printf("open %s failed\n",BOARD_APP_NAME);
        return -1;
    }
    close(fd);

    while(1) {
        memset(&recv_msg, 0, sizeof(struct ota_data));
        printf("recv msg...\n");
        ret = AdapterDeviceRecv(adapter, &recv_msg, sizeof(struct ota_data));
        if(ret >= 0 && recv_msg.header.frame_flag == 0x5A5A) 
        {
            if(0 == strncmp("aiit_ota_end",recv_msg.frame.frame_data, strlen("aiit_ota_end"))) 
            {
                printf("total [%d]frames [%d]Bytes crc[%x],receive successful,\n",frame_cnt,recv_msg.header.total_len,recv_msg.frame.crc);
                if(0 != CrcFileCheck(recv_msg.frame.crc, recv_msg.header.total_len))
                {
                    printf("crc check %s bin failed.please try again.\n", BOARD_APP_NAME);
                    ret = -1;
                    break;
                }
                PrivTaskDelay(500);
                printf("tolal file crc done.send ok\n");
                memset(reply, 0, 16);
                memcpy(reply, "ok", strlen("ok"));

                AdapterDeviceSend(adapter, reply, strlen(reply));
                ret = 0;
                break;
            }
            frame_cnt = recv_msg.frame.frame_id;

            if(0 == strncmp("wait_ok_timeout",recv_msg.frame.frame_data, strlen("wait_ok_timeout"))) 
            {
                printf("go to send ok again.\n");
                goto send_ok_again;
                
            }

            if (recv_msg.frame.crc == OtaCrc16(recv_msg.frame.frame_data,recv_msg.frame.frame_len))
            {
                printf("save current [%d] frame,length[%d] Bytes.\n",frame_cnt,recv_msg.frame.frame_len);
                for(int i = 0; i < recv_msg.frame.frame_len;i++ ){
                    printf(" %x ",*((char *)&recv_msg.frame.frame_data + i));
                }
                printf("\n");
                SaveAppBin(fd, recv_msg.frame.frame_data, recv_msg.frame.frame_len);
            }  
            else 
            {
                printf("current [%d] frame crc check failed,try again!\n",frame_cnt);
                goto try_again;
            }
            
send_ok_again:
            memset(reply, 0, 16);
            memcpy(reply, "ok", strlen("ok"));
            // PrivTaskDelay(100);

            ret = AdapterDeviceSend(adapter, reply, strlen(reply));
            if(ret < 0){
                printf("send ok failed.\n");
                goto send_ok_again;
            }
            printf("send reply[%s] done.\n",reply);
            try_times = 10;
            continue;
        } 
        else 
        {
try_again:
            if(try_times == 0)
            {
                printf("oops!!! current [%d] frame try 10 times failed,break out!\n",frame_cnt);
                ret = -1;
                break;
            }
            memset(reply, 0, 16);
            memcpy(reply, "retry", strlen("retry"));
            printf("[%d] frame receive failed. retry\n",frame_cnt);
            AdapterDeviceSend(adapter, reply, strlen(reply));
            try_times--;
            continue;
        }
    }
    close(fd);

    if(0 == ret) {
        printf("ota file done,start application.\n");
        RestartApplication();
    }
    return ret;
}

static void *OtaKTaskEntry(void *parameter)
{
    struct ota_data recv_msg;
    char reply[16] = {0};
    int baud_rate = BAUD_RATE_115200;
    int len = 0;
    int ret = 0;

    struct Adapter* adapter =  AdapterDeviceFindByName("4G");
    uint8 server_addr[64] = "115.238.53.61";
    uint8 server_port[64] = "9898";

    adapter->socket.socket_id = 0;

    AdapterDeviceOpen(adapter);
    AdapterDeviceControl(adapter, OPE_INT, &baud_rate);
    AdapterDeviceConnect(adapter, CLIENT, server_addr, server_port, IPV4);

    /* using nbiot as connection way*/

//     struct Adapter* adapter =  AdapterDeviceFindByName("nbiot");

//     while(1)
//     {
//         int connect_times = 5;
//         ret = AdapterDeviceOpen(adapter);
//         if(ret < 0)
//         {
//            printf("open adapter failed\n");
//            continue;
//         }

// connect_again:
//         connect_times--;   
//         ret = AdapterDeviceConnect(adapter, 1, "115.238.53.61","9898",1);
//         if(ret < 0)
//         {
//             if(connect_times > 0){
//                 goto connect_again;
//             }
//             else
//             {
//                 AdapterDeviceClose(adapter);
//                 continue;
//             }
//         }
//         break;
//     }
    PrivTaskDelay(5000);
    while(1)
    {
        memset(&recv_msg, 0, sizeof(struct ota_data));
        /* step1: Confirm the start signal of transmission*/
        printf("waiting for start msg...\n");
        ret = AdapterDeviceRecv(adapter, &recv_msg, sizeof(struct ota_data));
        for(int i = 0; i < sizeof(struct ota_data);i++ ){
            printf(" %x ",*((char *)&recv_msg + i));
        }
        printf("\n");
        if(ret >= 0 && recv_msg.header.frame_flag == 0x5A5A) 
        {
            if (0 == strncmp("aiit_ota_start",recv_msg.frame.frame_data, strlen("aiit_ota_start"))) 
            {
                memset(reply, 0, 16);
                memcpy(reply, "ready", strlen("ready"));
                // PrivTaskDelay(3000);
                printf("receive start signal,send [ready] signal to server\n");
send_ready_again:
                ret = AdapterDeviceSend(adapter, reply, strlen(reply));
                if(ret < 0)
                {
                    goto send_ready_again;
                }
                PrivTaskDelay(3000);
                printf("start receive ota file.\n");
                /* step2: start receive source bin file of application*/
                ret = OtaDataRecv(adapter);
                if (0 != ret)
                {
                    memset(reply, 0, 16);
                    memcpy(reply, "ota_restart", strlen("ota_restart"));
                    AdapterDeviceSend(adapter, reply, strlen(reply));
                    continue;
                } 
                else
                {
                    break;
                }
            }
        }
        else
        {
            memset(reply, 0, 16);
            memcpy(reply, "notready", strlen("notready"));
            printf("ota status:not ready\n");
            ret = AdapterDeviceSend(adapter, reply, strlen(reply));
        }
        PrivTaskDelay(3000); /* check ota signal every 5s */
    }
    AdapterDeviceClose(adapter);
    
}

void ApplicationOtaTaskInit(void)
{   
    pthread_attr_t attr;
    attr.schedparam.sched_priority = 20;
    attr.stacksize = 4096;

    PrivTaskCreate(&ota_task, &attr, OtaKTaskEntry, NULL);

}