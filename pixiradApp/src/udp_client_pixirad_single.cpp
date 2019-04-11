/*
 ============================================================================
 Name        : udp_client.c
 Author      : Massi
 Version     : Converted to EPICS OSI by Mark Rivers, Jan. 18, 2014
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <string.h>
#include "PIXIEII_data_utilities.h"
#include <math.h>

// EPICS includes
#include <osiSock.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>

#define DATA_BUFFLEN_BYTES 2048
#define HEADER_BUFFLEN_BYTES 32
#define BUFLEN (DATA_BUFFLEN_BYTES+HEADER_BUFFLEN_BYTES)
#define MAX_PACK_LEN 1448
#define PACKET_TAG_BYTES 2
#define PACKET_TAG_OFFSET 0
#define PACKET_ID_BYTES 2
#define PACKET_ID_OFFSET 2
#define PACKET_CRC_BYTES 4
#define PACKET_SENSOR_DATA_OFFSET (PACKET_TAG_BYTES+PACKET_ID_BYTES)
#define PACKET_EXTRA_BYTES (PACKET_ID_BYTES+PACKET_TAG_BYTES+PACKET_CRC_BYTES)
#define PACKET_SENSOR_DATA_BYTES (MAX_PACK_LEN-PACKET_EXTRA_BYTES)
#define PIXIEII_MODULES 1

#define FRAME_HAS_ALIGN_ERRORS 0x20
#define REG_PACKET 0x80
#define SLOT_ID_MASK 0xff
#define SLOT_ID_OFFSET 1
#define AUTOCAL_DATA 0x40
#define AUTOCAL_REG_DEPTH 5
#define COUNTER_REG_DEPTH 15

//bit masks used to store parameters in dummy0_1
//dummy0
#define DUMMY_0_OFFSET 0
#define PIXIE_THDAC_MASK 0x1f
#define PIXIE_THDAC_OFFSET 0
//dummy1
#define DUMMY_1_OFFSET 8
#define LOOP_MODE_OFFSET 0
#define LOOP_MODE_MASK 0xff
#define LOOP_COLOR_MODE_OFFSET 0
#define LOOP_COLOR_MODE_MASK 0xf
#define LOOP_DTF_MODE_OFFSET 4
#define LOOP_DTF_MODE_MASK 0xf


#define MAX_STRLEN 200

#define MAX_PENDING_BUFFERS 1500
#define DEFAULT_NPACK 360
#define AUTOCAL_NPACK 135
#define DAQ_PACK_FRAGM 45
#define PORTA 2223
#define MOD_UDP_REMOTE_PORT 3333
#define PORTB 2224
#define MAXBUF 256217728
#define BYTES_PER_ROWS 16
unsigned char *buf,*process_buf,looping=1;


unsigned char *uchar_ptr;
unsigned int crc_generator=0x2608edb,calculated_crc32_byme,calculated_crc32,packet_index=0,testword;
int convert_data=1;
unsigned int received_packets=0,error_packets=0,id_error_packets=0;

void print_packet(const unsigned char* buffer,unsigned int packet_len_bytes){
    unsigned int i,j;
    for(j=0;j<packet_len_bytes/BYTES_PER_ROWS;j++){
        printf("\n(%04X)",j*BYTES_PER_ROWS);
        for (i=0;(i<BYTES_PER_ROWS)&&(i+j*BYTES_PER_ROWS<packet_len_bytes);i++)
            printf("-%02x-",buffer[i+j*BYTES_PER_ROWS]);
    }
    printf("\n(%04X)",j*BYTES_PER_ROWS);
    for(i=0;i<packet_len_bytes%BYTES_PER_ROWS;i++)
        printf("-%02x-",buffer[i+j*BYTES_PER_ROWS]);
}

void key_proc(void* received_packet)
{
    char char_ptr;

    printf("\r\nkey process started\r\n");
    while(1){
        uchar_ptr=buf+packet_index*MAX_PACK_LEN;
        scanf("%c%*c",&char_ptr);
        if(char_ptr=='p')
             print_packet(uchar_ptr,MAX_PACK_LEN);

        if(char_ptr=='i'){
            packet_index++;
            printf("packet index=%d\n",packet_index);}
        if(char_ptr=='d'){
            packet_index--;
            printf("packet index=%d\n",packet_index);}
        if(char_ptr=='r'){
            packet_index=0;
            received_packets=0;
            error_packets=0;
            printf("packet index=%d\n",packet_index);}
        if(char_ptr=='s'){
            printf("packets stat: received=%d, errors=%d id_errors=%d\n",received_packets,error_packets,id_error_packets);}
        if(char_ptr=='c'){
            if(convert_data==0)
                convert_data=1;
            else
                convert_data=0;
            printf("\nconvert_data set to %1d\n",convert_data);}
        if(char_ptr=='q'){
            printf("closing loop\n");
            looping=0;
        }
        else if(char_ptr=='k'){
        }
    }
}

int verbose=0;

epicsMessageQueueId ptr_list;

//#define REG_PACKET 0x80
//#define AUTOCAL_DATA 0x40
#define DOUT_1 0x0001
#define DOUT_2 0x0002
#define DOUT_3 0x0004
#define DOUT_4 0x0008
#define DOUT_5 0x0010
#define DOUT_6 0x0020
#define DOUT_7 0x0040
#define DOUT_8 0x0080
#define DOUT_9 0x0100
#define DOUT_10 0x0200
#define DOUT_11 0x0400
#define DOUT_12 000800
#define DOUT_13 0x1000
#define DOUT_14 0x2000
#define DOUT_15 0x4000
#define DOUT_16 0x8000


int convert_bit_stream_to_counts(int code_depth,unsigned short* source_memory_offset,
                                unsigned short* destination_memory_offset,int reusulting_readings){
    int i,j;
    //unsigned short dout_masks[reusulting_readings],mask_seed=1;
    unsigned short dout_masks[PIXIE_DOUTS],mask_seed=1;
    for(i=0;i<reusulting_readings;i++) dout_masks[i]=(mask_seed<<i);
    for(j=0;j<reusulting_readings;j++){
        destination_memory_offset[j]=0;
        for(i=code_depth-1;i>=0;i--){
            if(source_memory_offset[i] & dout_masks[j])
                destination_memory_offset[j]|= dout_masks[code_depth-i-1];
            else
                destination_memory_offset[j]&= ~dout_masks[code_depth-i-1];
        }
    }

    //for(i=0;i<code_depth;i++)printf("\nsource_mem[%d]-->%4X",i,source_memory_offset[i]);
    //Sleep(1000);
    //for(i=0;i<reusulting_readings;i++) printf("\ndest[%d]-->%4X",i,destination_memory_offset[i]);

    return(j);
}

#define HEADER_LENGHT                 10
#define vt_dac 22
#define sh_code 33
#define shutter_duration_ms 44


void my_bytes_swap(unsigned short* us_ptr){
    char a,b,*temp_char_ptr;
    temp_char_ptr=(char*)us_ptr;
    a=*temp_char_ptr;
    b=*(temp_char_ptr+1);
    *(temp_char_ptr+1)=a;
    *(temp_char_ptr)=b;
}

void* module_data_parser_thread(void* arg){
    int is_autocal_data, reg_data,i,j,k,code_depth,err;
    unsigned short *local_buffer_ptr,*temp_us_ptr,*conv,*process_buf_ptr,*netbuffer,frame_header[HEADER_LENGHT];
    unsigned short packet_tag,slot_id=0;
    unsigned char th_dac=0,loop_mode=0;
    unsigned short this_frame_has_aligment_errors;
    SOCKET data_socket;
    struct sockaddr_in sock_addr;
    int status;

    local_buffer_ptr=databuffer_allocation(PIXIEII_MODULES*MATRIX_DIM_WORDS*15);//la dimensione in byte di un intero frame
    if(local_buffer_ptr==NULL){
        printf("PROC:error allocating buffers\n");
        exit(-1);}
    else{
        if(verbose>=2)printf("PROC:Processing Thread Started\n");
    }
    /***TCP/IP monitor***************inizio********************dacounting_daq.cpp********************/
    sock_addr.sin_family=AF_INET; // indico il protocollo utilizzato (TCP/IP)
    sock_addr.sin_port=htons(4444); //indico la porta a cui connettere il socket
    if (hostToIPAddr("127.0.0.1", &sock_addr.sin_addr) < 0)
        printf("PROC:hostToIPAddr failed\n");
    // The following is not supported on Linux.  Is it needed?
    //sock_addr.sin_addr.S_un.S_un_b.s_b1=127; // indico l'indirizzo IP
    //sock_addr.sin_addr.S_un.S_un_b.s_b2=0;
    //sock_addr.sin_addr.S_un.S_un_b.s_b3=0;
    //sock_addr.sin_addr.S_un.S_un_b.s_b4=1;
    netbuffer=databuffer_allocation((MATRIX_DIM_WORDS*PIXIEII_MODULES)+HEADER_LENGHT);

    /*****TCP/IP monitor****************fine********************dacounting_daq.cpp********************/
    printf("PROC:data Parser Thread started\n");
    conv=conversion_table_allocation();

    while(1){
        status = epicsMessageQueueReceive(ptr_list, &process_buf_ptr, sizeof(&process_buf_ptr));
printf("module_data_parser_thread: got process_buf_ptr=%p\n", process_buf_ptr);
        if (status <= 0){
            printf("PROC: error epicsMessageQueueReceive returned %d\n", status);
            continue;
        }
        if (process_buf_ptr==NULL){
            printf("PROC: error process_buf_ptr=NULL\n");
            continue;
        }
        packet_tag=*(process_buf_ptr+PACKET_TAG_OFFSET*2);
        th_dac=(packet_tag>>(PIXIE_THDAC_OFFSET+DUMMY_0_OFFSET))&PIXIE_THDAC_MASK;
        loop_mode=(packet_tag>>(LOOP_MODE_OFFSET+DUMMY_1_OFFSET))&LOOP_MODE_MASK;
        if(verbose>=2)printf("\nPROC: packet_tag=%04xh\n",packet_tag);
        if(verbose>=2)printf("\nPROC: th_dac=%d, loop_mode=%02Xh\n",th_dac,loop_mode);
        slot_id=*((char*)process_buf_ptr+SLOT_ID_OFFSET)&SLOT_ID_MASK;
        if(verbose>=2)printf("PROC: slot_id=%04xh\n",slot_id);
        if(packet_tag & AUTOCAL_DATA){
            if(verbose>=2)printf("\nPROC:Autocal");
            code_depth=5;
            is_autocal_data=1;
        } else{
            if(verbose>=2)printf("\nPROC:Counters");
            code_depth=15;
            is_autocal_data=0;
        }
        if(packet_tag & REG_PACKET){
            if(verbose>=2)printf("\nPROC:REG 1 data\n");
            reg_data=1;
        } else{
            if(verbose>=2)printf("\nPROC:REG 0 data\n");
            reg_data=0;
        }
        if(packet_tag & FRAME_HAS_ALIGN_ERRORS){
            this_frame_has_aligment_errors=1;
        } else{
            this_frame_has_aligment_errors=0;
        }

        temp_us_ptr=process_buf_ptr+(PACKET_TAG_BYTES/2);
        if(verbose>=3)printf("local_buffer filling code_depth=%d\n",code_depth);
        for(i=0;i<PIXIEII_MODULES;i++) {
            for(j=0;j<COLS_PER_DOUT*PIXIE_ROWS;j++) {
                for(k=0;k<code_depth;k++){
                    my_bytes_swap(temp_us_ptr+i+(j*PIXIEII_MODULES*code_depth)+(k*PIXIEII_MODULES));
                    local_buffer_ptr[(i*COLS_PER_DOUT*PIXIE_ROWS*code_depth)+(j*code_depth)+k]=
                        temp_us_ptr[i+(j*PIXIEII_MODULES*code_depth)+(k*PIXIEII_MODULES)];
                }
            }
        }
        //printf("local_buffer has been filled with original data and data modules are grouped\n");
        //memcpy(buff+PACKET_TAG_BYTES,process_buf_ptr+PACKET_TAG_BYTES,PIXIEII_MODULES*MATRIX_DIM_WORDS*code_depth);
        for(i=0;i<PIXIEII_MODULES;i++){
            //printf("parsing %d module data\n",i);
            for(j=0;j<COLS_PER_DOUT*PIXIE_ROWS;j++) {
                convert_bit_stream_to_counts(code_depth,
                    local_buffer_ptr+(i*COLS_PER_DOUT*PIXIE_ROWS*code_depth)+(j*code_depth),
                    process_buf_ptr+(i*MATRIX_DIM_WORDS)+(j*PIXIE_DOUTS)+(PACKET_TAG_BYTES/2),PIXIE_DOUTS);

            }
        }
        //printf("data parsed\n");
        for(i=0;i<PIXIEII_MODULES;i++){
            //printf("processing module %d data\n,i);
            if(is_autocal_data==0 && convert_data==1)
                decode_pixie_data_buffer(conv,process_buf_ptr+(PACKET_TAG_BYTES/2)+i*MATRIX_DIM_WORDS);
            databuffer_sorting(process_buf_ptr+(PACKET_TAG_BYTES/2)+i*MATRIX_DIM_WORDS);
            map_data_buffer_on_pixie(process_buf_ptr+(PACKET_TAG_BYTES/2)+i*MATRIX_DIM_WORDS);
        }

        /*********************sendig data to TCPIP monitor*********************/
        for(i=0;i<HEADER_LENGHT;i++) frame_header[i]=0x8000;
        frame_header[0]=BUFFER_HDR_TAG;
        frame_header[1]|=(unsigned short)(this_frame_has_aligment_errors);
        frame_header[2]|=(unsigned short)(is_autocal_data);
        frame_header[3]|=(sh_code & 0x1);
        frame_header[4]|=(unsigned short)ceil((double)shutter_duration_ms);
        frame_header[5]|=(unsigned short)slot_id;
        if(reg_data==0)
            frame_header[6]|=REG_0_BUFF_CODE;
        else
            frame_header[6]|=REG_1_BUFF_CODE;
        for(i=0;i<HEADER_LENGHT;i++) netbuffer[i]=frame_header[i];
        for(i=0;i<MATRIX_DIM_WORDS*PIXIEII_MODULES;i++) netbuffer[i+HEADER_LENGHT]=process_buf_ptr[i+(PACKET_TAG_BYTES/2)];
        data_socket=epicsSocketCreate(PF_INET,SOCK_STREAM,0);
        err=connect(data_socket,(struct sockaddr*)&sock_addr,sizeof(struct sockaddr));
        if(!err) {
            if (verbose>=1)printf("\r\nI got Pixie!!");
        } else
            if (verbose>=1)printf("\r\nGot error attempting to connect to pixie = %s",strerror(errno));
        if(!err) send(data_socket,(const char*)netbuffer,(MATRIX_DIM*PIXIEII_MODULES)+(HEADER_LENGHT*2),0);
        if(!err)epicsSocketDestroy(data_socket);
        /*********************sendig data to TCPIP monitor*********************/
        free(process_buf_ptr);
        if (verbose>=1)printf("PROC:pocessing buffer released\n");

    }
    free(local_buffer_ptr);//qui non ci arriver mai!!!!
}

typedef enum{FRAG_ID,NOFRAG_ID} id_mode;
typedef enum{UDPMOD,NOUDPMOD} moderation_type;

int main(int argc, char**argv)
{
    struct sockaddr_in si_me;
    struct sockaddr_in moderator_udp_sockaddr;
    unsigned short packet_tag;
    unsigned int i,j,temp_NPACK;
    SOCKET s, moderator_udp_sock_fd;
    int this_frame_has_aligment_errors=0;
    char temp_char;
    unsigned int packet_id, local_packet_id;
    long packet_id_gap;
    int received_data_subsets=0;
    moderation_type moderation=NOUDPMOD;
    clock_t timer_a=0,timer_b;
    double  time_interval;
    char moderator_string[MAX_STRLEN];
    id_mode packet_id_mode;

    ptr_list = epicsMessageQueueCreate(MAX_PENDING_BUFFERS, sizeof(unsigned char *));
    if (ptr_list == 0) {
        printf("epicsMessageQueueCreate failed\n");
        return -1;
    }

    if (osiSockAttach() == 0) {
        printf("osiSockAttach failed\n");
        return -1;
    }

    if(argc==4){
        if (strcmp(argv[3],"UDPMOD")==0)
            moderation=UDPMOD;
        else
            moderation=NOUDPMOD;
        verbose=atoi(argv[2]);
        if (strcmp(argv[1],"FRAG_ID")==0)
            packet_id_mode=FRAG_ID;
        else
            packet_id_mode=NOFRAG_ID;
    }
    if(argc==3){
        verbose=atoi(argv[2]);
        if (strcmp(argv[1],"FRAG_ID")==0)
            packet_id_mode=FRAG_ID;
        else
            packet_id_mode=NOFRAG_ID;
    }
    else if(argc==2){
        verbose=0;
        if (strcmp(argv[1],"FRAG_ID")==0)
            packet_id_mode=FRAG_ID;
        else
            packet_id_mode=NOFRAG_ID;
    }
    else{
        verbose=0;
        packet_id_mode=FRAG_ID;
        moderation=NOUDPMOD;
    }

    buf=(unsigned char*)databuffer_allocation(MAX_PACK_LEN*DEFAULT_NPACK);


    if ((s=epicsSocketCreate(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    {
        printf("\r\nError creating socket error =%s", strerror(errno));
        scanf("%c%*c",&temp_char);
        return 1;
    }

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORTA);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *) &si_me, sizeof(si_me))==-1)
    {
        printf("\r\nError in binding data receiver socket");
        //wait_akey();
        return 1;
    }


    if ((moderator_udp_sock_fd=epicsSocketCreate(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    {
        printf("\r\nError creating moderator socket error =%s", strerror(errno));
        scanf("%c%*c",&temp_char);
        return 1;
    }

    memset((char *) &moderator_udp_sockaddr, 0, sizeof(moderator_udp_sockaddr));
    moderator_udp_sockaddr.sin_family = AF_INET;
    moderator_udp_sockaddr.sin_port = htons(MOD_UDP_REMOTE_PORT);
    // The following is not support on Linux.  Is it needed?
    if (hostToIPAddr("192.168.0.255", &moderator_udp_sockaddr.sin_addr) < 0)
        printf("PROC:hostToIPAddr failed\n");
    //moderator_udp_sockaddr.sin_addr.S_un.S_un_b.s_b1=192; //l'indirizzo IP!!!
    //moderator_udp_sockaddr.sin_addr.S_un.S_un_b.s_b2=168;
    //moderator_udp_sockaddr.sin_addr.S_un.S_un_b.s_b3=0;
    //moderator_udp_sockaddr.sin_addr.S_un.S_un_b.s_b4=255;

    int buffsize;
    osiSocklen_t czm = sizeof( int);
    int received_bytes;
    buffsize = MAXBUF;
    if (setsockopt(s, SOL_SOCKET,
                   SO_RCVBUF,
                   (char*)&buffsize,
                   czm) != -1)
    {
        if(verbose>=2)printf("\r\nTrying to set Receive Buffer = %d", buffsize);
    }

    if (getsockopt(s, SOL_SOCKET,
                   SO_RCVBUF,
                   (char*)&buffsize,
                   &czm) != -1)
    {
        if(verbose>=2)printf("\r\nReceive buffer is now = %d  ", buffsize);
        if (buffsize == MAXBUF)
        {
            if(verbose>=2)printf("OK");
        }
        else
        {
            if(verbose>=2)printf("ERROR Buffer Size too big!");
        }
    }
    epicsThreadCreate("key_proc",
                       epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)key_proc, (void*)&i);

    epicsThreadCreate("module_data_parser_thread",
                       epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)module_data_parser_thread, NULL);

    while(looping){
//        sem_wait(&(ptr_list.put_sem));
        this_frame_has_aligment_errors=0;
        temp_NPACK=DEFAULT_NPACK;
        i=0;
        if(packet_id_mode==FRAG_ID){
            while(i<temp_NPACK ){
                j=0;
                while(j<DAQ_PACK_FRAGM){
                    received_bytes=recvfrom(s,(char*) buf+(i*MAX_PACK_LEN), MAX_PACK_LEN,0, NULL, 0);
                    if (received_bytes== -1)
                        printf("\rError receiveing datagram");
                    else if(received_bytes==MAX_PACK_LEN){
                        if(i==0)  timer_a=clock();

                        received_packets++;
                        packet_tag=*buf;
                        if(packet_tag & AUTOCAL_DATA)
                            temp_NPACK=AUTOCAL_NPACK;
                        else
                            temp_NPACK=DEFAULT_NPACK;

                        /********************************************************************/;
                        packet_id=buf[MAX_PACK_LEN*(i)+PACKET_ID_OFFSET]<<8;
                        packet_id+=buf[MAX_PACK_LEN*(i)+1+PACKET_ID_OFFSET];
                        packet_id=packet_id%DAQ_PACK_FRAGM;
                        packet_id_gap=(packet_id-j);

                        if(packet_id_gap!=0){
                            if(id_error_packets<10)
                                printf("ID: %d j= %d\n",packet_id,j);
                            id_error_packets++;
                            this_frame_has_aligment_errors=1;
                        }
                        if(packet_id_gap>=0){
                            j+=(packet_id_gap+1);
                            i+=(packet_id_gap+1);
                        }
                        else{
                            j=DAQ_PACK_FRAGM;
                            i+=DAQ_PACK_FRAGM;
                        }

                    }
                }
                if(moderation==UDPMOD){
                    sprintf(moderator_string,"DATASUBSET_RECEIVED %d %d\n",received_data_subsets,temp_NPACK/DAQ_PACK_FRAGM);
                    received_data_subsets++;
                    if (sendto(moderator_udp_sock_fd,
                               moderator_string,
                               strlen(moderator_string),
                               0,
                               (struct sockaddr*)&moderator_udp_sockaddr,
                               (osiSocklen_t)sizeof(moderator_udp_sockaddr))==-1)
                        printf("\r\n!!Error sending moderating datagram!!");
                }
            }
       } else{
            while(i<temp_NPACK ){
                received_bytes=recvfrom(s,(char*) buf+(i*MAX_PACK_LEN), MAX_PACK_LEN,0, NULL, 0);
                if (received_bytes== -1)
                    printf("\rError receiveing datagram");
                else if(received_bytes==MAX_PACK_LEN){
                    if(i==0)  timer_a=clock();
                    //i++;j++;
                    received_packets++;
                    packet_tag=*buf;
                    if(packet_tag & AUTOCAL_DATA)
                        temp_NPACK=AUTOCAL_NPACK;
                    else
                        temp_NPACK=DEFAULT_NPACK;

                    packet_id=buf[MAX_PACK_LEN*(i)+PACKET_ID_OFFSET]<<8;
                    packet_id+=buf[MAX_PACK_LEN*(i)+1+PACKET_ID_OFFSET];
                    packet_id_gap=(packet_id-i);//
                    if(packet_id_gap!=0){
                        id_error_packets++;
                        this_frame_has_aligment_errors=1;
                    }
                    if(packet_id_gap>=0){
                        i+=(packet_id_gap+1);
                    }
                    else{
                        i=temp_NPACK;
                    }


                    if((i%DAQ_PACK_FRAGM)==0 && i!=0 && moderation==UDPMOD){
                        sprintf(moderator_string,"DATASUBSET_RECEIVED %d %d\n",received_data_subsets,temp_NPACK/DAQ_PACK_FRAGM);
                        received_data_subsets++;
                        if (sendto(moderator_udp_sock_fd,
                                   moderator_string,
                                   strlen(moderator_string),
                                   0,
                                   (struct sockaddr*)&moderator_udp_sockaddr,
                                   sizeof(moderator_udp_sockaddr))==-1)
                            printf("\r\n!!Error sending moderating datagram!!");
                    }
                }
            }
        }


        received_data_subsets=0;

        if(verbose>=1){
            timer_b=clock();
            time_interval=(double)(timer_b-timer_a)/CLOCKS_PER_SEC;
            printf("MAIN:%d packets(%d bytes each) have been received in %.3f s ",i,MAX_PACK_LEN,time_interval);
            printf("(%.3f Mbps)\n",(i*MAX_PACK_LEN*8)/(time_interval*1024*1024));
        }

        i=0;
        local_packet_id=0;
        process_buf=(unsigned char*)databuffer_allocation((MAX_PACK_LEN-PACKET_EXTRA_BYTES+PACKET_TAG_BYTES)*DEFAULT_NPACK);
        if((buf==NULL)||(process_buf==NULL)){
            printf("error allocating buffers\n");
            exit(0);
            }
        else
            if(verbose>=1)
                printf("MAIN:New Processing Buffer Allocated\n");
        while((i<temp_NPACK))
        {
            memcpy(process_buf+PACKET_TAG_BYTES+(PACKET_SENSOR_DATA_BYTES*local_packet_id),
                   buf+PACKET_SENSOR_DATA_OFFSET+(MAX_PACK_LEN*i), PACKET_SENSOR_DATA_BYTES);
            for(j=0;j<PACKET_TAG_BYTES;j++)
                process_buf[PACKET_TAG_OFFSET+j]=buf[j];//copio il PACKET_TAG del buffer per processing
            if(this_frame_has_aligment_errors)
                process_buf[PACKET_TAG_OFFSET]|=FRAME_HAS_ALIGN_ERRORS;
            else
                process_buf[PACKET_TAG_OFFSET]&=(~FRAME_HAS_ALIGN_ERRORS);
            local_packet_id++;
            i++;
        }
printf("Main: sending process_buf=%p\n", process_buf);
        epicsMessageQueueSend(ptr_list, &process_buf, sizeof(&process_buf));


    }
    free(buf);
    free(process_buf);
    scanf("%c%*c",&temp_char);
    return EXIT_SUCCESS;
}

