/*
 * pxrd2_interface_misc.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: massimo
 */
#include "pxrd2_interface_misc.h"
#include "PIXIEII_data_utilities_v2.h"
#include "interface_command_utilities.h"
#include "pthread.h"
#include <string.h>
#include <stdio.h>
#include <iostream>
#include "windef.h"
#include "filesystem_int.h"
#include"BoxUtilities_v2.h"
#include "pxrd2_detector_utilities.h"

using namespace std;

void SetRunState(RUN* Run_ptr,RunState State){
	pthread_mutex_lock(&(*Run_ptr).RunMutex);
	(*Run_ptr).State=State;
	pthread_mutex_unlock(&(*Run_ptr).RunMutex);
	}
RunState GetRunState(RUN* Run_ptr){
	RunState Copy;
	pthread_mutex_lock(&(*Run_ptr).RunMutex);
	Copy=(*Run_ptr).State;
	pthread_mutex_unlock(&(*Run_ptr).RunMutex);
	return(Copy);
	}
void StrRunState(RunState State,char* deststr,int maxlen){
	if(State==IDLERUN) 	strncpy(deststr,"RunState:Idle",maxlen);
	if(State==RUNNING) 	strncpy(deststr,"RunState:Running",maxlen);
	if(State==CLOSINGRUN) 	strncpy(deststr,"RunState:Closing",maxlen);
}

RUNMODE getRunModeFromStr(const char* str){
	if(strcmp(str,"2COL")==0)	return TWO_COLS;
	if(strcmp(str,"1COL0")==0)	return ONE_COL0;
	if(strcmp(str,"1COL1")==0)	return ONE_COL1;
	if(strcmp(str,"DTF")==0)	return ONE_COL_DTF;
	if(strcmp(str,"2COLDTF")==0)return TWO_COLS_DTF;
	else
		return(UNDEFINED);
}




/********************************************************************************************************/
int send_int_over_the_net(SOCKET data_socket,int data){
	int ret;
	char *buffer;
	buffer=(char*)malloc(sizeof(data));
	buffer[0]=(char)(data&0xff);
	buffer[1]=(char)((data>>8)&0xff);
	buffer[2]=(char)((data>>16)&0xff);
	buffer[3]=(char)((data>>24)&0xff);
	ret=send(data_socket,(const char*)buffer,sizeof(data),0);
	free(buffer);
	return(ret);
}
/********************************************************************************************************/
int ReadDataFromSocket(Connection* Conn_ptr){
	unsigned int index;
	int bytes_recvd;
	int rx_msglen;
	char* data_buffer;
	/**********************************************************************************/
	pthread_mutex_lock(&(*Conn_ptr).socket_rx_mutex);
	/**********************************************************************************/

		/***********************************MSGLEN receive********************************/
		index=0;
		do{
			bytes_recvd=recv((*Conn_ptr).client_sock,(char*)&rx_msglen,sizeof(rx_msglen)-index,0);
			index+=bytes_recvd;
			}
		while(bytes_recvd>0 && index<sizeof(rx_msglen));

		if(bytes_recvd==SOCKET_ERROR || index<sizeof(rx_msglen) ){
			if(verbose>=VERBOSITY_HIGH)
				printf("Client Hung Up!! Going to close me too!!\r\n");
			/**********************************************************************************/
			pthread_mutex_unlock(&(*Conn_ptr).socket_rx_mutex);
			return(bytes_recvd);
			/**********************************************************************************/
			}

		if(verbose>=VERBOSITY_HIGH)
				printf("Going To Receive %d bytes\r\n",rx_msglen);
		/***********************************MSG receive********************************/

		/***********************************Allocating Buffer*************************/
		data_buffer=(char*)malloc(rx_msglen*sizeof(char));
		if(data_buffer==NULL){
			if(verbose>=VERBOSITY_LOW) cout<<"RX: Error Allocating RX buffer"<<endl;
			/**********************************************************************************/
			pthread_mutex_unlock(&(*Conn_ptr).socket_rx_mutex);
			return(-2);
			/**********************************************************************************/
			}
		/*****************************************************************************/
		index=0;
		do{
			bytes_recvd=recv((*Conn_ptr).client_sock,(char*)(data_buffer+index),rx_msglen-index,0);
			index+=bytes_recvd;
			}
		while(bytes_recvd<rx_msglen && bytes_recvd!=SOCKET_ERROR);
		/**********************************************************************************/
		(*Conn_ptr).RXMsgLen=index%CONN_RX_BUFF_LEN;
		if((*Conn_ptr).RXBuffer!=NULL)
			memcpy((void*)(*Conn_ptr).RXBuffer,(void*)data_buffer,index%CONN_RX_BUFF_LEN);
		free(data_buffer);
		/**********************************************************************************/
		if(verbose>=VERBOSITY_MEDIUM)
								cout<<"RX Received "<<(*Conn_ptr).RXMsgLen<< " bytes"<<endl;
		if(bytes_recvd==SOCKET_ERROR)
		   if(verbose>=VERBOSITY_MEDIUM)
						printf("Error Receiveing Message\r\n");

		/**********************************************************************************/
		pthread_mutex_unlock(&(*Conn_ptr).socket_rx_mutex);
		return(bytes_recvd);
		/**********************************************************************************/

	/**********************************************************************************/

}
int WriteDatatoSocket(Connection * Conn_ptr, char* str_buffer, int str_len, char* binary_buffer,int binary_len){
	//every msg transmission is made of a string follwed by a binary paiload if needed
	//two size are transmitted one for the total and one for the strlen
	int sent_bytes;
	int tot_datalen=str_len+binary_len+sizeof(str_len);
	/*******************************************************/
	/*******************************************************/
	pthread_mutex_lock(&(*Conn_ptr).socket_tx_mutex);
	/*******************************************************/

	/*******************sending tot_datalen******************/
	sent_bytes=send_int_over_the_net((*Conn_ptr).client_sock,tot_datalen);
	if(sent_bytes==SOCKET_ERROR){
		if(verbose>=VERBOSITY_LOW)
			printf("TX: Error accessing interface socket \n");

		/*******************************************************/
		pthread_mutex_unlock(&(*Conn_ptr).socket_tx_mutex);
		/*******************************************************/
		return(sent_bytes);
		}

	else
		if(verbose>=VERBOSITY_HIGH)
			printf("TX: TotMsglen %d (%d)bytes Sent\n",tot_datalen,sent_bytes);
	/*******************sending strlen******************/
		sent_bytes=send_int_over_the_net((*Conn_ptr).client_sock,str_len);
		if(sent_bytes==SOCKET_ERROR){
			if(verbose>=VERBOSITY_LOW)
				printf("TX: Error accessing interface socket \n");

			/*******************************************************/
			pthread_mutex_unlock(&(*Conn_ptr).socket_tx_mutex);
			/*******************************************************/
			return(sent_bytes);
			}

		else
			if(verbose>=VERBOSITY_HIGH)
				printf("TX: StrMsglen %d (%d)bytes Sent\n",str_len,sent_bytes);

	/*******************sending str data******************/
	sent_bytes=send((*Conn_ptr).client_sock,(const char*)str_buffer,str_len,0);
	if(sent_bytes==SOCKET_ERROR)
		if(verbose>=VERBOSITY_LOW)
			printf("TX: Error accessing interface socket \n");

	/*******************sending bynary data******************/
	if(binary_len>0 && binary_buffer!=NULL){
			sent_bytes=send((*Conn_ptr).client_sock,(const char*)binary_buffer,binary_len,0);
			if(sent_bytes==SOCKET_ERROR)
				if(verbose>=VERBOSITY_LOW)
					printf("TX: Error accessing interface socket \n");
	}

	/*******************************************************/
		pthread_mutex_unlock(&(*Conn_ptr).socket_tx_mutex);
	/*******************************************************/
	return(sent_bytes);
}
/********************************************************************************************************/
// ": * ? "< > |"
int filename_check(const char* input_str){
	if(	strlen(input_str)<=0 ||
		strlen(input_str) > MAX_FILENAME_STR_LENGTH
		)
	return(-1);

	if (	strchr(input_str,':')!=NULL ||
			strchr(input_str,'*')!=NULL ||
			strchr(input_str,'?')!=NULL ||
			strchr(input_str,'"')!=NULL ||
			strchr(input_str,'<')!=NULL ||
			strchr(input_str,'>')!=NULL ||
			strchr(input_str,'|')!=NULL
			)
	return(-2);
	return(1);
}
/********************************************************************************************************/
int update_run_sett(command incoming_cmd, Connection* Conn_ptr){
	int ret=0;
	if(incoming_cmd.cmd_param_number>=1){
		if(strcmp(incoming_cmd.cmd_parameter[0],"DATA")==0){
			/**********************************/
			(*Conn_ptr).Run.Type=DATA;
			(*Conn_ptr).Run.FlatEn=ENABLED;
			strncpy((*Conn_ptr).Run.FF_filetoread_name,"TempFF.fdat",MAX_FILENAME_STR_LENGTH);
		}
		else
		if(strcmp(incoming_cmd.cmd_parameter[0],"SUMMED_DATA")==0){
			/**********************************/
			(*Conn_ptr).Run.Type=SUMMED_DATA;
			(*Conn_ptr).Run.FlatEn=ENABLED;
			strncpy((*Conn_ptr).Run.FF_filetoread_name,"TempFF.fdat",MAX_FILENAME_STR_LENGTH);
			}
		else
		if(strcmp(incoming_cmd.cmd_parameter[0],"COLLECT_FLAT")==0){
			/**********************************/
			(*Conn_ptr).Run.Type=COLLECT_FLAT;
			(*Conn_ptr).Run.FlatEn=DISABLED;
			strncpy((*Conn_ptr).Run.FF_filetosave_name,"TempFF.fdat",MAX_FILENAME_STR_LENGTH);

			}
		else
		if(strcmp(incoming_cmd.cmd_parameter[0],"SUMMED_COLLECT_FLAT")==0){
			/**********************************/
			(*Conn_ptr).Run.Type=SUMMED_COLLECT_FLAT;
			(*Conn_ptr).Run.FlatEn=DISABLED;
			strncpy((*Conn_ptr).Run.FF_filetosave_name,"TempFF.fdat",MAX_FILENAME_STR_LENGTH);

			}
		else
			return(-2);
		/**********************default file saving *************************/

		(*Conn_ptr).Run.FileSavingEnabled=1;// or SAVE_OFF???
		strncpy((*Conn_ptr).Run.filetosave_name,"Temp.dat",MAX_FILENAME_STR_LENGTH);
		/**********************************/
	}

	if(incoming_cmd.cmd_param_number>=2){
		if(strcmp(incoming_cmd.cmd_parameter[1],"-")==0){
			(*Conn_ptr).Run.FileSavingEnabled=0;
		}
		else
		{
			(*Conn_ptr).Run.FileSavingEnabled=1;
			ret=filename_check(incoming_cmd.cmd_parameter[1]);
			if(ret>0)
				strncpy((*Conn_ptr).Run.filetosave_name,incoming_cmd.cmd_parameter[1],MAX_FILENAME_STR_LENGTH);
			else
				strncpy((*Conn_ptr).Run.filetosave_name,"Unknown.dat",MAX_FILENAME_STR_LENGTH);
		}
	}
	if(incoming_cmd.cmd_param_number==3){
		if(strcmp(incoming_cmd.cmd_parameter[2],"-")==0){
			(*Conn_ptr).Run.FlatEn=DISABLED;
		}
		else
		{
			ret=filename_check(incoming_cmd.cmd_parameter[2]);
			if(strcmp(incoming_cmd.cmd_parameter[0],"COLLECT_FLAT")==0 || strcmp(incoming_cmd.cmd_parameter[0],"SUMMED_COLLECT_FLAT")==0 ){
				(*Conn_ptr).Run.FlatEn=DISABLED;
				if(ret>0)
					strncpy((*Conn_ptr).Run.FF_filetosave_name,incoming_cmd.cmd_parameter[2],MAX_FILENAME_STR_LENGTH);
				else
					strncpy((*Conn_ptr).Run.FF_filetosave_name,"FFtemp.fdat",MAX_FILENAME_STR_LENGTH);

			}
			else{
				(*Conn_ptr).Run.FlatEn=ENABLED;
				if(ret>0)
					strncpy((*Conn_ptr).Run.FF_filetoread_name,incoming_cmd.cmd_parameter[2],MAX_FILENAME_STR_LENGTH);
				else
					strncpy((*Conn_ptr).Run.FF_filetoread_name,"FFtemp.fdat",MAX_FILENAME_STR_LENGTH);
			}
		}

	}
	if(		incoming_cmd.cmd_param_number	!=1 &&
			incoming_cmd.cmd_param_number	!=2 &&
			incoming_cmd.cmd_param_number	!=3
			)
	return(-1);
	else
		return(1);
}
/********************************************************************************************************/
int update_sensor_sett_from_cmd(BOX *temp_box,command incoming_cmd, Connection* Conn_ptr){
	SENSOR Sens;
	int ret=0;
	if(incoming_cmd.cmd_param_number==3){

		/******************************************************/
		Sens.ReadoutSchema = DEFAULT;
		/******************************************************/
		if(strcmp(incoming_cmd.cmd_parameter[0],"PII")==0)
			Sens.Asic=PII;
		else
		if(strcmp(incoming_cmd.cmd_parameter[0],"PIII")==0)
			Sens.Asic=PIII;
		else
			ret=-1;
		/******************************************************/
		if(strcmp(incoming_cmd.cmd_parameter[1],"CDTE")==0)
			Sens.Hybrid=CDTE;
		else
		if(strcmp(incoming_cmd.cmd_parameter[1],"GAAS")==0)
			Sens.Hybrid=GAAS;
		else
			ret=-2;
		/******************************************************/
		if(strcmp(incoming_cmd.cmd_parameter[2],"PX1")==0)
			Sens.Build=PX1;
		else

		if(strcmp(incoming_cmd.cmd_parameter[2],"PX2")==0)
			Sens.Build=PX2;
		else

		if(strcmp(incoming_cmd.cmd_parameter[2],"PX4")==0)
			Sens.Build=PX4;
		else

		if(strcmp(incoming_cmd.cmd_parameter[2],"PX8")==0)
			Sens.Build=PX8;
		else

		if(strstr(incoming_cmd.cmd_parameter[2],"PX8_MONO") != NULL){
			Sens.Build = PX8;
			/*************/
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO0") == 0)
				Sens.ReadoutSchema = MONO0;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO1") == 0)
				Sens.ReadoutSchema = MONO1;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO2") == 0)
				Sens.ReadoutSchema = MONO2;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO3") == 0)
				Sens.ReadoutSchema = MONO3;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO4") == 0)
				Sens.ReadoutSchema = MONO4;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO5") == 0)
				Sens.ReadoutSchema = MONO5;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO6") == 0)
				Sens.ReadoutSchema = MONO6;
			else
			if(strcmp(incoming_cmd.cmd_parameter[2],"PX8_MONO7") == 0)
				Sens.ReadoutSchema = MONO7;
			else
				Sens.ReadoutSchema = DEFAULT;
			}
			/*************/
		else
			ret=-3;
		/******************************************************/
	}
	else
		ret=-4;
	/**********************************************************/
	if(ret<0){
		if (verbose>=VERBOSITY_LOW)
			printf("update_sensor_sett_from_cmd(%d)",ret);
		return ret;
		}

	Conn_ptr->Sens.Asic		=	Sens.Asic;
	Conn_ptr->Sens.Hybrid	=	Sens.Hybrid;
	Conn_ptr->Sens.Build	=	Sens.Build;
	/**********************************************************/
	if(Sens.Build == PX1)
		(*Conn_ptr).ConnBox.data_port = DEFAULT_PX1_BOX_DATA_PORT;
	else
	if(Sens.Build == PX2)
		(*Conn_ptr).ConnBox.data_port = DEFAULT_PXX_BOX_DATA_PORT;
	else
	if(Sens.Build == PX8)
		(*Conn_ptr).ConnBox.data_port = DEFAULT_PXX_BOX_DATA_PORT;
	else
		(*Conn_ptr).ConnBox.data_port = PORTA;
	/**********************************************************/

	ret=edit_sensor_properties(Conn_ptr,Sens);
	if (ret<0) return(ret);
	apply_special_configurations_tobox(temp_box,(*Conn_ptr).Sens);
	return(ret);
}
/********************************************************************************************************/
int update_file_saving_style_from_cmd(command incoming_cmd, Connection* Conn_ptr){
	if(incoming_cmd.cmd_param_number==1)
	{
		if(strcmp(incoming_cmd.cmd_parameter[0],"SAVE_PX1_OLD_STYLE")==0){
			(*Conn_ptr).Run.SaveMode=SAVE_GLORIA_PX1_OLD_STYLE;
			if(verbose>=VERBOSITY_HIGH)
						printf( "SET FILE SAVING STYLE to (%d)\n"
								,(*Conn_ptr).Run.SaveMode);
			}
		else{
			(*Conn_ptr).Run.SaveMode=SAVE_OLD_STYLE;
			if(verbose>=VERBOSITY_HIGH)
						printf( "SET FILE SAVING STYLE to default(%d)\n"
								,(*Conn_ptr).Run.SaveMode);
			}
		return 1 ;

	}
	else{
		if(verbose>=VERBOSITY_HIGH)
			printf("Error Updating FILE SAVING STYLE\n");
		return -1;
		}
}
/********************************************************************************************************/
int createFileList(FileList *List){
	int i=0;
	char* temp_ptr;
	do{
		temp_ptr=(char*)calloc(MAX_PATH,sizeof(char));
		(*List).names[i]=temp_ptr;
		i++;}
	while(i<FILE_LIST_LEN && temp_ptr!=NULL);
	(*List).entries=0;
	if(temp_ptr==NULL){
		if(verbose>=VERBOSITY_LOW)
			printf("CreateFileList creation got errors. Only %d records where allocated\n",i-1);
		return(-1);
	}
	else
		return(i);

}
int FreeFileList(FileList *List){
	int i=0;
	for(i=0;i<FILE_LIST_LEN;i++)
		if((*List).names[i]!=NULL)
			free((*List).names[i]);
	(*List).entries=0;
	return(0);
}
int file_list_add_record(FileList* List_ptr,const char *char_ptr){
 if ( (*List_ptr).entries<FILE_LIST_LEN )
	 strncpy((*List_ptr).names[(*List_ptr).entries++],char_ptr,FILE_LIST_LEN);
 return((*List_ptr).entries);
}
int file_check_extension(const char* filename,const char* ext){
	char* temp_pchar;
	temp_pchar=strchr(filename,'.');
	if(temp_pchar==NULL)
		return(-1);
	if(strncmp(temp_pchar+1,ext,strlen(ext))==0 || strcmp(ext,".*")==0)
		return(1);
	else
		return(0);
}
/********************************************************************************************************/
int edit_map_sett_from_file(FFMapRecord *MapListRec_ptr,const char* filename){
	FILE *file;
	int header[FF_FILEHEADER_LENGHT],ret;
	file=fopen(filename,"rb");
	if(file!=NULL){
		ret=fread(
				header,
				sizeof(int),
				FF_FILEHEADER_LENGHT,
				file
				);
		fclose(file);

		if(ret!=FF_FILEHEADER_LENGHT)
			return(-1);
		}
	else
		return(-1);
	/****************/
	if(header[1] 	== 2)
		(*MapListRec_ptr).Reg=REG01;
	else
	if(header[4]	== 0)
		(*MapListRec_ptr).Reg=REG0;
	else
		(*MapListRec_ptr).Reg=REG1;
	/******/

	/******/
	(*MapListRec_ptr).frame_width	=	header[2];
	(*MapListRec_ptr).frame_height	=	header[3];
	(*MapListRec_ptr).Asic			=	header[6];
	(*MapListRec_ptr).Build			=	header[7];
	(*MapListRec_ptr).Hybrid		=	header[8];
	(*MapListRec_ptr).RunMode		=	(RUNMODE)header[9];
	/****************/
	return(ret);
	}
/********************************************************************************************************/
int edit_frame_sett_from_file(FFMapRecord *MapListRec_ptr,const char* filename){
	FILE *file;
	unsigned short header[FILE_HEADER_LENGHT],ret;
	file=fopen(filename,"rb");
	if(file!=NULL){
		ret=fread(
				header,
				sizeof(int),
				FILE_HEADER_LENGHT,
				file
				);
		fclose(file);

		if(ret!=FILE_HEADER_LENGHT)
			return(-1);
		}
	else
		return(-1);
	/****************/

	/****************/
	if(header[3] 	==  TWO_COLS)
		(*MapListRec_ptr).Reg=REG01;
	else
	if(header[3]	== ONE_COL1)
		(*MapListRec_ptr).Reg=REG1;
	else
		(*MapListRec_ptr).Reg=REG0;

	/*In all other cases REG0 is selected*/

	/******/
	(*MapListRec_ptr).frame_width	=	header[2];
	(*MapListRec_ptr).frame_height	=	header[1];
	(*MapListRec_ptr).Asic			=	0;
	(*MapListRec_ptr).Build			=	header[4];
	(*MapListRec_ptr).Hybrid		=	0;
	(*MapListRec_ptr).RunMode		=	(RUNMODE)header[3];
	/****************/
	return(ret);
	}
int populate_ffmap_list(FFMapList* MapList,const char* dir,const char* ext){
	FileList FileList;
	char complete_path[MAX_PATH];
	int i;

	/*********generic_file list*********/
	createFileList(&FileList);
	get_file_list((TCHAR *)dir, &FileList);
	/***********************************/
	if(verbose>=VERBOSITY_HIGH){
	printf("Directory %s\n",dir);
	for (i=0;i<FileList.entries;i++)
		printf("FileList[%d]-->\t%s\n",i,FileList.names[i]);
	}

	/***********************************/


	if(MapList!=NULL){

		(*MapList).entries=0;
		for (i=0;i<FileList.entries;i++)
			/******************************************/
			if(file_check_extension(FileList.names[i],ext)>0  && i < FF_MAP_LIST_LEN){
				strncpy((*MapList).Records[(*MapList).entries].filename,FileList.names[i],MAX_PATH);
				sprintf(complete_path,"%s/%s",dir,FileList.names[i]);
				edit_map_sett_from_file((&(*MapList).Records[(*MapList).entries]),complete_path);
				(*MapList).entries++;
				}
			/******************************************/
			if(verbose>=VERBOSITY_HIGH){
				printf("FF Map Search Folder:'%s'\n",dir);
				for(i=0;i<(*MapList).entries;i++)
					printf("Map[%d]-->\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n"
							,i
							,(*MapList).Records[i].filename
							,(*MapList).Records[i].frame_width
							,(*MapList).Records[i].frame_height
							,(*MapList).Records[i].Reg
							,(*MapList).Records[i].Asic
							,(*MapList).Records[i].Build
							,(*MapList).Records[i].Hybrid
							,(*MapList).Records[i].RunMode);
				}
			}
	else
		return(-1);

	/***********************************/
	FreeFileList(&FileList);
	return(0);
}
/********************************************************************************************************/
int populate_frame_list(FFMapList* MapList,const char* dir,const char* ext){
	FileList FileList;
	char complete_path[MAX_PATH];
	int i;

	/*********generic_file list*********/
	createFileList(&FileList);
	get_file_list((TCHAR *)dir, &FileList);
	/***********************************/
	if(verbose>=VERBOSITY_HIGH){
	printf("Directory %s\n",dir);
	for (i=0;i<FileList.entries;i++)
		printf("FileList[%d]-->\t%s\n",i,FileList.names[i]);
	}

	/***********************************/


	if(MapList!=NULL){

		(*MapList).entries=0;
		for (i=0;i<FileList.entries;i++)
			/******************************************/
			if(file_check_extension(FileList.names[i],ext)>0 && i < FF_MAP_LIST_LEN){
				strncpy((*MapList).Records[(*MapList).entries].filename,FileList.names[i],MAX_PATH);
				sprintf(complete_path,"%s/%s",dir,FileList.names[i]);
				edit_frame_sett_from_file((&(*MapList).Records[(*MapList).entries]),complete_path);
				(*MapList).entries++;
				}
			/******************************************/
			if(verbose>=VERBOSITY_HIGH){
				printf("Frame Data Files Search Folder:'%s'\n",dir);
				for(i=0;i<(*MapList).entries;i++)
					printf("Map[%d]-->\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n"
							,i
							,(*MapList).Records[i].filename
							,(*MapList).Records[i].frame_width
							,(*MapList).Records[i].frame_height
							,(*MapList).Records[i].Reg
							,(*MapList).Records[i].Asic
							,(*MapList).Records[i].Build
							,(*MapList).Records[i].Hybrid
							,(*MapList).Records[i].RunMode);
				}
			}
	else
		return(-1);

	/***********************************/
	FreeFileList(&FileList);
	return(0);
}
/********************************************************************************************************/

int stringfy_ffmap_list(FFMapList MapList,char* str,int str_len){
	const char* tag={"FFMAPLIST:"};
	char temp_str[MAX_MSG_STR_LENGTH]={0};
	char reg_tag[20];
	int i;

	if(str==NULL)
		return(-1);


	/********Header******/
	strncpy(str,tag,str_len);

//	strncat(str
//			,"Name Type Width Height\n"
//			,str_len-strlen(str)
//			);
	if(MapList.entries==0)
		strncat(str,"List is Empty",str_len-strlen(str));
	else{
	for(i=0;i<MapList.entries;i++){

		/********************/
		if(MapList.Records[i].RunMode == ONE_COL0)
			strncpy(reg_tag,"1Col_Reg0",20);
		else
		if(MapList.Records[i].RunMode == ONE_COL1)
			strncpy(reg_tag,"1Col Reg1",20);
		else
		if(MapList.Records[i].RunMode == TWO_COLS)
			strncpy(reg_tag,"2Cols",20);
		else
		if(MapList.Records[i].RunMode == ONE_COL_DTF)
			strncpy(reg_tag,"1ColDTF",20);
		else
		if(MapList.Records[i].RunMode == TWO_COLS_DTF)
			strncpy(reg_tag," 2Cols DTF",20);
		else
			strncpy(reg_tag,"Undefined",20);
		/********************/
		snprintf(temp_str
				,MAX_MSG_STR_LENGTH
				,"%s %s %d X %d\n"
				,MapList.Records[i].filename
				,reg_tag
				,MapList.Records[i].frame_width
				,MapList.Records[i].frame_height
				);
		strncat(str,temp_str,str_len-strlen(str));

		if(str_len==strlen(str) || MapList.entries == FF_MAP_LIST_LEN){
			strcpy(str+str_len-strlen("\nBufferFull\n")-2,
					"\nBufferFull\n");
			break;}
		}
	}
	if(verbose>=VERBOSITY_ULTRAHIGH){
		printf("reply string: %s\n",str);
		printf("strlen%d\n",strlen(str));
		}
	return(1);
}
/********************************************************************************************************/
int stringfy_frame_list(FFMapList MapList,char* str,int str_len){
	const char* tag={"DATAFILE_LIST:"};
	char temp_str[MAX_MSG_STR_LENGTH]={0};
	char reg_tag[20];
	int i;

	if(str==NULL)
		return(-1);


	/********Header******/
	strncpy(str,tag,str_len);

//	strncat(str
//			,"Name Type Width Height\n"
//			,str_len-strlen(str)
//			);
	if(MapList.entries==0)
		strncat(str,"List is Empty",str_len-strlen(str));
	else{
		for(i=0;i<MapList.entries;i++){

			/********************/
			if(MapList.Records[i].RunMode == ONE_COL0)
				strncpy(reg_tag,"1Col_Reg0",20);
			else
			if(MapList.Records[i].RunMode == ONE_COL1)
				strncpy(reg_tag,"1Col Reg1",20);
			else
			if(MapList.Records[i].RunMode == TWO_COLS)
				strncpy(reg_tag,"2Cols",20);
			else
			if(MapList.Records[i].RunMode == ONE_COL_DTF)
				strncpy(reg_tag,"1ColDTF",20);
			else
			if(MapList.Records[i].RunMode == TWO_COLS_DTF)
				strncpy(reg_tag," 2Cols DTF",20);
			else
				strncpy(reg_tag,"Undefined",20);
			/********************/
			snprintf(temp_str
					,MAX_MSG_STR_LENGTH
					,"%s %s %d X %d\n"
					,MapList.Records[i].filename
					,reg_tag
					,MapList.Records[i].frame_width
					,MapList.Records[i].frame_height
					);
			strncat(str,temp_str,str_len-strlen(str));

			if(str_len==strlen(str) || MapList.entries== FF_MAP_LIST_LEN){
				strcpy(str+str_len-strlen("\nBufferFull\n")-2,
						"\nBufferFull\n");
				break;}
			}
	}

	if(verbose>=VERBOSITY_ULTRAHIGH){
		printf("reply string: %s\n",str);
		printf("strlen%d\n",strlen(str));
		}
	return(1);
}

int create_folders(Connection* Conn_ptr,char*data_path,char* flat_towrite_path,char* flat_toread_path, char* default_path,int strlen){
	char temp_str_root[MAX_FILENAME_STR_LENGTH];
	char temp_str[MAX_FILENAME_STR_LENGTH];

	/****************************************************************/
		sprintf(temp_str_root,".\\SavedData");
		CreateDirectory(temp_str_root,NULL);
		sprintf(temp_str_root,"%s\\BOX_%06d",temp_str_root,(*Conn_ptr).ConnBox.SN);
		CreateDirectory(temp_str_root,NULL);
		/***********************************************/
		sprintf(temp_str,"%s\\DATA\\",temp_str_root);
		CreateDirectory(temp_str,NULL);

		strncpy((*Conn_ptr).filetosave_folder,temp_str,MAX_FILENAME_STR_LENGTH);

		sprintf(temp_str,"%s\\DATA\\%s",temp_str_root,(*Conn_ptr).Run.filetosave_name);

		if(data_path!=NULL)
			strncpy(data_path,temp_str,strlen);

		/***********************************************/
		sprintf(temp_str,"%s\\FLAT\\",temp_str_root);
		CreateDirectory(temp_str,NULL);

		strncpy((*Conn_ptr).FF_filetoread_folder,temp_str,MAX_FILENAME_STR_LENGTH);

		sprintf(temp_str,"%s\\FLAT\\%s",temp_str_root,(*Conn_ptr).Run.FF_filetoread_name);

		if(flat_toread_path!=NULL)
			strncpy(flat_toread_path,temp_str,strlen);

		sprintf(temp_str,"%s\\FLAT\\%s",temp_str_root,(*Conn_ptr).Run.FF_filetosave_name);

		if(flat_towrite_path != NULL)
			strncpy(flat_towrite_path,temp_str,strlen);
		/***********************************************/
		sprintf(temp_str,"%s\\DEFAULT\\",temp_str_root);
		CreateDirectory(temp_str,NULL);
		strncpy((*Conn_ptr).default_folder,temp_str,MAX_FILENAME_STR_LENGTH);

		sprintf(temp_str,"%s\\DEFAULT\\%s",temp_str,(*Conn_ptr).Run.filetosave_name);

		if(default_path!=NULL)
			strncpy(default_path,temp_str,strlen);
	/****************************************************************/
	return(1);
}

int edit_sensor_properties(Connection* Conn_ptr,SENSOR Sensor){
	//the following is a sensor typical configuration//
	(*Conn_ptr).Sens.Build=Sensor.Build;
	(*Conn_ptr).Sens.Asic=Sensor.Asic;
	(*Conn_ptr).Sens.Hybrid=Sensor.Hybrid;
	(*Conn_ptr).Sens.ReadoutSchema = Sensor.ReadoutSchema;
	(*Conn_ptr).Sens.separation_columns = Sensor.separation_columns;

	/**************************************/
	if(Sensor.Build==PX1)
		(*Conn_ptr).Sens.modules=1;
	else
	if(Sensor.Build==PX2)
		(*Conn_ptr).Sens.modules=2;
	else
	if(Sensor.Build==PX4)
		(*Conn_ptr).Sens.modules=4;
	else
	if(Sensor.Build==PX8){
		if (Sensor.ReadoutSchema == DEFAULT)
			(*Conn_ptr).Sens.modules = 8;
		else//MONO
			(*Conn_ptr).Sens.modules = 1;
		}
	else{
		(*Conn_ptr).Sens.modules=0;
		if(verbose>=VERBOSITY_LOW)
			printf("edit_sensor_properties: Wrong Sensor Build(%d)",Sensor.Build);
		return(-1);}
	/**************************************/
	if(Sensor.Asic==PII){
		(*Conn_ptr).Sens.matrix_size_pxls	=PII_MATRIX_DIM_WORDS;
		(*Conn_ptr).Sens.cols				=PII_PIXIE_COLS;
		(*Conn_ptr).Sens.rows				=PII_PIXIE_ROWS;
		(*Conn_ptr).Sens.dout				=PII_PIXIE_DOUTS;
		(*Conn_ptr).Sens.cols_per_dout		=PII_COLS_PER_DOUT;
		(*Conn_ptr).Sens.bit_per_cnt_std	=PII_STD_COUNTER_CODE_DEPTH;
		(*Conn_ptr).Sens.bit_per_cnt_short	=PII_STD_COUNTER_CODE_DEPTH;
		(*Conn_ptr).Sens.autocal_bit_cnt	=PII_AUTOCAL_CODE_DEPTH;
		(*Conn_ptr).Sens.autocal_regs		=PII_AUTOCAL_REGS;
		(*Conn_ptr).Sens.cnt_regs			=PII_COUNTER_REGS;
		(*Conn_ptr).Sens.separation_columns	=PII_SEPARATION_COLUMNS_INPXX;
		(*Conn_ptr).Sens.pixel_arr			=EXAGON;
		}
	else
		{
		(*Conn_ptr).Sens.matrix_size_pxls	=PIII_MATRIX_DIM_WORDS;
		(*Conn_ptr).Sens.cols				=PIII_PIXIE_COLS;
		(*Conn_ptr).Sens.rows				=PIII_PIXIE_ROWS;
		(*Conn_ptr).Sens.dout				=PIII_PIXIE_DOUTS;
		(*Conn_ptr).Sens.cols_per_dout		=PIII_COLS_PER_DOUT;
		(*Conn_ptr).Sens.bit_per_cnt_std	=PIII_STD_COUNTER_CODE_DEPTH;
		(*Conn_ptr).Sens.bit_per_cnt_short	=PIII_SHORT_COUNTER_CODE_DEPTH;
		(*Conn_ptr).Sens.autocal_bit_cnt	=PIII_AUTOCAL_CODE_DEPTH;
		(*Conn_ptr).Sens.autocal_regs		=PIII_AUTOCAL_REGS;
		(*Conn_ptr).Sens.cnt_regs			=PIII_COUNTER_REGS;
		(*Conn_ptr).Sens.separation_columns	=PIII_SEPARATION_COLUMNS_INPXX;
		(*Conn_ptr).Sens.pixel_arr			=SQUARE;
		}
	/***************************************************************/
	(*Conn_ptr).Frame.Height	=(*Conn_ptr).Sens.rows ;
	(*Conn_ptr).Frame.Width		=(*Conn_ptr).Sens.cols * (*Conn_ptr).Sens.modules ;
	(*Conn_ptr).Frame.bytes_per_pixel=2;

	(*Conn_ptr).Frame.Crop.crop_xmin = 0;
	(*Conn_ptr).Frame.Crop.crop_xmax = (*Conn_ptr).Frame.Width - 1 ;

	(*Conn_ptr).Frame.Crop.crop_ymin = 0;
	(*Conn_ptr).Frame.Crop.crop_ymax = (*Conn_ptr).Frame.Height -1 ;


	/***************************************************************/
	if(Sensor.Asic!=PII && Sensor.Asic!=PIII)
	{	if(verbose>=VERBOSITY_LOW)
			printf("edit_sensor_properties: Wrong Asic Type(%d)",Sensor.Asic);
		return(-2);
		}
	return(1);
}


/*****************************************************************************/
int apply_special_configurations_tobox(BOX *tempbox,SENSOR Sens){
	char temp_str[MAX_MSG_STR_LENGTH];
	int ret=0;
	/***********************************************************************************/
	if (Sens.Build==PX1){
		/***************************/
		//strncpy(temp_str,"SYS:! SET_MEAS_DEST_ADD 192.168.0.255 9999\n",MAX_MSG_STR_LENGTH);
		//ret=pixirad_send_cmd(tempbox,temp_str, NULL,0,0);
		//cambiata strategia: adesso il numero di porta cambiato nel udp thread
		/***************************/
		snprintf(temp_str,MAX_MSG_STR_LENGTH,"SYS:! SET_ALARMS %d %d %d %d %d %d %d\n",
				DEF_BOX_RH_AL,
				DEF_BOX_TC_AL,
				DEF_BOX_TH_AL,
				DEF_BOX_RH_AL_ST,
				DEF_BOX_TC_AL_ST,
				DEF_BOX_TH_AL_ST,
				DEF_BOX_TH_AL_ST_MSG);
		ret=pixirad_send_cmd(tempbox,temp_str, NULL,0,0);
		/***************************/
		snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! INIT %d %d %d %d\n",
				DEF_BOX_TCOLD,
				DEF_BOX_TMNG_STAT,
				DEF_HV_VAL,
				0);
		ret=pixirad_send_cmd(tempbox,temp_str, NULL,0,0);
		/***************************/

	}
	/*********************************************************************************/
	if(Sens.Build == PX8){
		if( Sens.ReadoutSchema == MONO0 )
			snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP0\n");
		else
			if( Sens.ReadoutSchema == MONO1 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP1\n");
		else
			if( Sens.ReadoutSchema == MONO2 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP2\n");
		else
			if( Sens.ReadoutSchema == MONO3 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP3\n");
		else
			if( Sens.ReadoutSchema == MONO4 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP4\n");
		else
			if( Sens.ReadoutSchema == MONO5 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP5\n");
		else
			if( Sens.ReadoutSchema == MONO6 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP6\n");
		else
			if( Sens.ReadoutSchema == MONO7 )
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA CHIP7\n");
			else
				snprintf(temp_str,MAX_MSG_STR_LENGTH,"DAQ:! SET_RO_SCHEMA DEFAULT\n");

		/********/
		ret=pixirad_send_cmd(tempbox,temp_str, NULL,0,0);
		/********/

		}


	/***********************************************************************************/
	return (ret);
}


int get_crop_from_cmd(command incoming_cmd, Connection*Conn){

	if(incoming_cmd.cmd_param_number != 4)
		return -1;
	else{
		if(	atoi(incoming_cmd.cmd_parameter[0]) < 0
			|| atoi(incoming_cmd.cmd_parameter[1]) < 0
			|| atoi(incoming_cmd.cmd_parameter[2]) < 0
			|| atoi(incoming_cmd.cmd_parameter[3]) < 0
			|| atoi(incoming_cmd.cmd_parameter[1]) >= (*Conn).Frame.Width
			|| atoi(incoming_cmd.cmd_parameter[3]) >= (*Conn).Frame.Height
			|| atoi(incoming_cmd.cmd_parameter[0]) >= atoi(incoming_cmd.cmd_parameter[1])
			|| atoi(incoming_cmd.cmd_parameter[2]) >= atoi(incoming_cmd.cmd_parameter[3])

			)
			return (-2);

		else
		{
			Conn->Frame.Crop.crop_xmin = atoi(incoming_cmd.cmd_parameter[0]);
			Conn->Frame.Crop.crop_xmax = atoi(incoming_cmd.cmd_parameter[1]);
			Conn->Frame.Crop.crop_ymin = atoi(incoming_cmd.cmd_parameter[2]);
			Conn->Frame.Crop.crop_ymax = atoi(incoming_cmd.cmd_parameter[3]);
			return 0;
		}

	}


}
/*********************************************************************************************/
int get_interpol_options_from_cmd(command incoming_cmd, Connection*Conn){

	if(incoming_cmd.cmd_param_number != 3)
		return -1;
	else{
		Conn->Frame.Interpoldata.pitch = atof(incoming_cmd.cmd_parameter[0]);
		if(strstr(incoming_cmd.cmd_parameter[1], "EN") != NULL){
			Conn->Run.SaveInterpolatedData = ENABLED;
		}
		else
			Conn->Run.SaveInterpolatedData = DISABLED;


		if(strstr(incoming_cmd.cmd_parameter[2], "EN") != NULL){
			Conn->Run.SaveInterpolatedDataTiff = ENABLED;
		}
		else
			Conn->Run.SaveInterpolatedDataTiff = DISABLED;

	}
}
