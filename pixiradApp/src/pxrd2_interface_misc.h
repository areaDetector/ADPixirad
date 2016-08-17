/*
 * pxrd2_interface_misc.h
 *
 *  Created on: Jul 31, 2014
 *      Author: massimo
 */

#ifndef PXRD2_INTERFACE_MISC_H_
#define PXRD2_INTERFACE_MISC_H_
#define NOVERBOSE 0
#define VERBOSITY_LOW 1
#define VERBOSITY_MEDIUM 2
#define VERBOSITY_HIGH 3
#define VERBOSITY_VERYHIGH 4
#define VERBOSITY_ULTRAHIGH 5

extern int verbose;
#define MACHINE_SHORT_DELAY_MS 10
#define MACHINE_LONG_DELAY_MS 500
#define HUMAN_SHORT_DELAY_MS 1000

/**************************************************************/
#define MAX_FILENAME_STR_LENGTH 1000
#define MAX_MSG_STR_LENGTH 10000
#define MAX_SHORT_MSG_STR_LEN 50
#define MAX_INPUT_STRING_LENGTH MAX_MSG_STR_LENGTH
#define MAX_CMD_PARAMETERS 15
#define CMD_STR_ELEMENT_MAX_LENGTH 50
/**************************************************************/
#define CONN_RX_BUFF_LEN 8192
#define CONN_TX_BUFF_LEN 8192
/**************************************************************/
#define SRV_TAG_STR "SRV"
#define DAQ_TAG_STR "DAQ"
#define SYS_TAG_STR "SYS"
/**************************************************************/

//#include <windows.h>
//#include <Windef.h>
//#include <winsock2.h>
//#include "pthread.h"
//#include "BoxUtilities_v2.h"
//#include "interface_command_utilities.h"
//#include "raw_image_correction_module.h"
//#include <stdio.h>
//#include "build_ver.h"

/**************************************************************/
enum EnState{DISABLED,ENABLED};
enum ConnectionState{FREE,ESTABILISHED,CLOSING};
/**************************************************************/

enum ImageType{FULL_RAW,FULL_BMP,DOWNSAMPLED_RAW,DOWNSAMPLED_BMP};


//typedef struct{
//	ImageType ImageMode;
//	int			Width;
//	int 		Heigth;
//	void *		data_ptr;
//}IMAGE;

/**************************************************************/
enum npi_mode_t{NONPI, NPI,NPISUM};
enum RUNMODE{TWO_COLS,ONE_COL0,ONE_COL1,ONE_COL_DTF,TWO_COLS_DTF,UNDEFINED};
enum HVType{AUTOHV,STDHV};

typedef struct{

	float frames;
	float shutter_len_ms;
	float pause_ms;
	RUNMODE Mode;
	HVType HVMode;

}RunConfiguration;


/**************************************************************/
enum DETECTOR_BUILD{PX1,PX2,PX4,PX8};
enum ROSCHEMA_t {MONO0,MONO1, MONO2, MONO3, MONO4, MONO5, MONO6, MONO7, DEFAULT} ;
enum ASIC_TYPE{PII,PIII};
enum HYBRID_TYPE{CDTE,GAAS};
enum PIXEL_ARRANGEMENT{EXAGON,SQUARE};
typedef struct{
	unsigned short *ptr;
	int depth;
}CONV_TABLE;

typedef struct{
	DETECTOR_BUILD Build;
	ASIC_TYPE Asic;
	HYBRID_TYPE Hybrid;
	ROSCHEMA_t ReadoutSchema;
	npi_mode_t npi_mode;
	int matrix_size_pxls;
	int modules;
	int	rows;
	int cols;
	int separation_columns;
	int dout;
	int cols_per_dout;
	int autocal_bit_cnt;
	int autocal_regs;
	int cnt_regs;
	int bit_per_cnt_std;
	int bit_per_cnt_short;
	int bit_parity;
	PIXEL_ARRANGEMENT pixel_arr;
	CONV_TABLE conv_table;
}SENSOR;

typedef struct{
	float energy;
	int 	reg;
	int 	align_err;
	int		is_autocal;
	int 	remaining_frames;
	int 	slot_id;
	int 	decode;
}ACQ_PROP;


//cropped image vertexes
typedef struct{
	int crop_xmin;
	int crop_xmax;
	int crop_ymin;
	int crop_ymax;
}CROP_t;

//interpolated image dimensions depends on interpolation pitch
typedef struct{
	float pitch;
	void* data;
	int rows;
	int cols;
}INTERPOL_t;

typedef struct{
	void* 	data_ptr;
	int Width;
	int Height;
	RUNMODE Mode;
	ImageType Image;
	int bytes_per_pixel;
	CROP_t Crop;
	INTERPOL_t Interpoldata;
}FRAME_PROP;

typedef struct{
	char ip_str[15];
	unsigned short port;
}HOST;



/**************************************************************/
enum RunState{IDLERUN,RUNNING,CLOSINGRUN,BREAKINGRUN,CLOSEDRUN};
enum RunType{DATA,SUMMED_DATA,COLLECT_FLAT,SUMMED_COLLECT_FLAT,AUTOCAL}; //The "SUMMED" prefix applyes only as a display options, not raw data saving
enum DataTransferType{OLD_DIFFERENT_SOCKET,NEW_SAME_SOCKET,NEW_DIFFERENT_SOCKET};
enum FileSaveType{SAVE_OFF,SAVE_OLD_STYLE,SAVE_NEW_STYLE,SAVE_GLORIA_PX1_OLD_STYLE};
enum Processing_t {DEFAULT_PROCESSING, CORRECTIONS, EX_TO_SQ_MAP_SP, NO_PROCESSING};


/**************************************************************/
typedef struct{
	int entries;
	unsigned short* buff;
}FrameAccumulator16;
typedef struct{
	int entries;
	unsigned int* buff;
}FrameAccumulator32;
typedef struct{
	int regmaplen;
	float* reg0buff;
	float* reg1buff;
}FFMap;
/**************************************************************/
#define FILE_LIST_LEN 200

typedef struct{
	char *names[FILE_LIST_LEN];
	int entries;
	int recordlen;
	int size;
}FileList;

/**************************************************************/


#define FF_MAP_LIST_LEN 500

enum RegType{REG0,REG1,REG01};


/**************************************************************/
void StrRunState(RunState State,char* deststr,int maxlen);

/**************************************************************/
RUNMODE getRunModeFromStr(const char*);
/**************************************************************/

#endif /* PXRD2_INTERFACE_MISC_H_ */
