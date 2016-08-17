/*
 * PIXIEII_data_utilities_v2.h
 *
 *  Created on: Jul 31, 2014
 *      Author: massimo
 */

#ifndef PIXIEII_DATA_UTILITIES_V2_H_
#define PIXIEII_DATA_UTILITIES_V2_H_


#define WINDOWS


/*********************************************************************************************************/
//#define PIXIEIII 1
/*********************************************************************************************************/





#include "pxrd2_interface_misc.h"
#include "build_ver.h"

//12 bit data fpga writing function Msb first
//PA0 SCLK
//PA1 SDATA to fpga
//PA2 SI_EN

#define PIXIE_SI_STATUS_REG 0
#define SER_INT_RST			0x1
#define SER_INT_START		0x2
#define CAL_MODE			0x00
#define CONF_MODE			0x0C
#define	RD_CONF_MODE		0x04
#define	DEF_CONF_MODE		0x08



#define PIXIEII_THSET_REG 	7
#define PIXIEII_REG0_1		15
#define GLOBAL_CONF_REG_WR_OPCODE			0x0000
#define GLOBAL_DEFCONF_REG_WR_OPCODE		0x0020
#define GLOBAL_CONF_REG_RD_OPCODE			0x0010
#define COLUMN_BYPASS_WR_OPCODE				0x0030
#define COLUMN_BYPASS_RD_OPCODE				0x0040
#define COLUMN_BYPASS_WRDEF_OPCODE			0x0050
#define COLUMN_PAIR_SELECTION_OPCODE		0x0060
#define ALL_COLUMN_PAIR_SELECTION_OPCODE	0x0070
#define PIXEL_CONFIGURATION_OPCODE			0x0080
#define AUTOCALIBRATION_OPCODE				0x0090
#define COLUMN_BYPASS_WRDEFAULT_OPCODE		0x00a0
#define REG_0_BUFF_CODE						0
#define REG_1_BUFF_CODE						1
#define NETWORK_WAIT_ms						10


#define PIXIEII_GLOBAL_CONF_REG 18
//nota che PIXIEII_PIXEL_CONF_REG e' lo steso indirizzo di PIXIE_MEM_DATA_REG
#define PIXIEII_PIXEL_CONF_REG 12
#define PIXIEII_READ_OUT_MODE_REG 21
#define STANDARD_READOUT	0
#define READ_AUTOCAL_CODES	1

#define PIXIE_MEM_DATA_REG 12
#define	PIXIE_MEM_DATA_LOADER_REG 11
#define PIXIE_SHUTTER_OPENING_REG 10
#define PIXIE_STIMULI_REG_STATUS  1
#define PIXIEII_MEM_WR_CLK		0x4000
//#define MEM_WR_EN		0x8000
#define PIXIEII_FIFO_RESET		0x8000
#define MEM_ADD_MASK	0x03ff
#define EN_WR_INJ		0x1
#define	COUNT_MODE		0x0
#define SERIALIZE		0x8
#define TESTENABLE		0x1
#define SHUTTEREND_FLAG	0x20
#define	MASKREAD		0x4
#define PXDIN			0x2

#define PSCNT_WIDTH		15
#define	PSTABLE_DEPTH	32768

//#ifndef PIXIEIII
/*****************************************************************************************/
#define PII_AUTOCAL_CODE_DEPTH			5
#define	PII_STD_COUNTER_CODE_DEPTH		15
#define PII_PIXIE_COLS		 			512
#define PII_PIXIE_ROWS		 			476
#define PII_PIXIE_DOUTS					16
#define PII_COLS_PER_DOUT				32
#define PII_PIXELS_IN_SECTOR_MAP		(PII_COLS_PER_DOUT*PII_PIXIE_ROWS)
#define PII_MATRIX_DIM_WORDS 			(PII_PIXIE_COLS*PII_PIXIE_ROWS)
#define PII_AUTOCAL_REGS				1
#define PII_COUNTER_REGS				2
#define PII_SEPARATION_COLUMNS_INPXX	3


/*****************************************************************************************/

#define PIII_AUTOCAL_CODE_DEPTH			9
#define PIII_STD_COUNTER_CODE_DEPTH		15
#define PIII_SHORT_COUNTER_CODE_DEPTH	7
#define PIII_PIXIE_COLS		 			512
#define PIII_PIXIE_ROWS		 			402
#define PIII_PIXIE_DOUTS				16
#define PIII_COLS_PER_DOUT				32
#define PIII_PIXELS_IN_SECTOR_MAP		(PIII_COLS_PER_DOUT*PIII_PIXIE_ROWS)
#define PIII_MATRIX_DIM_WORDS 			(PIII_PIXIE_COLS*PIII_PIXIE_ROWS)
#define PIII_AUTOCAL_REGS				3
#define PIII_COUNTER_REGS				2
#define PIII_SEPARATION_COLUMNS_INPXX	4
/*****************************************************************************/
#define OLDRAW_HEADER_LENGHT			10
#define FILE_HEADER_LENGHT				12
#define FF_FILEHEADER_LENGHT			12
/*****************************************************************************/
#define BUFFER_HDR_TAG			0xffff
#define RESET_BUFFER_HDR_TAG	0xfff0
#define FLE_END_TAG				0xfffe

/*****************************************************************************/
#define NEG_MODE_OFF 745


/*****************************************************************************/
#define OUTLIER_STDEV 8
#define MEDIAN_RADIUS 2

#define INTERPOLA_PITCH_DEFAULT 60.0



#define GRADIENT		0
#define LINE_45			1
#define SOMB			2
#define INJ				1
#define	QUIET			0
#define PIXIE_CLOCKSEL_REG 2
#define PIXIE_AUTOCAL_CLOCKSEL_REG 7
#define PIXIE_WRITEPULSES_REG 8
#define PIXIE_OUTSEL_REG   3
#define PIXIE_CHIPSEL_REG 17
#define PIXIE_CLOCK_CNT_LSW_REG 4
#define PIXIE_CLOCK_CNT_MSW_REG	5
#define PIXIE_INJ_FREQ_REG		6








void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth);
unsigned short * conversion_table_allocation(SENSOR* Sens_ptr);
unsigned short * databuffer_allocation(unsigned long size);
void * buffer_allocation(unsigned long elems,int size_el);
int databuffer_sorting(unsigned short *buffer_a,SENSOR Sens);
int map_data_buffer_on_pixie(unsigned short *buffer_a,SENSOR Sens);
int map_data_buffer_on_pixieIII(unsigned short *buffer_a,SENSOR Sens);
int generate_test_buffer(unsigned short *buffer_a,unsigned short type,SENSOR Sens);
void decode_pixie_data_buffer(unsigned short* table, int table_depth,unsigned short* databuffer,int databuffer_len);

void databuffer_filtering(unsigned short *buffer_a,unsigned short low_limit,unsigned short high_limit,SENSOR Sens);
void check_data_consistency(unsigned short*data_buffer,unsigned short test_pattern);
int dummy_buffer_init(unsigned short*data_buffer,char *namefile,SENSOR Sens);
void copy_databuff_to_netbuff(unsigned short* databuff, unsigned short* netbuff,unsigned char position,SENSOR Sens);
void my_bytes_swap(unsigned short* us_ptr);
int convert_bit_stream_to_counts(int code_depth,unsigned short* source_memory_offset,
								unsigned short* destination_memory_offset,SENSOR Sens,int verbose);
//void get_pixie_raw_data(unsigned short*source_buff_ptr,unsigned short*dest_buff_ptr,unsigned short pixieii_modules,int decode,unsigned short* conv,int code_depth);
void get_pixie_raw_data(unsigned short*source_buff_ptr,unsigned short*dest_buff_ptr,ACQ_PROP Acq_sett,SENSOR Sens);

/*********************************************************************************************************************************/
#define PIII_PRANDOM_15BITS_B0 13
#define PIII_PRANDOM_15BITS_B1 14
#define PIII_PRANDOM_07BITS_B0	5
#define PIII_PRANDOM_07BITS_B1	6
#define CONVERSIONTABLEDEPTH 32768
int GeneratePIIIConversionTable(unsigned short *table_ptr,int table_size, int code_dept);
int InvertPIIIConversionTable(unsigned short *table_ptr,int table_size, int code_dept);
unsigned short * PIIIConversion_table_allocation(int code_depth);
void accumulate_frame16(FrameAccumulator16 *regacc_ptr,unsigned short*temp_us_ptr,Connection Conn_ptr);
void accumulate_frame32(FrameAccumulator32 *regacc_ptr,unsigned short*temp_us_ptr,Connection Conn_ptr);
int accumulators16_allocator(FrameAccumulator16 *reg0acc16,FrameAccumulator16 *reg1acc16,Connection Conn);
int accumulators32_allocator(FrameAccumulator32 *reg0acc32,FrameAccumulator32 *reg1acc32,Connection Conn);
int normaps_allocator(FFMap* FFMap_ptr,Connection Conn);
int normaps_fetcher(FFMap* FFMap_ptr,Connection* Conn_ptr,char* filename);
int calmap_fetcher(FFMap* FFMAP_ptr, Connection *Conn);
int flat(unsigned short *temp_us_ptr,FFMap FlatMap,ACQ_PROP AcqSett,Connection* Conn_ptr);
int calculate_and_save_flatfield(FrameAccumulator32 reg0acc,FrameAccumulator32 reg1acc,Connection Conn,char* filename);
int apply_pixel_calibration(FFMap CalMap,unsigned short *temp_us_ptr,Connection Conn);
int apply_pixel_median_filtering(Connection Conn);
int crop_Image(unsigned short *temp_us_ptr,Connection *Conn);
int remap_cropped_Image(unsigned short* temp_us_ptr, Connection *Conn_ptr);
#endif /* PIXIEII_DATA_UTILITIES_V2_H_ */
