/*
 * pixieii_mammopix_daq.h
 *
 *  Created on: May 30, 2012
 *      Author: massimo
 */

#ifndef PIXIEII_DATA_UTILITIES_H_
#define PIXIEII_DATA_UTILITIES_H_

#ifdef _WIN32
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
#endif

//12 bit data fpga writing function Msb first
//PA0 SCLK
//PA1 SDATA to fpga
//PA2 SI_EN

#define PIXIE_SI_STATUS_REG   0
#define SER_INT_RST           0x1
#define SER_INT_START         0x2
#define CAL_MODE              0x00
#define CONF_MODE             0x0C
#define RD_CONF_MODE          0x04
#define DEF_CONF_MODE         0x08

#define PIXIEII_THSET_REG                 7
#define PIXIEII_REG0_1                    15
#define GLOBAL_CONF_REG_WR_OPCODE         0x0000
#define GLOBAL_DEFCONF_REG_WR_OPCODE      0x0020
#define GLOBAL_CONF_REG_RD_OPCODE         0x0010
#define COLUMN_BYPASS_WR_OPCODE           0x0030
#define COLUMN_BYPASS_RD_OPCODE           0x0040
#define COLUMN_BYPASS_WRDEF_OPCODE        0x0050
#define COLUMN_PAIR_SELECTION_OPCODE      0x0060
#define ALL_COLUMN_PAIR_SELECTION_OPCODE  0x0070
#define PIXEL_CONFIGURATION_OPCODE        0x0080
#define AUTOCALIBRATION_OPCODE            0x0090
#define COLUMN_BYPASS_WRDEFAULT_OPCODE    0x00a0
#define REG_0_BUFF_CODE                   0
#define REG_1_BUFF_CODE                   1
#define NETWORK_WAIT_ms                   10

#define PIXIEII_GLOBAL_CONF_REG   18
//nota che PIXIEII_PIXEL_CONF_REG e' lo steso indirizzo di PIXIE_MEM_DATA_REG
#define PIXIEII_PIXEL_CONF_REG    12
#define PIXIEII_READ_OUT_MODE_REG 21
#define STANDARD_READOUT           0
#define READ_AUTOCAL_CODES         1

#define PIXIE_MEM_DATA_REG        12
#define PIXIE_MEM_DATA_LOADER_REG 11
#define PIXIE_SHUTTER_OPENING_REG 10
#define PIXIE_STIMULI_REG_STATUS  1
#define PIXIEII_MEM_WR_CLK        0x4000
//#define MEM_WR_EN               0x8000
#define PIXIEII_FIFO_RESET        0x8000
#define MEM_ADD_MASK              0x03ff
#define EN_WR_INJ                 0x1
#define COUNT_MODE                0x0
#define SERIALIZE                 0x8
#define TESTENABLE                0x1
#define SHUTTEREND_FLAG           0x20
#define MASKREAD                  0x4
#define PXDIN                     0x2
#define PSCNT_WIDTH               15
#define PSTABLE_DEPTH             32768

#define PIXELS_IN_LAST_SECTOR         15232// the last sector contains only 15 columns
#define PIXELS_IN_LAST_SECTOR_MAP     15232
#define PIXIE_COLS                    512
#define PIXIE_ROWS                    476
#define PIXIE_DOUTS                   16
#define SECTORS_IN_PIXIE              16
#define COLS_PER_DOUT                 32
#define COLS_PER_DOUT_LAST_SECTOR     32
#define PIXIEII_COLUMN_PAIRS          256
#define HEADER_LENGTH                 10
#define FRAME_LENGTH                  (PIXIE_DOUTS*COLS_PER_DOUT*PIXIE_ROWS*2)
#define GROUPED_COLS_MAP              (COLS_PER_DOUT)
#define PIXELS_IN_SECTOR_MAP          (COLS_PER_DOUT*PIXIE_ROWS)
#define MATRIX_DIM                    (PIXIE_COLS*PIXIE_ROWS*2)
#define MATRIX_DIM_WORDS              (PIXIE_COLS*PIXIE_ROWS)
#define DATA_BUFFER_DEPHT             (COLS_PER_DOUT*PIXIE_ROWS*PIXIE_DOUTS)


#define BUFFER_HDR_TAG            0xffff
#define RESET_BUFFER_HDR_TAG      0xfff0
#define FLE_END_TAG               0xfffe

#define NEG_MODE_OFF 745

#define GRADIENT                    0
#define LINE_45                     1
#define SOMB                        2
#define INJ                         1
#define QUIET                       0
#define PIXIE_CLOCKSEL_REG          2
#define PIXIE_AUTOCAL_CLOCKSEL_REG  7
#define PIXIE_WRITEPULSES_REG       8
#define PIXIE_OUTSEL_REG            3
#define PIXIE_CHIPSEL_REG           17
#define PIXIE_CLOCK_CNT_LSW_REG     4
#define PIXIE_CLOCK_CNT_MSW_REG     5
#define PIXIE_INJ_FREQ_REG          6

void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth);
unsigned short * conversion_table_allocation(void);
unsigned short * databuffer_allocation(unsigned long size);
int databuffer_sorting(unsigned short *buffer_a);
int map_data_buffer_on_pixie(unsigned short *buffer_a);
int generate_test_buffer(unsigned short *buffer_a,unsigned short type);
void decode_pixie_data_buffer(unsigned short* table, unsigned short* databuffer);

void databuffer_filtering(unsigned short *buffer_a,unsigned short low_limit,unsigned short high_limit);
void check_data_consistency(unsigned short*data_buffer,unsigned short test_pattern);
int dummy_buffer_init(unsigned short*data_buffer, char *namefile);
void copy_databuff_to_netbuff(unsigned short* databuff, unsigned short* netbuff,unsigned char position);


#endif /* PIXIEII_MAMMOPIX_DAQ_H_ */
