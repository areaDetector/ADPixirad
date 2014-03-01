#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PIXIEII_data_utilities.h"


// this routine has been developed by SAndro, abd generates the lookup table
// for pixie counter conversion
void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth){
    //unsigned int clocks[32768];
    unsigned long potenze[16], bit1,bit2;
    unsigned long i,tempo;


    potenze[0]=1;
    for (i=1; i<(unsigned long)counterwidth+1; i++)
        potenze[i]=potenze[i-1]*2;

    clocks[0]=0;
    tempo=0;
    for(i=1; i<dim; i++){

        bit1=tempo&potenze[14];
        if(bit1 != 0)bit1=1;
        bit2=tempo&potenze[6];
        if(bit2 != 0)bit2=1;
        bit1=!(bit1^bit2);
        tempo=tempo*2+bit1;
        tempo=tempo%potenze[15];
        clocks[tempo]=i;

    }
    clocks[0]=0;
    return;
}

unsigned short * conversion_table_allocation(void){
      unsigned short *ush_ptr;
      ush_ptr=(unsigned short*)calloc(PSTABLE_DEPTH, sizeof(unsigned short));
      if (ush_ptr==NULL) {
          printf("\r\nCOversion Table:Memory allocation unsuccesfull!! Please contact an expert :-)");
          return(NULL);
      }
      else{
          genera_tabella_clock(ush_ptr,PSTABLE_DEPTH,PSCNT_WIDTH);
          return(ush_ptr);
    }
}

unsigned short * databuffer_allocation(unsigned long size){
    unsigned short *ush_ptr;
    ush_ptr=(unsigned short *)calloc(size, sizeof(unsigned short));
    if (ush_ptr==NULL) {
        printf("\r\nData:Memory allocation unsuccesfull!! Please contact an expert (Massimo) :-)");
        return(0);
    }
    else{
        return(ush_ptr);}
}

//databuffer_sorting arranges data from fpga in an array in which 16 sectors (31200 pixel each) are contiguos in memory
int databuffer_sorting(unsigned short *buffer_a){
    unsigned short PIXELS_IN_SECTOR;
    unsigned short    *buffer_b;
    PIXELS_IN_SECTOR=(COLS_PER_DOUT * PIXIE_ROWS);
    unsigned  long sector_cntr,pixel_cntr;
        buffer_b=(unsigned short*)calloc(PIXIE_COLS*PIXIE_ROWS, sizeof(unsigned short));
        if (buffer_b==NULL) {
            printf("\r\nDATA sorting:Memory allocation unsuccesfull!! Please contact an expert :-)");
            return(0);
        }

        //in the original buffer same index(position in the sector) pixels are stored contiguosly
        //the filling starts from the end of each sector up to the beginning
        //this because the first data you get from fpga actually is the last pixel of the sector
        //you are reading out
        for(sector_cntr=0;sector_cntr<SECTORS_IN_PIXIE-1;sector_cntr++){
            for(pixel_cntr=0;pixel_cntr<PIXELS_IN_SECTOR;pixel_cntr++){
                buffer_b[((sector_cntr+1)*PIXELS_IN_SECTOR_MAP)-pixel_cntr-1]=buffer_a[sector_cntr+pixel_cntr*SECTORS_IN_PIXIE];
            }
        }

        //last sector hasn't as many pixels as others so filling for it, must start before than PIXEL_IN_SECTORS-1
        // it has to start from PIXELS_IN_LAST_SECTOR-1
        //at this point sector_cntr is already at 15
        for(pixel_cntr=0;pixel_cntr<PIXELS_IN_LAST_SECTOR;pixel_cntr++){
                buffer_b[((sector_cntr)*PIXELS_IN_SECTOR_MAP+PIXELS_IN_LAST_SECTOR_MAP)-pixel_cntr-1] = 
                    buffer_a[sector_cntr+pixel_cntr*SECTORS_IN_PIXIE];
        }

        //copying buffer to the original one and adding
        for(pixel_cntr=0;pixel_cntr<(MATRIX_DIM_WORDS);pixel_cntr++)
            buffer_a[pixel_cntr]=buffer_b[pixel_cntr];
        free(buffer_b);
        return(1);
}

//map_data_buffer_on_pixie rearranges data in PIXIE layout taking in account the "snake" readout architecture
int map_data_buffer_on_pixie(unsigned short *buffer_a){
    unsigned short* temp_col;
    unsigned short col_cntr,row_cntr;
        temp_col=(unsigned short*)calloc(PIXIE_ROWS, sizeof(unsigned short));
        if (temp_col==NULL) {
            printf("\r\nDATA mapping:Memory allocation unsuccesfull!! Please contact an expert :-)");
            return(0);
        }

        //sectors 0,2,4,6,8,10,12,14 start with the first column(800 pix) in the right dir
        //sectors 1,3,5,7,8,11,13,15 start with the first column(800 pix) in the reversed dir
        //in general even index columns has the right dir and odd ones are reversed
        for(col_cntr=0;col_cntr<PIXIE_COLS;col_cntr++){
            if ((col_cntr%2)){//only odd index columns are reversed
                for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                    temp_col[PIXIE_ROWS-row_cntr-1]=buffer_a[col_cntr*PIXIE_ROWS+row_cntr];
                }
                for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                    buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=temp_col[row_cntr];
                }
            }
        }
        //mirroring
//        for(col_cntr=0;col_cntr<PIXIE_COLS/2;col_cntr++){
//            for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//                temp_col[row_cntr]=buffer_a[col_cntr*PIXIE_ROWS+row_cntr];
//            }
//            for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//                buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=buffer_a[(PIXIE_COLS-col_cntr-1)*PIXIE_ROWS+row_cntr];
//            }
//            for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//                buffer_a[(PIXIE_COLS-col_cntr-1)*PIXIE_ROWS+row_cntr]=temp_col[row_cntr];
//            }
//        }

        free(temp_col);
        return(1);}




int generate_test_buffer(unsigned short *buffer_a,unsigned short type){
    unsigned int col_cntr,row_cntr;
    switch(type){
    case GRADIENT:{//this is implemented with the "snake" structure
                    for(col_cntr=0;col_cntr<PIXIE_COLS;col_cntr++){
                        if (!(col_cntr%2)){
                            for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                                buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=row_cntr+col_cntr;
                            }
                        }

                        else{
                             for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                                buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=PIXIE_ROWS+col_cntr-row_cntr;
                            }
                        }
                    }
                    break;
                  }
    case LINE_45:{//this is implemented with no "snake" structure
                    for(col_cntr=0;col_cntr<PIXIE_COLS;col_cntr++){
                        for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                            if(row_cntr==col_cntr)
                                buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=500;
                            else
                                buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=0;
                        }
                    }
                    break;
                  }

    case SOMB:{
                  //this is implemented with no "snake" structure
                  for(col_cntr=0;col_cntr<PIXIE_COLS;col_cntr++){
                      for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
                              buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=
                                  10000/(1+(row_cntr-PIXIE_ROWS/2)*(row_cntr-PIXIE_ROWS/2)+
                                  (col_cntr-PIXIE_COLS/2)*(col_cntr-PIXIE_COLS/2));

                      }
                  }
                  break;
                }
    default:{
            printf("\n unknown test reqested");
        }
    }
    return(1);
}

void decode_pixie_data_buffer(unsigned short* table, unsigned short* databuffer){
    int i;
    for(i=0;i<MATRIX_DIM_WORDS;i++)
        databuffer[i]=table[(0x7fff)& databuffer[i]];
}




void databuffer_filtering(unsigned short *buffer_a,unsigned short low_limit,unsigned short high_limit){
    unsigned long i,suppressed=0;

    for(i=HEADER_LENGTH;i<(PIXIE_COLS*PIXIE_ROWS)+HEADER_LENGTH;i++)
        if ((buffer_a[i]<low_limit)||(buffer_a[i]>high_limit)){
                buffer_a[i]=0;
                suppressed++;
        }

    printf("\r\n %.2f suppressed channels",(float)((suppressed/(PIXIE_COLS*PIXIE_ROWS))*100));
}


void check_data_consistency(unsigned short*data_buffer,unsigned short test_pattern){
    unsigned long i,total_failed_channels=0, partial_failed_channels[PIXIE_DOUTS]={0};
    double percentage_total_failed_channels,percentage_partial_failed_channels[PIXIE_DOUTS]={0};
    for(i=0;i<(PIXIE_COLS*PIXIE_ROWS);i++){
        if(data_buffer[i+HEADER_LENGTH]!=test_pattern) {
                        total_failed_channels++;
                        partial_failed_channels[(unsigned short)(i/(COLS_PER_DOUT*PIXIE_ROWS))]++;
        }
    }
    percentage_total_failed_channels=(double)total_failed_channels*100/(PIXIE_COLS*PIXIE_ROWS);
    for (i=0;i<PIXIE_DOUTS-1;i++)
        percentage_partial_failed_channels[i]=(double)partial_failed_channels[i]*100/(COLS_PER_DOUT*PIXIE_ROWS);
    percentage_partial_failed_channels[PIXIE_DOUTS-1]=
        (double)partial_failed_channels[PIXIE_DOUTS-1]*100/(COLS_PER_DOUT_LAST_SECTOR*PIXIE_ROWS);
    printf("\r\n\t Found a total of\t%3.2f%%\tbad readings", percentage_total_failed_channels);
    for(i=0;i<PIXIE_DOUTS;i++)
        printf("\r\n\t Sector [%2ld]:\t\t%3.2f%%\tbad readings",i,percentage_partial_failed_channels[i]);
}

int dummy_buffer_init(unsigned short*data_buffer,char *namefile){
    FILE * imagefile;
    unsigned char dummy_buff[FRAME_LENGTH/2];
    unsigned long i;
    imagefile=fopen(namefile,"r");
    if(imagefile==NULL){
        for(i=HEADER_LENGTH;i<FRAME_LENGTH/2;i++)
        data_buffer[i]=0;
        return(1);
    }
    fread(dummy_buff,1,MATRIX_DIM_WORDS,imagefile);
    fclose(imagefile);
    for(i=0;i<MATRIX_DIM_WORDS;i++)
        data_buffer[i+HEADER_LENGTH]=dummy_buff[i];
    for(;i<FRAME_LENGTH/2;i++)
        data_buffer[i+HEADER_LENGTH]=0;

    return(0);
}
void copy_databuff_to_netbuff(unsigned short* databuff, unsigned short* netbuff,unsigned char position){
    unsigned long i=HEADER_LENGTH;
    if (position==0) {
        for(i=0;i<HEADER_LENGTH+MATRIX_DIM_WORDS;i++) netbuff[i]=databuff[i];
    }//MATRIX_DIM is intended to be for bytes
    else
        for(i=0;i<MATRIX_DIM_WORDS;i++) netbuff[i+HEADER_LENGTH+position*MATRIX_DIM_WORDS]=databuff[i+HEADER_LENGTH];
}
