/*
 * PIXIEII_data_utilities_v2.cpp
 *
 *  Created on: Jul 31, 2014
 *      Author: massimo
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef PIXIEII_DATA_UTILITIES_V2_CPP_
#define PIXIEII_DATA_UTILITIES_V2_CPP_
#include"PIXIEII_data_utilities_v2.h"

extern int verbose;

// this routine has been developed by SAndro, abd generates the lookup table
// for pixie counter conversion
void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth){
		//unsigned int clocks[32768];
		unsigned long potenze[16], bit1,bit2;
		unsigned long i,tempo;


		potenze[0]=1;
		for (i=1; i<counterwidth+1; i++)
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

unsigned short * conversion_table_allocation(SENSOR* Sens_ptr){
	unsigned short *ush_ptr;
	int i;
	if((*Sens_ptr).Asic==PII){
		(*Sens_ptr).conv_table.depth=PSTABLE_DEPTH;
		ush_ptr=(unsigned short*)calloc(PSTABLE_DEPTH, sizeof(unsigned short));
		(*Sens_ptr).conv_table.ptr=ush_ptr;
		if (ush_ptr==NULL) {
					if(verbose>=VERBOSITY_LOW)
						printf("Conversion Table:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
					return(NULL);}
		else{
			if(verbose>=VERBOSITY_HIGH)printf("Applied PII Cnversions\n");
			genera_tabella_clock(ush_ptr,PSTABLE_DEPTH,PSCNT_WIDTH);
			return(ush_ptr);}
			}
	else
		if((*Sens_ptr).Asic==PIII){
			(*Sens_ptr).conv_table.depth=PSTABLE_DEPTH;
			ush_ptr=PIIIConversion_table_allocation((*Sens_ptr).bit_per_cnt_std);
			(*Sens_ptr).conv_table.ptr=ush_ptr;
			if (ush_ptr==NULL) {
						if(verbose>=VERBOSITY_LOW)
							printf("COversion Table:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
						return(NULL);}
			else{
				if(verbose>=VERBOSITY_HIGH)printf("Applied PIII COnversions\n");
				return(ush_ptr);
				}
		}
	else{
		if(verbose>=VERBOSITY_LOW)
				printf("Conversion Table:Memory allocation unsuccesfull Because Unknown ASIC TYPE\n");
				(*Sens_ptr).conv_table.ptr=NULL;
				(*Sens_ptr).conv_table.depth=0;
				return(NULL);
		}
	}


unsigned short * databuffer_allocation(unsigned long size){
		unsigned short *ush_ptr;
		ush_ptr=(unsigned short *)calloc(size, sizeof(unsigned short));
		if (ush_ptr==NULL) {
					printf("\r\nData:Memory allocation unsuccesfull!! Please contact an expert (Massimo) :-)");
					return(0);}
		else{
			return(ush_ptr);}
			}
void* buffer_allocation(unsigned long elems,int size_el){
		void *ush_ptr;
		ush_ptr=calloc(elems , size_el); //malloc return zeroed buffers
		if (ush_ptr==NULL) {
					printf("\r\nBuffer Memory allocation unsuccesfull!! Please contact an expert (Massimo) :-)");
					return(NULL);}
		else{
			memset (ush_ptr , 0, elems * size_el);
			return(ush_ptr);}
			}


//databuffer_sorting arranges data from fpga in an array in which 16 sectors (31200 pixel each) are contiguos in memory
int databuffer_sorting(unsigned short *buffer_a,SENSOR Sens){
		int PIXELS_IN_SECTOR;
		unsigned short	*buffer_b;
		PIXELS_IN_SECTOR=(Sens.cols_per_dout * Sens.rows);
		int sector_cntr,pixel_cntr;
			buffer_b=(unsigned short*)calloc(Sens.matrix_size_pxls, sizeof(unsigned short));
			if (buffer_b==NULL) {
				printf("DATA sorting:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
				return(0);}

			//in the original buffer same index(position in the sector) pixels are stored contiguosly
			//the filling starts from the end of each sector up to the beginning
			//this because the first data you get from fpga actually is the last pixel of the sector
			//you are reading out
			for(sector_cntr=0; sector_cntr<Sens.dout; sector_cntr++){
				for(pixel_cntr=0; pixel_cntr<PIXELS_IN_SECTOR; pixel_cntr++){
					buffer_b[((sector_cntr+1)*PIXELS_IN_SECTOR) - pixel_cntr - 1]=buffer_a[sector_cntr + pixel_cntr*Sens.dout];
					}
				}


			//copying buffer to the original one
			for(pixel_cntr=0; pixel_cntr<Sens.matrix_size_pxls; pixel_cntr++)
				buffer_a[pixel_cntr]=buffer_b[pixel_cntr];

			free(buffer_b);
			return(1);}
			
			
//map_data_buffer_on_pixie rearranges data in PIXIE layout taking in account the "snake" readout architecture
int map_data_buffer_on_pixie(unsigned short *buffer_a, SENSOR Sens){
		unsigned short* temp_col;
		unsigned short col_cntr,row_cntr;
		int PIXIE_ROWS;
		/******************************/
		PIXIE_ROWS=Sens.rows;
		/******************************/
			temp_col=(unsigned short*)calloc(PIXIE_ROWS, sizeof(unsigned short));
			if (temp_col==NULL) {
				printf("DATA mapping:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
				return(-1);}
			if(Sens.Asic!=PII){
				if(verbose>=VERBOSITY_LOW)
				printf("Asked To map data on PII but Asic type is not PII\n");
			}
			//sectors 0,2,4,6,8,10,12,14 start with the first column(800 pix) in the right dir
			//sectors 1,3,5,7,8,11,13,15 start with the first column(800 pix) in the reversed dir
			//in general even index columns has the right dir and odd ones are reversed
			for(col_cntr=0;col_cntr<Sens.cols;col_cntr++){
				if ((col_cntr%2)){//only odd index columns are reversed
					for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
						temp_col[Sens.rows-row_cntr-1]=buffer_a[col_cntr*Sens.rows+row_cntr];}
					for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
						buffer_a[col_cntr*Sens.rows+row_cntr]=temp_col[row_cntr];}
					}
				}


			free(temp_col);
			return(1);}



int map_data_buffer_on_pixieIII(unsigned short *buffer_a,SENSOR Sens){
		unsigned short* temp_sector;
		unsigned short col_cntr,row_cntr,sector;
		int PIXIE_ROWS,COLS_PER_DOUT;
		/***************************************/
		PIXIE_ROWS=Sens.rows;
		COLS_PER_DOUT=Sens.cols_per_dout;
		/***************************************/
			temp_sector=(unsigned short*)calloc(PIXIE_ROWS*COLS_PER_DOUT, sizeof(unsigned short));
			if (temp_sector==NULL) {
				printf("DATA mapping:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
				return(0);}
			if(Sens.Asic!=PIII){
				printf("Asked to map data on PIII but Asic is not PIII\n");
				return(-1);
			}
			//sectors 0,2,4,6,8,10,12,14 start with the first column(800 pix) in the right dir
			//sectors 1,3,5,7,8,11,13,15 start with the first column(800 pix) in the reversed dir
			//in general even index columns has the right dir and odd ones are reversed
			for(sector=0;sector<Sens.dout;sector++){
				for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
					for(col_cntr=0;col_cntr<Sens.cols_per_dout;col_cntr++){
						temp_sector[row_cntr+(Sens.cols_per_dout-1-col_cntr)*Sens.rows]=buffer_a[row_cntr*Sens.cols_per_dout+col_cntr+sector*Sens.rows*Sens.cols_per_dout];
						}
					}
				memcpy(buffer_a+Sens.rows*Sens.cols_per_dout*sector,temp_sector,Sens.cols_per_dout*Sens.rows*sizeof(unsigned short));
			}


			free(temp_sector);
			return(1);}






void decode_pixie_data_buffer(unsigned short* table, int table_depth, unsigned short* databuffer,int databuffer_len){
	int i;
	for(i=0;i<databuffer_len;i++)
		databuffer[i]=table[databuffer[i]%table_depth];
}








void my_bytes_swap(unsigned short* us_ptr){
	char a,b,*temp_char_ptr;
	temp_char_ptr=(char*)us_ptr;
	a=*temp_char_ptr;
	b=*(temp_char_ptr+1);
	*(temp_char_ptr+1)=a;
	*(temp_char_ptr)=b;
}


int convert_bit_stream_to_counts(int code_depth,unsigned short* source_memory_offset,
								unsigned short* destination_memory_offset,SENSOR Sens,int verbose){
	int i,j;
	unsigned short dout_masks[Sens.dout],counter_masks[code_depth]
	               ,dout_mask_seed,cnt_mask_seed;
	if(Sens.Asic==PIII){
		dout_mask_seed=0x0001;
		cnt_mask_seed=(0x0001 << code_depth-1);//Be aware counter are 15 bits wide
		for(i=0;i<Sens.dout;i++) dout_masks[i]=(dout_mask_seed<<i);
		for(i=0;i<code_depth;i++) counter_masks[i]=(cnt_mask_seed>>i);
		}
	if(Sens.Asic==PII){
		dout_mask_seed=0x0001;
		cnt_mask_seed=(0x0001 << code_depth-1);
		for(i=0;i<Sens.dout;i++) dout_masks[i]=(dout_mask_seed<<i);
		for(i=0;i<code_depth;i++) counter_masks[i]=(cnt_mask_seed>>i);
		}
	for(j=0;j<Sens.dout;j++){
			destination_memory_offset[j]=0;
		 for(i=0;i<code_depth;i++){
			if(source_memory_offset[i] & dout_masks[j]){
				destination_memory_offset[j]|= counter_masks[i];
				//printf("bit[%d] found @ %d applying mask %x(set)\n",i,source_memory_offset[i] & dout_masks[j],counter_masks[i]);
				}
			else{
				destination_memory_offset[j]&= ~counter_masks[i];
				//printf("bit[%d] found @ %d applying mask %x(reset)\n",i,source_memory_offset[i] & dout_masks[j],counter_masks[i]);
				}
		}
	}
	if(verbose>=VERBOSITY_ULTRAHIGH){
		for(i=0;i<code_depth;i++)printf("\nsource_mem[%d]-->%4X",i,source_memory_offset[i]);
		Sleep(1000);
		for(i=0;i<Sens.dout;i++) printf("\ndest[%d]-->%4X",i,destination_memory_offset[i]);
	}
return(j);
}


void get_pixie_raw_data(unsigned short*source_buff_ptr,unsigned short*dest_buff_ptr,ACQ_PROP Acq_sett,SENSOR Sens){
	unsigned short* temp_us_ptr=source_buff_ptr;
	unsigned short* local_buffer_ptr=dest_buff_ptr;
	int i,j,k;

	int pixieii_modules = Sens.modules;
	int cols_per_dout=Sens.cols_per_dout;
	int douts=Sens.dout;
	int pixie_rows=Sens.rows;
	//unsigned short *conv=Sens.conv_table.ptr;
	int decode=Acq_sett.decode;
	int matrix_dim_words=Sens.cols * Sens.rows;
	/****************************************************************************************/

	/****************************************************************************************/
	int code_depth;
	if(Acq_sett.is_autocal)
		code_depth=Sens.autocal_bit_cnt;
	else
		code_depth=Sens.bit_per_cnt_std;
	/****************************************************************************************/
//	if(verbose>=VERBOSITY_VERYHIGH){
//	for(i=0;i<pixieii_modules;i++)
//	 for(j=0;j<cols_per_dout*pixie_rows;j++)
//		for(k=0;k<code_depth;k++)
//			printf("Net Buffer Module[%d]- Counter[%d]- Bit[%d]--->%4X\n",i,j,k,temp_us_ptr[i+(j*pixieii_modules*code_depth)+(k*pixieii_modules)]);
//		}
	if(verbose>=VERBOSITY_HIGH)printf("local_buffer filling code_depth=%d\n",code_depth);
	for(i=0;i<pixieii_modules;i++)
		for(j=0;j<cols_per_dout*pixie_rows;j++)
			for(k=0;k<code_depth;k++){
				my_bytes_swap(temp_us_ptr+i+(j*pixieii_modules*code_depth)+(k*pixieii_modules));
				local_buffer_ptr[(i*cols_per_dout*pixie_rows*code_depth)+(j*code_depth)+k]=temp_us_ptr[i+(j*pixieii_modules*code_depth)+(k*pixieii_modules)];
			}
//	if(verbose>=VERBOSITY_VERYHIGH){
//		for(i=0;i<pixieii_modules;i++)
//		{
//			for(j=0;j<10;j++)
//				for(k=0;k<code_depth;k++)
//					printf("Module[%d]- Counter[%d]- Bit[%d]--->%4X\n",i,j,k,local_buffer_ptr[(i*cols_per_dout*pixie_rows*code_depth)+(j*code_depth)+k]);
//			//Sleep(200);
//		}
//	}
	//printf("local_buffer has been filled with original data and data modules are grouped\n");
	//adesso il buffer(local_buffer_ptr) ha separati i dati modulo per modulo
	//user� local_buff_ptr come buffer di origine e process_buf come buffer di destinazione dei dati converiti.
	//memcpy(buff+BUFFER_TAG_BYTES,process_buf_ptr+BUFFER_TAG_BYTES,pixieii_modules*MATRIX_DIM_WORDS*code_depth);
	if(verbose>=VERBOSITY_HIGH)
		printf("PROC:Going to convert bit stream to counts\n");
		for(i=0;i<pixieii_modules;i++){
			for(j=0;j<cols_per_dout*pixie_rows;j++)
				convert_bit_stream_to_counts(code_depth,
						local_buffer_ptr+(i*cols_per_dout*pixie_rows*code_depth)+(j*code_depth),
						temp_us_ptr+(i*matrix_dim_words)+(j*douts),Sens,verbose);

			}

	//printf("data parsed\n");
	//a questo punto il process_buffer contiene le letture dei contatori (pseudo random)locazione per locazione


		for(i=0;i<pixieii_modules;i++){
			if (verbose>=VERBOSITY_HIGH) printf("processing module %d data\n",i);
		if(decode && !Acq_sett.is_autocal){
			if(verbose>=VERBOSITY_HIGH) printf("PROC:Going to decode\n");
			decode_pixie_data_buffer(	Sens.conv_table.ptr,
										Sens.conv_table.depth,
										temp_us_ptr+i*matrix_dim_words,
										Sens.matrix_size_pxls
										);
			}


		if(verbose>=VERBOSITY_HIGH)
						printf("PROC:Going to sort\n");
		databuffer_sorting(temp_us_ptr+i*matrix_dim_words,Sens);


		if(verbose>=VERBOSITY_HIGH)
						printf("PROC:Going to Map\n");
		if (Sens.Asic==PII)
		map_data_buffer_on_pixie(temp_us_ptr+i*matrix_dim_words,Sens);
		else
		if(Sens.Asic==PIII)
		map_data_buffer_on_pixieIII(temp_us_ptr+i*matrix_dim_words,Sens);
		}
}

/***************************************************************************************/
int InvertPIIIConversionTable(unsigned short *table_ptr,int table_size, int code_dept){
	unsigned short *us_ptr;
	int i,overranges=0;
	us_ptr=(unsigned short*)malloc(table_size*sizeof(unsigned short));
	if(us_ptr==NULL) return-1;
	if(table_ptr==NULL)return -2;
	memcpy(us_ptr,table_ptr,table_size*sizeof(unsigned short));
	for(i=0;i<table_size;i++){
		table_ptr[us_ptr[i]%table_size]=i;
		if(us_ptr[i]/table_size)overranges++;
	}
	table_ptr[0]=0;//please be aware that Francois table assigns 0 to both 0 and 32767
			   //this line corrects for the ambiguity after inversion that would
			   //assign 32767 to 0 pulses...
	free(us_ptr);
	return(overranges);
}


int GeneratePIIIConversionTable(unsigned short *table_ptr,int table_size, int code_dept){
	int p2[16];
	int tf,i,t,b,bit0,bit1,Nbits;
	int r[4]={1,0,0,1};
	Nbits=code_dept;
	if(Nbits==15){
		bit0=PIII_PRANDOM_15BITS_B0;
		bit1=PIII_PRANDOM_15BITS_B1;
		}
	else{
		bit0=PIII_PRANDOM_07BITS_B0;
		bit1=PIII_PRANDOM_07BITS_B1;
		}


	t=0;
	i=0;

	table_ptr[i]=(unsigned short)t;
	p2[0]=1;
	for(i=1;i<Nbits;i++){
		p2[i]=2*p2[i-1];

	}
	tf=p2[Nbits-1]*2-1;
	t=0;
	for(i=0;i<tf && i<table_size;i++){
		b=0;
		if((t&p2[bit0]) !=0)b=1;
		if((t&p2[bit1]) !=0)b+=2;
		t=t*2+r[b];
		t=t&tf;
		table_ptr[i+1]=(unsigned short)t;
		//fprintf(out,"%i %b %i\n",i+1,t,t);

	}
	if(i<table_size)
		return (1);
	else
		return (-1);
}


unsigned short * PIIIConversion_table_allocation(int code_depth){
	unsigned short *ush_ptr;
	int ret;
	ush_ptr=(unsigned short*)calloc(CONVERSIONTABLEDEPTH, sizeof(unsigned short));
	if (ush_ptr==NULL) {
				printf("\r\nPIII Conversion Table:Memory allocation unsuccesfull!! Please contact an expert :-)");
				return(NULL);}
	else{
		ret=GeneratePIIIConversionTable(ush_ptr,CONVERSIONTABLEDEPTH, code_depth);
		if(ret<0) printf("Memory Buffer is Too small for the requested code depth\n");
		//the following function reverses the table generated by brez
		ret=InvertPIIIConversionTable(ush_ptr,CONVERSIONTABLEDEPTH,code_depth);
		if(ret!=0) printf("Table inverted returned with errors (%d)\n",ret);
		return(ush_ptr);}
	}







#endif /* PIXIEII_DATA_UTILITIES_V2_CPP_ */

