#include "ap_storage_service.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"
#include "fs_driver.h"
#include "avi_encoder_app.h"
#include "stdio.h"


const INT8U  R_CopyrightMSG_Buf[]= "版权声明: 本产品由深圳市乐信兴业科技有限公司设计,版权所有,仿冒必究"; //
const INT8U   sensor_type_name[9][13]={"OV7670","OV9712","SOI_H22","BF3703","SOI_H22_MIPI","OV3640","OV5642",
	                                   "GC1004","GC1004_MIPI"
};
INT8U   init_buff[22]="2017-01-01 23:59:59 N";

static void save_COPYRIGHT_MESSAGE_to_disk(void)
{
    INT16S fd;
	INT32U addr;
	fd = open("C:\\CopyrightMSG.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(66+1);
	if(!addr)
	{
		close(fd);
		return;
	}
	gp_strcpy((INT8S*)addr, (INT8S *)R_CopyrightMSG_Buf);
	write(fd, addr, 66);
	close(fd);
	gp_free((void*) addr);
}
static void save_VERSION_NUMBER_to_disk(void)
{
    INT8U  *p;
	INT16S fd;
	INT32U addr;
	INT32U product_num;
	INT32U data_num;
	product_num = PRODUCT_NUM;
	data_num = PRODUCT_DATA;
	fd = open("C:\\Version.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(22+5);
	if(!addr)
	{
		close(fd);
		return;
	}
	p = (INT8U*)addr;
	sprintf((char *)p, (const char *)"%08d,JH%04d,v0.00,", data_num, product_num);
	p[17] = ((PROGRAM_VERSION_NUM%1000)/100) + '0';
	p[19] = ((PROGRAM_VERSION_NUM%100)/10) + '0';
	p[20] = ((PROGRAM_VERSION_NUM%10)/1) + '0';
	write(fd, addr, 22);
	close(fd);
	gp_free((void*) addr);
}
static void save_SENSOR_TYPE_to_disk(void)
{
	INT8U  *p;
	INT16S fd;
	INT32U addr;
	fd = open("C:\\sensor.txt", O_RDWR|O_TRUNC|O_CREAT);
	if(fd < 0)
		return;
	addr = (INT32U)gp_malloc(13+5);
	if(!addr)
	{
		close(fd);
		return;
	}

	gp_strcpy((INT8S*)addr, (INT8S *)sensor_type_name[USE_SENSOR_NAME]);

	write(fd, addr, strlen(sensor_type_name[USE_SENSOR_NAME]));
	close(fd);
	gp_free((void*) addr);
}
INT8U display_osd_flag=0;//默认bu显示OSD

static INT32S save_example_time_to_disk(void)
{
	INT16S fd;
	INT32U addr;
	
	//return STATUS_OK;
	
	#if 0
	fd = open("C:\\TAG.TXT", O_RDWR|O_TRUNC|O_CREAT);
	#else
	fd = open("C:\\time.txt", O_RDWR|O_TRUNC|O_CREAT);
	#endif
	if(fd < 0)
		return STATUS_FAIL;

	addr = (INT32U)gp_malloc(20+5);
	if(!addr)
	{
		close(fd);
		return STATUS_FAIL;
	}
		
	//gp_strcpy((INT8S*)video_info->AudSubFormat, (INT8S *)"adpcm");
	//gp_strcpy((INT8S*)addr, (INT8S *)"2017-01-01 23:59:59 ");
			if(display_osd_flag)
		      init_buff[20]='Y';
	        else
		      init_buff[20]='N';
	gp_strcpy((INT8S*)addr, (INT8S *)init_buff);
	write(fd, addr, 22);
	close(fd);
	gp_free((void*) addr);
	
	return STATUS_OK;
}
INT8U  record_led_flag=0;
INT32S JH_Message_Get(void)
{
#if 0
	INT32U addr;
	INT32S nRet;
	INT16S fd;

	fd = open("C:\\jh_message.txt", O_RDWR);
	
	if(fd >= 0)
		{
		   addr = (INT32U)gp_malloc(20+5);
		   if(!addr)
		   	goto Fail_Return_2;
		   nRet = read(fd, addr, 20);
		   if(nRet <= 0)
		   	goto Fail_Return;
		   
		}
	else
		return;
	
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"COPYFIGHT MESSAGE? ", 19);
	if (nRet==0) { save_COPYRIGHT_MESSAGE_to_disk(); goto Fail_Return; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"VERSION_NUMBER??", 16);
	if (nRet==0) { save_VERSION_NUMBER_to_disk(); goto Fail_Return; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"SENSOR?", 7);
	if (nRet==0) { save_SENSOR_TYPE_to_disk(); goto Fail_Return; }
Fail_Return:
	gp_free((void*) addr);
Fail_Return_2:
	if(fd>=0) close(fd);
#else
    INT8U  read_flag=0;
	INT8U  data;
	INT8U  *pdata;
	INT8U  *yndata;
	INT16S fd;
	INT16U wtemp;
	INT32U addr;
	INT32S nRet;
	TIME_T	time_set;
	
	#if 0
	fd = open("C:\\TAG.TXT", O_RDONLY);
	#else
	fd = open("C:\\time.txt", O_RDONLY);
	#endif

	if(fd < 0)
	{
		DBG_PRINT("OPEN time.txt FAIL!!!!!\r\n");
		goto Fail_Return_3;
	}

	addr = (INT32U)gp_malloc(20+5);
	if(!addr)
	{
		goto Fail_Return_2;
	}
	else	
	{
		nRet = read(fd, addr, 22);
		if(nRet <= 0) goto Fail_Return;
	}

	//gp_strcpy((INT8S*)addr, (INT8S *)"2015-01-01 00:00:00 ");
	//display_osd_flag=1;

	read_flag=1;
	
	yndata = (INT8U*)addr;
	data = *(yndata+20);
	if(data == 'Y')
	display_osd_flag=1;
	else
	display_osd_flag=0;
	
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"Motion Trig LED Ope", 19);
	if(nRet==0){record_led_flag=1;gp_free((void*) addr);if(fd>=0) close(fd);return STATUS_FAIL; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"COPYFIGHT MESSAGE? ", 19);
	if (nRet==0) { save_COPYRIGHT_MESSAGE_to_disk(); gp_free((void*) addr);if(fd>=0) close(fd);return STATUS_FAIL; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"VERSION_NUMBER??", 16);
	if (nRet==0) { save_VERSION_NUMBER_to_disk();gp_free((void*) addr); if(fd>=0) close(fd);return STATUS_FAIL; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"SENSOR?", 7);
	if (nRet==0) { save_SENSOR_TYPE_to_disk(); gp_free((void*) addr);if(fd>=0) close(fd);return STATUS_FAIL; }
	nRet = gp_strncmp((INT8S*)addr, (INT8S *)"2017-01-01 23:59:59 ", 19);	//返回0表示参数1和参数2的内容完全相同;
	if (nRet==0) goto Fail_Return;
	read_flag=0;

	pdata = (INT8U*)addr;
	//year
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1000;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*100;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if((wtemp > 2025) || (wtemp < 2015)) goto Fail_Return;
	time_set.tm_year = wtemp;
	
	//skip -		
	pdata++;	
	
	//month
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if(wtemp>12) goto Fail_Return;
	time_set.tm_mon = wtemp;
			
	//skip -		
	pdata++;
	
	//day
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if(wtemp>31) goto Fail_Return;
	time_set.tm_mday = wtemp;
	
	//skip space		
	pdata++;
	
	//hour
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if(wtemp>23) goto Fail_Return;
	time_set.tm_hour = wtemp;
			
	//skip :	
	pdata++;
			
	//minute
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if(wtemp>59) goto Fail_Return;
	time_set.tm_min = wtemp;
	
	//skip :	
	pdata++;
			
	//second
	wtemp = 0;
	data = *pdata++;
	data -= 0x30;
	wtemp += data*10;
	
	data = *pdata++;
	data -= 0x30;
	wtemp += data*1;
	if(wtemp>59) goto Fail_Return;
	time_set.tm_sec = wtemp;
	
	if(fd>=0) close(fd);
	gp_free((void*) addr);
	cal_time_set(time_set);
	ap_state_handling_calendar_init();
	cal_time_get(&time_set);

	save_example_time_to_disk();
	return STATUS_OK;
	
	Fail_Return:
		gp_free((void*) addr);
	Fail_Return_2:
		if(fd>=0) close(fd);
	Fail_Return_3:
	ap_state_handling_calendar_init();
	cal_time_get(&time_set);
	if( read_flag == 0)
	{
	 	save_example_time_to_disk();
	}
	return STATUS_FAIL;

#endif
}
void WriteTool_card(void)
{
#if 0
	INT16S fd;
	INT16U tool_fd;
	INT32U size;
	INT32U read_buf;
	INT32S nRet;
	INT32U tf_free_size;

 	fd =  open("C:\\SecretPen.exe", O_RDONLY); 
	if (fd < 0)
	{
		fd = open("C:\\SecretPen.exe", O_RDWR|O_TRUNC|O_CREAT);    //建立 TimeUpdater.exe 工具文件并打开
		if (fd < 0) 
			{
			// DBG_PRINT("Creat_file_fail\r\n");
			 return;    
		    }      //建立出错时返回
		else
		{ 
		//DBG_PRINT("Creat_file_ok\r\n");
			nvmemory_init();   //初始化SPI相关差数, 资源文件的位置
			tool_fd = nv_open((INT8U *)"SECRETPEN.EXE");    //格式化资源文件路径及索引
			if (tool_fd != 0xFFFF)
			{
			//DBG_PRINT("Find tool file\r\n");
				size = nv_rs_size_get(tool_fd);  //检测资源文件所需的空间大小 
				//DBG_PRINT("Tool size=%d\r\n",size);
				read_buf = (INT32S) gp_malloc(size);  //申请大小为 size 的动态内存容量用来存放数据
				if (!read_buf)
				{
				//DBG_PRINT("Err1\r\n");
					gp_free((void*)read_buf);
					return;
				}
				if (nv_read(tool_fd,(INT32U)read_buf,size))  //从 tool_fd 开始读取大小为 size 的资源文件到 read_buf
				{
				//DBG_PRINT("Err2\r\n");
					gp_free((void*)read_buf);                //释放内存空间
					return;
				}
				//DBG_PRINT("write data to card\r\n");
				write(fd,read_buf,size);                     //将大小为 size 的 read_buf 内的数据写到TF卡内, 文件名参数为 fd
				gp_free((void*)read_buf);                    //释放内存空间
			}
		}
		close(fd);  //关闭文件
	}
	#endif
}

