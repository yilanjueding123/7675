#include "drv_l1_i2c.h"
#include "drv_l2_sensor.h"
#include "drv_l1_cdsp.h"
#include "gp_aeawb.h"

#include "LDWs.h"

#include "sensor_soi_h42.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (USE_SENSOR_NAME == SENSOR_SOI_H42)
//================================================================//
static sensor_exposure_t jxh42_seInfo;
static sensor_calibration_t h42_cdsp_calibration;
static int *p_expTime_table;

extern INT8U LDWS_Enable_Flag;
extern void sensor_set_fps(INT32U fpsValue);
extern INT32U sensor_get_fps(void);

static gpCdspWBGain_t soi_h42_wbgain;

INT8U	black_sun_flag = 0;

static INT8U h42_id;
/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
sensor_exposure_t *jxh42_get_senInfo(void)
{
	return &jxh42_seInfo;
}

void sensor_calibration_str(void)
{
	//sensor_calibration_t h42_cdsp_calibration;

	//H42_cdsp_calibration.lenscmp = g_H42_lenscmp_table;	//GPCV1248 not support
	h42_cdsp_calibration.r_b_gain = g_H42_r_b_gain;
/*
	h42_cdsp_calibration.gamma = g_H42_gamma_045_table;
	h42_cdsp_calibration.color_matrix = g_H42_color_matrix4gamma045;
	h42_cdsp_calibration.awb_thr = g_H42_awb_thr;
*/
}

/* For test
void jxh42_set_gamma_table(int is_night)
{
	if(is_night == 0) hwCdsp_InitGamma((INT32U *)g_H42_gamma_045_table);
	else  hwCdsp_InitGamma((INT32U *)g_H42_gamma_night_table);
}*/

sensor_calibration_t *jxh42_get_calibration(void)
{
	return &h42_cdsp_calibration;
}
void SOi_h42_sensor_calibration(void)
{
	//OB
	gp_Cdsp_SetBadPixOb((INT16U *)g_h42_badpix_ob_table);
	//Gamma
	hwCdsp_InitGamma((INT32U *)g_H42_gamma_045_table);
	//Color Correction
	hwCdsp_SetColorMatrix_Str((INT16S *)g_H42_color_matrix4gamma045);
	//AWB
	gp_Cdsp_SetAWBYUV((INT16S *)g_H42_awb_thr);
	
	//Lenscmp
	/*
	hwIsp_luc_MaxTan8_Slop_CLP((INT16U *)g_H42_MaxTan8 ,(INT16U *)g_H42_Slope4 ,(INT16U *)g_H42_CLPoint);
	hwIsp_RadusF0((INT16U *)g_H42_Radius_File_0);
	hwIsp_RadusF1((INT16U *)g_H42_Radius_File_1);
	*/
	//R,G,B Linearity correction
	//hwIsp_InitLiCor((INT8U *)LiTable_rgb);		
}

gpCdspWBGain_t *soi_h42_awb_r_b_gain_boundary(void)
{
	
	soi_h42_wbgain.max_rgain = g_H42_r_b_gain[49][0];
	soi_h42_wbgain.max_bgain = g_H42_r_b_gain[11][1];
	soi_h42_wbgain.min_rgain = g_H42_r_b_gain[11][0];
	soi_h42_wbgain.min_bgain = g_H42_r_b_gain[49][1];
	
	return &soi_h42_wbgain;
}

/**************************************************************************
 *             F U N C T I O N    I M P L E M E N T A T I O N S           *
 **************************************************************************/

static int h42_cvt_agc_gain(int agc_gain)
{
	int h42_agc_gain, i;
	int pow2;

	pow2 = 0;        
    i = 5; 
    do { 
            if(agc_gain <= 0x1f) 
                    break; 
            
            agc_gain >>= 1; 
            pow2++;
            
            i--; 
    } while(i != 0); 

    agc_gain -= 0x10; 
    if(agc_gain < 0) agc_gain = 0; 
    h42_agc_gain = (pow2 << 4) + agc_gain; 
    
    return h42_agc_gain; 
}


int H42_get_night_ev_idx(void)
{
	return jxh42_seInfo.night_ev_idx;
}

int H42_get_max_ev_idx(void)
{
	return jxh42_seInfo.max_ev_idx;
}


INT8U pid_check(void)
{
	INT8U pidh, pidl;
	INT8U i, h42_id_tmp;//, rel_h42_id;
	INT8U h42_slave_id = 0xFF;

		//rel_h42_id = 0xFF;
		
		for (i=0; i<4 ; i++)
		{
			h42_id_tmp = 0x60 + i*4;
			
			pidh = sccb_read(h42_id_tmp, 0x0a);
			pidl = sccb_read(h42_id_tmp, 0x0b);
			//DBG_PRINT("h:%d,L:%d",pidh,pidl);
			if ((pidh == 0xA0) && (pidl == 0x42))
			{		
				h42_slave_id = h42_id_tmp;
				//h42_slave_id = rel_h42_id = h42_id_tmp;
				DBG_PRINT("H42 ID = 0x%x\r\n",h42_slave_id);
			}
		}

		return h42_slave_id;	
}
#if 1
INT8U ver_check(INT8U id)
{
	INT8U h42_version;
	
	h42_version = sccb_read(id, 0x09);
	
	DBG_PRINT("\r\nH42_version = 0x%x\r\n", h42_version);
	
	return h42_version;
}

void reg_chang1(INT8U id)
{
	sccb_write(id, 0x27, 0x49);
	sccb_write(id, 0x2C, 0x00);
	sccb_write(id, 0x63, 0x19);		
}

void reg_chang2(INT8U id)
{
	sccb_write(id, 0x27, 0x3B);
	sccb_write(id, 0x2C, 0x04);
	sccb_write(id, 0x63, 0x59);		
}
#endif

int H42_set_exposure_time(sensor_exposure_t *si)
{
	//int ret=0;
	//unsigned short tmp;
	//int analog_gain, digital_gain;
	int lsb_time, msb_time;
	int idx;
	//unsigned char cvt_digital_gain;
		
#if 1	// From agoritham calc new data update to jxh42_seInfo.
	jxh42_seInfo.sensor_ev_idx += si->ae_ev_idx;
	if(jxh42_seInfo.sensor_ev_idx >= jxh42_seInfo.max_ev_idx) jxh42_seInfo.sensor_ev_idx = jxh42_seInfo.max_ev_idx;
	if(jxh42_seInfo.sensor_ev_idx < 0) jxh42_seInfo.sensor_ev_idx = 0;
	
	idx = jxh42_seInfo.sensor_ev_idx * 3;
	jxh42_seInfo.time = p_expTime_table[idx];
	jxh42_seInfo.analog_gain = p_expTime_table[idx+1];
	jxh42_seInfo.digital_gain = p_expTime_table[idx+2];
	
	jxh42_seInfo.userISO = si->userISO;
#endif	
	//DBG_PRINT("Time = %d, a gain = %d, d gain = %d, ev idx = %d\r\n", jxh42_seInfo.time, jxh42_seInfo.analog_gain, jxh42_seInfo.digital_gain, jxh42_seInfo.sensor_ev_idx);

	if (jxh42_seInfo.sensor_ev_idx <= BLACK_SUN_IDX)
	{
		if (black_sun_flag == 0)
		{	
			sccb_write(h42_id, 0x0C , 0x00);		//enable black sun
			black_sun_flag = 1;	
			//DBG_PRINT("s");
		}
	}
	else if (jxh42_seInfo.sensor_ev_idx > BLACK_SUN_IDX)
	{
		if (black_sun_flag == 1)
		{	
			sccb_write(h42_id, 0x0C , 0x40);		//disble black sun
			black_sun_flag = 0;	
			//DBG_PRINT("y");
		}	
	}
	
/*
	// set exposure time
	if (( 0x2EE < jxh42_seInfo.time)&&(jxh42_seInfo.time <= 0x384))
	//if (( 0x354 < jxh42_seInfo.time)&&(jxh42_seInfo.time <= 0x400))
	{
		if(sensor_get_fps() != 25)
		{
			sensor_set_fps(25);	
			//DBG_PRINT("f25");		
		}
	}
	else if (jxh42_seInfo.time > 0x384)
	//else if (jxh42_seInfo.time > 0x400)
	{
		if(sensor_get_fps() != 20)
		{
			sensor_set_fps(20);
			//DBG_PRINT("f20");
		}
	}
	else
	{
		if(sensor_get_fps() != 30)
		{
			sensor_set_fps(30);
			//DBG_PRINT("f30");
		}
	}
*/

	lsb_time = (jxh42_seInfo.time & 0xFF);
	msb_time = ((jxh42_seInfo.time >>8 )& 0xFF);
	sccb_write(h42_id, 0x01 , lsb_time );
	sccb_write(h42_id, 0x02 , msb_time );

	//DBG_PRINT("\r\n<H42_set_exposure_time> time = %d, Lsb_time: 0x%x, Msb_time: 0x%x\r\n",jxh42_seInfo.time,lsb_time,msb_time);
	
	return 0;
}


void H42_set_exposure_gain(void)
{
	int analog_gain, digital_gain;
	
	//analog_gain = H42_cvt_analog_gain(jxh42_seInfo.analog_gain);
	analog_gain = h42_cvt_agc_gain((jxh42_seInfo.analog_gain >> 4));
	
	
	//DBG_PRINT("analog_gain = 0x%x\r\n", analog_gain);
	sccb_write(h42_id, 0x00, analog_gain );
	
	digital_gain =jxh42_seInfo.digital_gain >> 3;
	hwCdsp_SetGlobalGain(digital_gain);	
}


void H42_get_exposure_time(sensor_exposure_t *se)
{
	gp_memcpy((INT8S *)se, (INT8S *)&jxh42_seInfo, sizeof(sensor_exposure_t));
}


void H42_set_exp_freq(int freq)
{

	if(freq == 50)
	{
			jxh42_seInfo.sensor_ev_idx = H42_50HZ_INIT_EV_IDX;
			jxh42_seInfo.ae_ev_idx = 0;
			jxh42_seInfo.daylight_ev_idx= H42_50HZ_DAY_EV_IDX;
			jxh42_seInfo.night_ev_idx= H42_50HZ_NIGHT_EV_IDX;			
			jxh42_seInfo.max_ev_idx = H42_50HZ_MAX_EXP_IDX - 1;
			p_expTime_table = (int *)g_h42_exp_time_gain_50Hz;
	}
	else if(freq == 60)

	{
		jxh42_seInfo.sensor_ev_idx = H42_60HZ_INIT_EV_IDX;
		jxh42_seInfo.ae_ev_idx = 0;
		jxh42_seInfo.daylight_ev_idx= H42_60HZ_DAY_EV_IDX;
		jxh42_seInfo.night_ev_idx= H42_60HZ_NIGHT_EV_IDX;
		jxh42_seInfo.max_ev_idx = H42_60HZ_MAX_EXP_IDX - 1;
		p_expTime_table = (int *)g_h42_exp_time_gain_60Hz;
	}
}

static int H42_init(void)
{
	jxh42_seInfo.max_time = H42_MAX_EXPOSURE_TIME;
	jxh42_seInfo.min_time = H42_MIN_EXPOSURE_TIME;

	jxh42_seInfo.max_digital_gain = H42_MAX_DIGITAL_GAIN ;
	jxh42_seInfo.min_digital_gain = H42_MIN_DIGITAL_GAIN ;

	jxh42_seInfo.max_analog_gain = H42_MAX_ANALOG_GAIN;
	jxh42_seInfo.min_analog_gain = H42_MIN_ANALOG_GAIN;

	jxh42_seInfo.analog_gain = jxh42_seInfo.min_analog_gain;
	jxh42_seInfo.digital_gain = jxh42_seInfo.min_digital_gain;
	jxh42_seInfo.time = jxh42_seInfo.max_time;// >> 1;
	
	H42_set_exp_freq(50);
	
	DBG_PRINT("H42_init\r\n");
	return 0;
}


void sensor_SOi_h42_init(INT32U WIDTH, INT32U HEIGHT)
{
	INT32U i;
	
#ifdef RESET_PIN_IO
	__msg("sensor_SOi_h42_init, reset\n");
	gpio_init_io(SCCB_RESET, GPIO_OUTPUT);
	gpio_set_port_attribute(SCCB_RESET, ATTRIBUTE_HIGH);
	gpio_write_io(SCCB_RESET, DATA_LOW);
	
	drv_msec_wait(20);
	gpio_write_io(SCCB_RESET, DATA_HIGH);
	drv_msec_wait(20);
#endif

	H42_init();
	sensor_calibration_str();
	
	h42_id = pid_check();
	if (h42_id == 0xFF){
		DBG_PRINT("Unknow H42 ID\r\n");
	}
	
	if(sensor_format == SOI_H42_RAW){
		if(WIDTH == 1280 && HEIGHT == 720)
		{
			for (i=0; i<sizeof(JXH42_1280x720x30_DVP_10b)/2; i++) 
			{
			//	DBG_PRINT("0x%02x, 0x%02x\r\n", JXH42_1280x720x30_DVP_10b[i][0],  JXH42_1280x720x30_DVP_10b[i][1]);
				sccb_write(h42_id, JXH42_1280x720x30_DVP_10b[i][0], JXH42_1280x720x30_DVP_10b[i][1]);	
			}
				if((ver_check(h42_id) == 0)||(ver_check(h42_id) == 0x80))	//SOI suggetion do it.
				{
					reg_chang1(h42_id);
				}
				else if (ver_check(h42_id) == 0x81)
				{
					reg_chang2(h42_id);
				}
		}			
		else 
		{
			while(1);
		}
	}else if(sensor_format == SOI_H42_MIPI){
		if(WIDTH == 1280 && HEIGHT == 800)
		{
			for (i=0; i<sizeof(H42_MIPI_1280_800_30)/2; i++) 
			{
				sccb_write(h42_id, H42_MIPI_1280_800_30[i][0], H42_MIPI_1280_800_30[i][1]);
			}
		}	
		else if	(WIDTH == 1280 && HEIGHT == 720)
		{
			for (i=0; i<sizeof(JXH42_1280x720x30_Mipi_1L_10b)/2; i++) 
			{
				//DBG_PRINT("0x%02x, 0x%02x\r\n", JXH42_1280x720x30_Mipi_1L_10b[i][0],  JXH42_1280x720x30_Mipi_1L_10b[i][1]);
				sccb_write(h42_id, JXH42_1280x720x30_Mipi_1L_10b[i][0], JXH42_1280x720x30_Mipi_1L_10b[i][1]);
			}
			
			if((ver_check(h42_id) == 0)||(ver_check(h42_id) == 0x80))	//SOI suggetion do it.
			{
				reg_chang1(h42_id);
			}
			else if (ver_check(h42_id) == 0x81)
			{
				reg_chang2(h42_id);
			}
		}	
		else if(WIDTH == 640 && HEIGHT == 480)
		{
			for (i=0; i<sizeof(SOI_H42_MIPI_VGA_f60)/2; i++) 
			{
				///DBG_PRINT("0x%02x, 0x%02x\r\n", SOI_H42_MIPI_VGA_f60[i][0],  SOI_H42_MIPI_VGA_f60[i][1]);
				sccb_write(h42_id, SOI_H42_MIPI_VGA_f60[i][0], SOI_H42_MIPI_VGA_f60[i][1]);
			}			
		}		
		else 
		{
			while(1);
		}
	}
}

INT8U get_h42_slave_id(void)
{
	return h42_id;	
}

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(USE_SENSOR_NAME == SENSOR_SOI_H42)     //
//================================================================//
