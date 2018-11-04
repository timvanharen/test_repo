/* 
MLX90640.h header file for the driver for the MLX90640 sensor 

Author:  	Tim van Haren (timvanharen@live.com)
Date:		7 March, 2018
*/

#ifndef MLX90640_h
#define MLX90640_h

#include "Arduino.h"

/* Defines */
// MLX register code
#define MLX_CONTROL_REG_M       	0x80
#define MLX_CONTROL_REG_L       	0x0D
#define MLX_STATUS_REG_M        	0x80
#define MLX_STATUS_REG_L        	0x00
#define MLX_RAM_ADDRESS_M          	0x04
#define MLX_RAM_ADDRESS_L          	0x00
#define MLX_EEPROM_M       			0x24
#define MLX_EEPROM_L       			0x00
#define MLX_EEPROM_SIZE				1664

// Configuration of bytes for the control register
#define MLX_CONTROL_CMD_REG_M   	0x0A
#define MLX_CONTROL_CMD_REG_L   	0x81

// MLX90640  sensor parameters
#define MLX_90640_I2C_ADDRESS       0x33 
#define MLX_90640_COL           	32  // Amount of pixels per column
#define MLX_90640_ROW           	24  // Amount of pixels per row
#define MLX_90640_FULL_ARRAY    	768
#define MLX_SENSOR_INPUT_LEN		1664

// When defined debug or calibration data is printed
//#define DISPLAY_ALL_READ_DATA				1
//#define DISPLAY_SENSOR_DEBUG				1
//#define DISPLAY_HOTSPOT_DEBUG				1	
//#define DISPLAY_RESTORE_DEBUG				1
//#define DISPLAY_CALIB_DEBUG				1
//#define DISPLAY_MEM_DEBUG					1
//#define DISPLAY_OFFSET_PER_PIXEL			1
//#define DISPLAY_SENSITIVITY_PER_PIXEL		1
//#define DISPLAY_KTA_PER_PIXEL				1

#define TRUE                    			1
#define FALSE                   			0

class MLX90640
{
  public:
	//public functions for user
    MLX90640(int pin, int RR, int mode, int res, int emi);
	void  wakeupSensor(bool on);
    void  initialize();
	void  capture(bool calibrationMode);
	int   getTempMatrix(int x, int y);
	int	  getMaxTemp();
	int	  getMinTemp();
	
  private:
  
	// These functions are used for communication between the controller and the sensor
	void	configSensor();
	void  	startMeasurement();
	bool  	measurementDone();
	void  	readSubPage(bool page);
	void  	getSensorData();
	
	//these functions are used to access memory
	void  	getEEPROMData();
	int		getResFromRAM();
	
	// These functions are used to calibrate the sensor data
	int   	getOddEven(int row, int col);
	void  	getPtat();
	void 	correctRes();
    void  	calculateVdd();
    void 	calculateTa();
    void 	calculateKgain();
	void 	IRdatacompensationInterleaved(bool page);
	void	IRdatacompensationChess(bool page);
	void	IRdatacompensationSimple(bool page);
	//float 	IRdatacompensation(int x, int y, int z);
	
	// These functions are called once on boot and are used to ensure reliable data conversion
    void  	restoreVDD();
    void 	restorePTAT();
    void  	restoreGAIN();    
    void  	restoreTGC();
	void  	restoreOFFSET();
    void  	restoreKV();
    void  	restoreKTA();
    void  	restoreKSTA();
    void  	restoreKSTO();
    void  	restoreALPHACP();  
    void  	restoreOFFSETCP();  
    void  	restoreKVCP();
    void  	restoreKTACP();
	void  	restoreSENSITIVITY();
	void 	restoreRESOLUTION();
    
	// private variables for sensor configuration
	int	  	MLXSupplyPin;
	float	sampleFreq;
    int	  	sampleTime;
    int  	measurementMode;
    int   	ADCResolution;
	int 	emissivity;
	int		maxTemp;
	int 	minTemp;
	
	// Variables used to store calibration data recalculated from eeprom storage
	double  Pix_os_ref_range_1[MLX_90640_ROW][MLX_90640_COL];
	double  Alpha[MLX_90640_ROW][MLX_90640_COL];
	double  Kta_range_1[MLX_90640_ROW][MLX_90640_COL];
	short   OUTLIERS [16][3];
	
	float   Kv_odd_odd, Kv_even_odd, Kv_odd_even, Kv_even_even;
	float   ALPHA_CP_subpage_0, ALPHA_CP_subpage_1;
	float   OFFSET_CP_subpage_0,OFFSET_CP_subpage_1;
	float   pix_gain_cp_sp0, pix_gain_cp_sp1;
	float   KVcp;
	float   Kta_cp;
	float   TGC;
	float   VPTAT;
	float   VBE;
	float   DELTA_V;
	int 	RES_EE;
	float	RES_CORR;
	float   Vdd_25;
	float   Kv_Vdd;
	float	Kv;
	float   K_gain;
	int     outliers_present; 

	// Amplifier gain 
	float   GainMeas_25_3v2;

	// Alpha values
	int 	Alpha_scale;
	float 	Alpha_ref;
	int 	ACC_scale_row;
	int 	ACC_scale_col;
	int 	ACC_scale_remnant;
	
	// Ambient Temperature 
	int   	VPTAT_25;  
	float	DELTA_Volt;
	float	VPTAT_art;
	float   Kv_PTAT;
	float   Kt_PTAT;
	float   ALPHA_PTAT;

	// Object Temp 
	float   KsTa;
	float   KsTo;
	float	Ct;
	float   emmisivity = 1.0;
	float   g_Vdd;
	float   Ta; 

	// Variables used to store sensor memory, data and to iterate through loops or subpages
	int     i,j,k;
	int     EEPROMData[1664];
	int   	temperatureMatrix[MLX_90640_ROW][MLX_90640_COL];
	int     dataArray[MLX_SENSOR_INPUT_LEN/2];
	int		maxMeasuredTemp;
	int		minMeasuredTemp;
};

#endif