/* 
MLX90640.h header file for the driver for the MLX90640 sensor 

Author:  	Tim van Haren (timvanharen@live.com)
Date:		7 March, 2018
*/

#include "Arduino.h"
#include "MLX90640.h"
#include "Wire.h"

/* Public functions */

// MLX90640 sensor class
MLX90640::MLX90640(int pin, int RR, int mode, int res, int emi) {
	MLXSupplyPin = pin;
    sampleFreq = RR;
	sampleTime = 1000/sampleFreq; // in msec
    measurementMode = mode;
    ADCResolution = res;
	emissivity = emi;
	
	// Define the pin which switches the power to the sensor.
	pinMode(MLXSupplyPin, OUTPUT);
}

// MLX90640 power switch function
void MLX90640::wakeupSensor(bool on)  {
    if(on==TRUE) {
      digitalWrite(MLXSupplyPin, HIGH); //switch 3.3V voltage regulator on 
      Serial.println("MLX sensor has been turned on");
    } else {
      digitalWrite(MLXSupplyPin, LOW); //switch 3.3V voltage regulator off 
      Serial.println("MLX sensor has been turned off");
    }
}

// Sensor initialize function
void MLX90640::initialize() {
	
    getEEPROMData(); 
	Serial.println("EEPROM read");
	
	Serial.println("Sensor ID : " 
	+ String(((int)EEPROMData[0x0e])*256+(int)EEPROMData[0x0f], HEX) + " "
	+ String(((int)EEPROMData[0x10])*256+(int)EEPROMData[0x11], HEX) + " "
	+ String(((int)EEPROMData[0x12])*256+(int)EEPROMData[0x13], HEX));
	
	Serial.println("IC addr : " + String(((int)EEPROMData[0x1e])*256+(int)EEPROMData[0x1f], HEX));

	Serial.println("Control register at startup HEX: 0x" 
	+ String(((int)EEPROMData[0x18])*256+(int)EEPROMData[0x19], HEX) + " BIN: "
	+ String(((int)EEPROMData[0x18])*256+(int)EEPROMData[0x19], BIN));
	
	// Clear temperatureMatrix array
	for (i=0;i<MLX_90640_ROW;i++) {        
		for (j=0;j<MLX_90640_COL;j++) {
			temperatureMatrix[i][j] = 0;
		}
	}
	
	// Configurate sensor, according to values defined by the user
    configSensor();
	Serial.println("Sensor configured");
	
	//first read of the sensor is always invalid, but it must be done to ensure a valid reading
	readSubPage(0);
	readSubPage(1);
	Serial.println("Sensor has been cleared");
	
	// Restore calibration values
	restoreVDD();
	restorePTAT();
	restoreGAIN();
	restoreKSTA();
	restoreKSTO();
	restoreALPHACP();  
	restoreOFFSET();	
	restoreOFFSETCP();  
	restoreTGC();
	restoreKV();
	restoreKTA();
	restoreKVCP();
	restoreKTACP();
	restoreSENSITIVITY();
	restoreRESOLUTION();
	
	// Calibrate sensor data
	calculateVdd();
	calculateTa(); 
	calculateKgain();
} 

// This function gets the data from the sensor and processes the data to usable and calibrated temperature values
void MLX90640::capture(bool calibrationMode) {
	
	// set maxTemp and minTemp to the minimum and maximum temperature measurement possible.
	maxTemp = -32768;
	minTemp = 32768;
	
	readSubPage(0);
	
	#ifdef DISPLAY_SENSOR_DEBUG
		Serial.println("Before pixel correction page 0 ");
		for (i=0;i<MLX_90640_ROW;i++) {        
			for (j=0;j<MLX_90640_COL;j++) {
				Serial.print(String(temperatureMatrix[i][j]) + ", ");
			}
			Serial.println("");
		}
		Serial.println("");
	#endif
	
	if(calibrationMode) {
		if(measurementMode == 0)
			// Calibrate the sensor according to the interleaved pattern, not working.
			IRdatacompensationInterleaved(0);
		else
			// Calibrate the sensor according to the chess patte, not working.
			IRdatacompensationChess(0);
	} else {
		// Calibrate the sensor K_gain per pixel and offset the minimal temperature data to 0
		IRdatacompensationSimple(0);
	}
	
	#ifdef DISPLAY_SENSOR_DEBUG
		Serial.println("After pixel correction page 0 ");
		for (i=0;i<MLX_90640_ROW;i++) {        
			for (j=0;j<MLX_90640_COL;j++) {
				Serial.print(String(temperatureMatrix[i][j]) + ", ");
			}
			Serial.println("");
		}
		Serial.println("");
	#endif
	
	readSubPage(1);
	
	#ifdef DISPLAY_SENSOR_DEBUG
		Serial.println("Before pixel correction page 1 ");
		for (i=0;i<MLX_90640_ROW;i++) {        
			for (j=0;j<MLX_90640_COL;j++) {
				Serial.print(String(temperatureMatrix[i][j]) + ", ");
			}
			Serial.println("");
		}
		Serial.println("");
	#endif

	if(calibrationMode) {
		if(measurementMode == 0)
			IRdatacompensationInterleaved(1);
		else
			IRdatacompensationChess(1);
	} else {
		IRdatacompensationSimple(1);
	}
	
	#ifdef DISPLAY_SENSOR_DEBUG
		Serial.println("After pixel correction page 1");
		for (i=0;i<MLX_90640_ROW;i++) {        
			for (j=0;j<MLX_90640_COL;j++) {
				Serial.print(String(temperatureMatrix[i][j]) + ", ");
			}
			Serial.println("");
		}
		Serial.println("");
	#endif
	
}

// This function returns the stored temperature matrix
int MLX90640::getTempMatrix(int x, int y) {
    return temperatureMatrix[x][y];
}

// Returns the highest measured temperature
int MLX90640::getMaxTemp() {
	return maxTemp;
}

// Returns the highest measured temperature
int MLX90640::getMinTemp() {
	return minTemp;
}
/* Private functions */

// This function configurates the MLX90640 sensor, according to the user defined settings.
void MLX90640::configSensor() {
    int controlRegister[2];
	
	controlRegister[0] = 0x00;
	controlRegister[1] = 0x01;
	
	// Set the sample frequency
	for(i=0;i<8;i++)
		if(pow(2, i-1) == sampleFreq)
			sampleFreq = i;
	
	controlRegister[0] = controlRegister[0] + ((int)sampleFreq >> 1) & 0x00FF;
	
	controlRegister[1] = controlRegister[1] + ((int)sampleFreq << 7) & 0x00FF;
	
	// Set the sampling mode
	controlRegister[0] = controlRegister[0] + (measurementMode << 4);
	
	// Set the ADC resolution
    controlRegister[0] = controlRegister[0] + ((ADCResolution - 16) << 2);

	// Write to the control register
    Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
    Wire.write(MLX_CONTROL_REG_M);  
    Wire.write(MLX_CONTROL_REG_L);  
    Wire.write(controlRegister[0]);
    Wire.write(controlRegister[1]);
    Wire.endTransmission();
	
    Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
    Wire.write(MLX_CONTROL_REG_M);  
    Wire.write(MLX_CONTROL_REG_L);  
    Wire.endTransmission(false);
    
    Wire.requestFrom(MLX_90640_I2C_ADDRESS, 2);
    if(Wire.available() <= 2) {  
		for(i=0;i<2;i++) 
			controlRegister[i] = Wire.read(); 
    }
    
	#ifdef DISPLAY_MEM_DEBUG
		Serial.println("Controlregister after configuration: 0x" + String(controlRegister[0]*256 + controlRegister[1], HEX) + " BIN: " + String(controlRegister[0]*256 + controlRegister[1], BIN));
	#endif
}	

// This function initiates a measurement
void MLX90640::startMeasurement() {
	Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
	Wire.write(MLX_STATUS_REG_M);
	Wire.write(MLX_STATUS_REG_L);
	Wire.write(0x00);
	Wire.write(0x00);
	Wire.endTransmission();
}

// This function polls the sensor till it has completed its measurement.
bool MLX90640::measurementDone() {
    int timeout = 0;
	int delayStep = sampleTime/5;
	char statusRegister[2];
    
    while(1) {
        Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
        Wire.write(MLX_STATUS_REG_M);
        Wire.write(MLX_STATUS_REG_L);
        Wire.endTransmission(false);
        
        Wire.requestFrom(MLX_90640_I2C_ADDRESS, 2);
        if(Wire.available() <= 2) {  
            for(i=0;i<2;i++) 
                statusRegister[i] = Wire.read(); 
        }

        if((statusRegister[1] & 0x08) == 0x08) {
			#ifdef DISPLAY_SENSOR_DEBUG
				Serial.println("New Data, wait counter :" + String(timeout) + ", " + "StatusRegister : " + String(statusRegister[0] * 256 + statusRegister[1], HEX));
			#endif
            return 1;
        }
        
        if(timeout == 30) {
            Serial.println("timout at : " + String(timeout)); 
            return 0;
        } else {
			delay(delayStep);  
			timeout++;
		}
    }
}

// This function reads one subpage and stores the values in a two dimensional array
void MLX90640::readSubPage(bool page) {
	
	int cnt;
	
	//check whether the measurement is done
	if(measurementDone()) {
		startMeasurement(); // start a new measurement
		getSensorData();
	} else {
		Serial.println("Measurement of subpage : " + String(page) + " failed");
	}
	
	// This part checks wheter the sensor data has to be saved according to the interleaved or chess pattern. The page argument is used to select the subpage.
	if(measurementMode == 0) {
		i=page;
		cnt=0;
		while(i<MLX_90640_ROW) {
			j=0;
			while(j<MLX_90640_COL) {
				temperatureMatrix[i][j] = dataArray[cnt];
				j+=2;
				cnt++;
			}
			j=1;
			while(j<MLX_90640_COL) {
				temperatureMatrix[i][j] = dataArray[cnt];
				j+=2;
				cnt++;
			}
			i+=2;
		}
	} else {
		cnt=0;
		int chess = page;
		for (i=0;i<MLX_90640_ROW;i++) {
			for (j=chess;j<MLX_90640_COL;j+=2) {
				temperatureMatrix[i][j] = dataArray[cnt];
				cnt++;
			}
			if(chess == 1)
				chess = 0;
			else
				chess = 1;
		}
	}
}

// This function requests the needed amount of bytes from the sensor RAM and puts it in a 1 dimensional array
void MLX90640::getSensorData() {
	
	int sensorDataMByte = 0x00;
	int sensorDataLByte = 0x00;
	
	//request sensor data from RAM
	Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
	Wire.write(MLX_RAM_ADDRESS_M);
	Wire.write(MLX_RAM_ADDRESS_L);
	Wire.endTransmission(false);
	
	Wire.requestFrom(MLX_90640_I2C_ADDRESS, MLX_SENSOR_INPUT_LEN);
	
	if(Wire.available() <= MLX_SENSOR_INPUT_LEN) {
		for(i=0;i<MLX_SENSOR_INPUT_LEN/2;i++) {
			sensorDataMByte = Wire.read();
			sensorDataLByte = Wire.read();
			dataArray[i] = (sensorDataMByte << 8) + sensorDataLByte;
			#ifdef DISPLAY_ALL_READ_DATA
				Serial.print(String(dataArray[i]) + " , ");
				if(i%32 == 0 and i != 0)
					Serial.println("");
			#endif
		}
	}
}


// This function requests all EEPROM data and stores it in a buffer
void MLX90640::getEEPROMData() {
    Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
    Wire.write(MLX_EEPROM_M);
    Wire.write(MLX_EEPROM_L);
    
    Wire.endTransmission(false);
    Wire.requestFrom(MLX_90640_I2C_ADDRESS, MLX_EEPROM_SIZE);
    if(Wire.available() <= MLX_EEPROM_SIZE) {  
		for(i=0;i<MLX_EEPROM_SIZE;i++)
			EEPROMData[i] = Wire.read();
    }
	
	#ifdef DISPLAY_MEM_DEBUG
		Serial.println("EEPROM Data : ");
		Serial.print("\t");
		for(i=0;i<16;i++)
			Serial.print(String(i, HEX) + " \t");
		
		for(i=0;i<MLX_EEPROM_SIZE / 2;i++) {
			if(i%16 == 0) 
				Serial.print("\n" + String(i, HEX) + " \t");
			Serial.print(String(EEPROMData[i*2]*256 + EEPROMData[i*2+1]) + "\t");
		}
		Serial.println("");
	#endif
}

// This function requests the resolution value stored in RAM
int MLX90640::getResFromRAM() {
	
	int sensorDataMByte = 0x00;
	int sensorDataLByte = 0x00;
	
	Wire.beginTransmission(MLX_90640_I2C_ADDRESS);
	Wire.write(0x80);
	Wire.write(0x0D);
	Wire.endTransmission(false);
	
	Wire.requestFrom(MLX_90640_I2C_ADDRESS, 2);
	
	if(Wire.available() <= 2) {
		sensorDataMByte = Wire.read();
		sensorDataLByte = Wire.read();
		return (sensorDataMByte << 8) + sensorDataLByte;
	}
}

// This function is used to figure out if the row and col from the designated pixel are even or odd. are used to calibrate the sensor data.
int MLX90640::getOddEven(int row,int col) {
    if (row % 2){               // odd row 
		if (col % 2) return 0;   // odd col
		else return 2;           // even col
    }
    else{                       // even row
		if (col % 2) return 1;    // odd col
		else return 3;           // even col
    }
    
}


// This function is used to calculate the PTAT at measurement.
void MLX90640::getPtat() {
	int val;

	val = dataArray[800];
	if (val>32767 ) val -= 65536;
	VPTAT = val;
	
	val = dataArray[768];
    if(val > 32767)
        val -= 65536;
	VBE = val;
	
	val = dataArray[810];
	if (val>32767 ) val -= 65536;
	DELTA_V = val; 

	#ifdef DISPLAY_CALIB_DEBUG
		Serial.println("VPTAT=" + String(VPTAT));
		Serial.println("VBE=" + String(VBE));
		Serial.println("DELTA_V=" + String(DELTA_V));
	#endif
}

// not finisched yet
void MLX90640::correctRes(){
	int val;

	val = 2; //getResFromRAM();
	int Res_ram = (val & 0x0C00) / 1024;

    RES_CORR = pow(2, RES_EE) / pow(2, Res_ram);
	
	#ifdef DISPLAY_CALIB_DEBUG
		Serial.println("Res_ram=" + String(Res_ram));
		Serial.println("RES_CORR=" + String(RES_CORR));
    #endif
}

// This function is used to calculate the supply voltage at measurement.
void MLX90640::calculateVdd() {

	int val = dataArray[810];
	if (val > 32767) val -= 65536;

	g_Vdd = ((RES_CORR * val - Vdd_25) / Kv_Vdd) + 3.3;
	
	#ifdef DISPLAY_CALIB_DEBUG
		Serial.println("-VDD_ram= " + String(val));
		Serial.println("-Vdd_25= " + String(Vdd_25, 4));
		Serial.println("-Kv_Vdd= " + String(Kv_Vdd, 4));
		Serial.println("-VDD= " + String(g_Vdd, 4)); 
	#endif
}

// This function is used to calculate the ambient temperature at measurement.
void MLX90640::calculateTa() {
   
	getPtat();
	
	DELTA_Volt = ((float)DELTA_V - Vdd_25) / Kv_Vdd;
	VPTAT_art = (float)(VPTAT / (VPTAT * ALPHA_PTAT + VBE)) * 262144.0;

	Ta = VPTAT_art / (1 + Kv_PTAT * DELTA_Volt);
	Ta = (Ta - VPTAT_25) / Kt_PTAT + 25;
    
	#ifdef DISPLAY_CALIB_DEBUG
	   Serial.println("-Ta= " + String(Ta,4));
	   Serial.println("-VPTAT= " + String(VPTAT));
	   Serial.println("-ALPHA_PTAT= " + String(ALPHA_PTAT,4));
	   Serial.println("-VBE= " + String(VBE));
	   Serial.println("-DELTA_Volt= " + String(DELTA_Volt,4));      
	   Serial.println("-Vdd_25= " + String(Vdd_25,4));
	   Serial.println("-Kv_Vdd= " + String(Kv_Vdd,4));    
	   Serial.println("-VPTAT_art= " + String(VPTAT_art,4));
	   Serial.println("-Kv_Vdd= " + String(Kv_Vdd,4));    
	   Serial.println("-Kv_PTAT= " + String(Kv_PTAT,4));
	   Serial.println("-Kt_PTAT= " + String(Kt_PTAT,4));
	#endif 
} 

// This function is used to calculate the commen gain of all pixels at measurement.
void MLX90640::calculateKgain() {
	int val;

	val = dataArray[778];
	if (val>32767 ) val-=65536;
	K_gain = GainMeas_25_3v2/(float)val; 
	
	#ifdef DISPLAY_CALIB_DEBUG 
		Serial.println("Gain compensation K_gain : " + String(K_gain)); 
	#endif 
}


// This function compensates the IR data according to the interleaved pattern. TODO: This function is not validated yet.
void MLX90640::IRdatacompensationInterleaved(bool page) {
	
	int val = 0;
	int tempVal = 0;
	int ODD_en,EVEN_en;
	
	float pix_gain_cp_sp0 = 0.0;
	float pix_gain_cp_sp1 = 0.0;
	float pix_os_cp_sp0 = 0.0;
	float pix_os_cp_sp1 = 0.0;
	
	float pixosref = 0.0;
	float pixos = 0.0;
	
	float vir_compensated = 0.0;
	float Alpha_comp = 0.0;
	

	calculateVdd();
	calculateTa(); 
	calculateKgain();

	// page 0
	val = dataArray[776];
	if (val > 32767) 
		val -= 65536;
	pix_gain_cp_sp0 = (float)val * K_gain;

	// page 1
	val = dataArray[808];
	if (val > 32767) val -= 65536;
	pix_gain_cp_sp1 = (float)val * K_gain;
	
	#ifdef DISPLAY_CALIB_DEBUG  
		Serial.println("#pix_gain_cp_sp0 ->" + String(pix_gain_cp_sp0, 15)); 
		Serial.println("#pix_gain_cp_sp1 ->" + String(pix_gain_cp_sp1, 15)); 
	#endif  

	pix_os_cp_sp0 = pix_gain_cp_sp0 - OFFSET_CP_subpage_0 * (1 + Kta_cp * (Ta - 25.0)) * (1 + KVcp * (g_Vdd - 3.3));
	pix_os_cp_sp1 = pix_gain_cp_sp1 - OFFSET_CP_subpage_1 * (1 + Kta_cp * (Ta - 25.0)) * (1 + KVcp * (g_Vdd - 3.3));
	
	#ifdef DISPLAY_CALIB_DEBUG 
		Serial.println("#pix_os_cp_sp0 ->" + String(pix_os_cp_sp0, 15)); 
		Serial.println("#pix_os_cp_sp1 ->" + String(pix_os_cp_sp1, 15));
	#endif  
	
	for (i=page;i<MLX_90640_ROW;i+=2) {        
		for (j=0;j<MLX_90640_COL;j++) {
			
			tempVal = temperatureMatrix[i][j];
			if(tempVal > 32767)
				tempVal -= 65536;
			tempVal *= K_gain;
			
			pixosref = Pix_os_ref_range_1[i][j];

			val = getOddEven(i+1,j+1);  
			switch (val){
				case 0:
					Kv = Kv_odd_odd;
					break; 
				case 1:
					Kv = Kv_even_odd;
					break;
				case 2:
					Kv = Kv_odd_even;
					break;
				case 3:
					Kv = Kv_even_even;
					break;
				default:
					Kv = Kv_odd_odd;
					Serial.println("-Kv- THIS MAY NOT HAPPEN");
					break;
			}

			pixos = tempVal - pixosref * (1 + Kta_range_1[i][j] * (Ta - 25.0)) * (1 + Kv * (g_Vdd - 3.3));
			//look at page 23, in case of interleaved mode, more shit is needed
			#ifdef DISPLAY_CALIB_DEBUG 
				Serial.println("#pixos ->" + String(pixos, 15)); 
			#endif 

			if (i % 2) { // odd row 
				ODD_en = 1 ;
				EVEN_en = 0;
			} else {
				ODD_en = 0;
				EVEN_en = 1;
			}

			vir_compensated = pixos - TGC * ((float)ODD_en * (float)pix_os_cp_sp0 + (float)EVEN_en * (float)pix_os_cp_sp1);
			
			#ifdef DISPLAY_CALIB_DEBUG  
				Serial.println("#vir_compensated ->"  + String(vir_compensated, 15)); 
				Serial.println("#pixos ->" + String(pixos, 15)); 
				Serial.println("#TGC ->" + String(TGC, 15));  
				Serial.println("#ODD_en ->" + String(ODD_en)); 
				Serial.println("#EVEN_en ->" + String(EVEN_en)); 
				Serial.println("#pix_os_cp_sp0 ->" + String(pix_os_cp_sp0, 15)); 
				Serial.println("#pix_os_cp_sp1 ->" + String(pix_os_cp_sp1, 15));     
				Serial.println("#TGC*.... ->" + String(TGC*(ODD_en*pix_os_cp_sp0+EVEN_en*pix_os_cp_sp1), 15));
				Serial.println("#ALPHA_CP_subpage_0 ->" + String(ALPHA_CP_subpage_0, 15));
				Serial.println("#ALPHA_CP_subpage_1 ->" + String(ALPHA_CP_subpage_1, 15));				
			#endif  

			Alpha_comp = (Alpha[i][j] - TGC * (ODD_en * ALPHA_CP_subpage_0 + EVEN_en * ALPHA_CP_subpage_1)) * (1 + KsTa * (Ta - 25.0)); 

			#ifdef DISPLAY_CALIB_DEBUG
				Serial.println("#Alpha[i][j] ->" + String(Alpha[i][j], 15)); 
				Serial.println("#Alpha_comp ->" + String(Alpha_comp, 15)); 
			#endif 
			
			temperatureMatrix[i][j] = tempVal;
		}
	}
	
}

// This function compensates the IR data according to the chess pattern. 
void MLX90640::IRdatacompensationChess(bool page) {
	
	int val = 0;
	int tempVal = 0;
	int chess = page;
	int chessPattern = 0;
	
	float pix_gain_cp_sp0 = 0.0;
	float pix_gain_cp_sp1 = 0.0;
	float pix_os_cp_sp0 = 0.0;
	float pix_os_cp_sp1 = 0.0;
	
	double pixosref = 0.0;
	double pixos = 0.0;
	
	float vir_compensated = 0.0;
	float vir_emissivity_compensated = 0.0;
	float Alpha_comp = 0.0;
	
	// Calculate calibration data.
	correctRes();
	calculateVdd();
	calculateTa(); 
	calculateKgain();

	// Calculate commen gain for all pixels per subpage
	val = dataArray[776];
	if (val > 32767) val -= 65536;
	pix_gain_cp_sp0 = (float)val * K_gain;

	val = dataArray[808];
	if (val > 32767) val -= 65536;
	pix_gain_cp_sp1 = (float)val * K_gain;
	
	#ifdef DISPLAY_CALIB_DEBUG  
		Serial.println("#pix_gain_cp_sp0 ->" + String(pix_gain_cp_sp0, 15)); 
		Serial.println("#pix_gain_cp_sp1 ->" + String(pix_gain_cp_sp1, 15)); 
	#endif  
	
	//g_Vdd = 2.85;
	
	// Calculate commen offset of all pixels per subpage. TODO: apply moving average filter to limit noise in To calculation.
	pix_os_cp_sp0 = pix_gain_cp_sp0 - OFFSET_CP_subpage_0 * (1 + Kta_cp * (Ta - 25.0)) * (1 + KVcp * (g_Vdd - 3.3));
	pix_os_cp_sp1 = pix_gain_cp_sp1 - OFFSET_CP_subpage_1 * (1 + Kta_cp * (Ta - 25.0)) * (1 + KVcp * (g_Vdd - 3.3));
	
	#ifdef DISPLAY_CALIB_DEBUG 
		Serial.println("#pix_os_cp_sp0 ->" + String(pix_os_cp_sp0, 2)); 
		Serial.println("#pix_os_cp_sp1 ->" + String(pix_os_cp_sp1, 7));
	#endif  
	
	for (i=0;i<MLX_90640_ROW;i++) {
		for (j=chess;j<MLX_90640_COL;j+=2) {
			
			// Gain compensation per pixel.
			if(temperatureMatrix[i][j] > 32767)
				temperatureMatrix[i][j] -= 65536;
			tempVal = temperatureMatrix[i][j] * K_gain;
			
			#ifdef DISPLAY_CALIB_DEBUG
				Serial.println("#temperatureMatrix :" + String(temperatureMatrix[i][j]));
			#endif 
			
			// Get offset per pixel.
			pixosref = Pix_os_ref_range_1[i][j];

			// Get supply voltage per pixel .
			val = getOddEven(i+1,j+1);  
			switch (val){
				case 0:
					Kv = Kv_odd_odd;
					break; 
				case 1:
					Kv = Kv_even_odd;
					break;
				case 2:
					Kv = Kv_odd_even;
					break;
				case 3:
					Kv = Kv_even_even;
					break;
				default:
					Kv = Kv_odd_odd;
					Serial.println("-Kv- THIS MAY NOT HAPPEN");
					break;
			}
			
			// Compensate for  the gain, offset and VDD.
			pixos = tempVal - pixosref * (1 + Kta_range_1[i][j] * (Ta - 25.0)) * (1 + Kv * (g_Vdd - 3.3));
				
			// Compensate for user defined emissivity.
			vir_emissivity_compensated = tempVal / emissivity;
			
			val = getOddEven(i+1,j+1);  
			switch (val){
				case 0:
					chessPattern = 0; // Odd_odd
					break; 
				case 1:
					chessPattern = 1; // Even_odd
					break;
				case 2:
					chessPattern = 1; // Odd_even
					break;
				case 3:
					chessPattern = 0; // Even_even
					break;
				default:
					chessPattern = 0; // Odd_odd
					Serial.println("-Kv- THIS MAY NOT HAPPEN");
					break;
			}
			
			// Compensate for temperature gradients per pixel.
			vir_compensated = vir_emissivity_compensated - TGC * ((1 - chessPattern) * (float)pix_os_cp_sp0 + chessPattern * (float)pix_os_cp_sp1);
			
			// Normalize the sensivity of each pixel.
			Alpha_comp = Alpha[i][j] - TGC * ((1 - chessPattern) * ALPHA_CP_subpage_0 + chessPattern * ALPHA_CP_subpage_1) * (1 + KsTa * (Ta - 25.0)); 

			// Calculate compensated IR pixel data
			temperatureMatrix[i][j] = tempVal; //vir_compensated/Alpha_comp;
			
			#ifdef DISPLAY_CALIB_DEBUG
				Serial.println("#Kta_range_1[i][j] ->" + String(Kta_range_1[i][j], 15)); 
				Serial.println("#Ta ->" + String(Ta, 15)); 
				Serial.println("#Kv ->" + String(Kv, 15));
				Serial.println("#g_Vdd ->" + String(g_Vdd, 15)); 
				Serial.println("#TGC ->" + String(TGC)); 
				Serial.println("#pixosref ->" + String(pixosref, 15)); 
				Serial.println("#pixos ->" + String(pixos, 15)); 
				Serial.println("#vir_emissivity_compensated ->"  + String(vir_emissivity_compensated, 15)); 
				Serial.println("#vir_compensated ->"  + String(vir_compensated, 15)); 
				Serial.println("#chessPattern ->" + String(chessPattern)); 
				Serial.println("#Alpha[i][j] ->" + String(Alpha[i][j], 15)); 
				Serial.println("#Alpha_comp ->" + String(Alpha_comp, 15));
				Serial.println("#temperatureMatrix :" + String(temperatureMatrix[i][j]));
			#endif 
		}
		
		// Making sure that the IR data array is compensated according to the subpage pixels.
		if(chess == 1)
			chess = 0;
		else
			chess = 1;
	}
}

// This function compensates the IR data according to the chess pattern. 
void MLX90640::IRdatacompensationSimple(bool page) {

	int chess = page;
	
	// Calculate calibration data.
	calculateKgain();
	
	for (i=0;i<MLX_90640_ROW;i++) {
		for (j=chess;j<MLX_90640_COL;j+=2) {
			// Gain compensation per pixel.
			if(temperatureMatrix[i][j] > 32767)
				temperatureMatrix[i][j] -= 65536;
			temperatureMatrix[i][j] = temperatureMatrix[i][j] * K_gain + 5000; // compensate for gain and offset the temperature range in the positive realm
			
			maxTemp = max(temperatureMatrix[i][j], maxTemp);
			minTemp = min(temperatureMatrix[i][j], minTemp);
		}
		// Making sure that the IR data array is compensated according to the subpage pixels.
		if(chess == 1)
			chess = 0;
		else
			chess = 1;
	}
}

// These functions are called once on boot and are used to restore calibration parameters, these values are fetched from the sensor EEPROM

void MLX90640::restoreVDD(){
	
	int val, val1;
	
	val = ((int)EEPROMData[(0x2433-0x2400)*2])*256+(int)EEPROMData[(0x2433-0x2400)*2+1];
	Serial.println("EEPROMData : " + String(EEPROMData[(0x2433-0x2400)*2]) + " " + String(EEPROMData[(0x2433-0x2400)*2+1]));
	val1 = val & 0xff00;
	val1 /=  256; 
	Serial.println ("val : val1 : " + String(val) + " " +  String(val1));	
	if (val1 > 127) val1 -= 256;
	Kv_Vdd = val1 * 32;
   
	val1 = val & 0x00ff;
	Vdd_25 = (val1 - 256) * 32 - 8192; 
   
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kv_vdd:" + String(Kv_Vdd));
		Serial.println("Vdd_25:" + String(Vdd_25));
	#endif    
}

void MLX90640::restorePTAT(){

	int val, val1;
	
	val = ((int)EEPROMData[(0x2432-0x2400)*2])*256+(int)EEPROMData[(0x2432-0x2400)*2+1]; 
	val1 = val & 0xfc00;
	val1 /= 1024;  
    
	if (val1 > 31) val1 -= 64;
	Kv_PTAT = val1 / 4096.0;
   
	val1 = val & 0x03ff;
	if (val1 > 511) val1 -= 1024;
	Kt_PTAT = val1 / 8.0;
	
	val = ((int)EEPROMData[(0x2431-0x2400)*2])*256+(int)EEPROMData[(0x2431-0x2400)*2+1];
	if (val > 32767 ) val -= 65536;
	VPTAT_25 = val;
   
	val = ((int)EEPROMData[(0x2410-0x2400)*2])*256+(int)EEPROMData[(0x2410-0x2400)*2+1];
	val1 = val & 0xf000;
	ALPHA_PTAT = (float)val1 / 4096;
	ALPHA_PTAT = ALPHA_PTAT / 4 + 8;
   
	#ifdef DISPLAY_RESTORE_DEBUG  
		Serial.println("Kv_PTAT:" + String(Kv_PTAT, 15));
		Serial.println("Kt_PTAT:" + String(Kt_PTAT, 15));
		Serial.println("VPTAT_25:" + String(VPTAT_25));
		Serial.println("ALPHA_PTAT:" + String(ALPHA_PTAT, 15));
	#endif
}

void MLX90640::restoreGAIN(){
    int val;

    val = ((int)EEPROMData[(0x2430-0x2400)*2])*256+(int)EEPROMData[(0x2430-0x2400)*2+1];
    if (val > 32767) val -= 65536;
    GainMeas_25_3v2 = (float)val;
	
    #ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("GainMeas_25_3v2 = " + String(GainMeas_25_3v2, 15));
    #endif 
}

void MLX90640::restoreTGC(){
    int val;
	
    // TGC
    val = ((int)EEPROMData[(0x243c-0x2400)*2])*256+(int)EEPROMData[(0x243c-0x2400)*2+1];
    val = (val & 0x00ff); 
    if (val > 127) val -= 256; 
    
    TGC = (float)val / 32.0;
	
    #ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("TGC = " + String(TGC, 15));
    #endif
}

void MLX90640::restoreKSTA(){
	
    // KsTa
    int val;
    val = ((int)EEPROMData[(0x243c-0x2400)*2])*256+(int)EEPROMData[(0x243c-0x2400)*2+1];
    val = (val & 0xff00) / 256;
    if (val > 127) val -= 256;
    KsTa = (float)val / 8192.0; 
	
    #ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("KsTa = " + String(KsTa, 15)); 
    #endif
}

void MLX90640::restoreKSTO(){
    // KsTo
    int val,val1;
    val = ((int)EEPROMData[(0x243d-0x2400)*2])*256+(int)EEPROMData[(0x243d-0x2400)*2+1];
    val = (val & 0xff00)/256;
    if (val > 127) val=val-256;
    
    val1 = ((int)EEPROMData[(0x243f-0x2400)*2])*256+(int)EEPROMData[(0x243f-0x2400)*2+1];
    val1 = (val1 & 0x000f)+8;
    
    KsTo = (float)val/(float)pow(2.0,(double)val1);
	
    #ifdef DISPLAY_RESTORE_DEBUG  
		Serial.println("KsTo = " + String(KsTo, 15)); 
    #endif
}

void MLX90640::restoreALPHACP(){

    int val, val1;
	
	val = ((int)EEPROMData[(0x2420-0x2400)*2])*256+(int)EEPROMData[(0x2420-0x2400)*2+1];
	val1 = val & 0xF000;
	float alphaScale = val1 / 4096 + 27;
	
	val = ((int)EEPROMData[(0x2439-0x2400)*2])*256+(int)EEPROMData[(0x2439-0x2400)*2+1];
	val1 = (val & 0x03FF);
    ALPHA_CP_subpage_0 = val1 / (float)pow(2, (double)alphaScale);
    
    val1 = (val & 0xFC00) / 1024;
    if (val1 > 31)
        val1 -= 64;
	ALPHA_CP_subpage_1 = (1 + val1 / 128) * ALPHA_CP_subpage_0;
	
    #ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("alphaScale = " + String(alphaScale, 15));
		Serial.println("ALPHA_CP_subpage_0 = " + String(ALPHA_CP_subpage_0, 15));
		Serial.println("ALPHA_CP_subpage_1 = " + String(ALPHA_CP_subpage_1, 15));
    #endif
}

void MLX90640::restoreOFFSET(){
	
	int row, col;
	int adr,adr1,adr2;
	
	//Offset average
	int val = ((int)EEPROMData[(0x2411-0x2400)*2])*256+(int)EEPROMData[(0x2411-0x2400)*2+1];
	if (val > 32767 ) val -= 65536;
	float OFFSET_avg = (float)val;

    // OCCscaleremnant
    val = ((int)EEPROMData[(0x2410-0x2400)*2])*256+(int)EEPROMData[(0x2410-0x2400)*2+1];
    unsigned int OCCscaleremnant = val & 0x000f;
    
    // OCC_scale_row
    int OCC_scale_row = (val & 0x0f00) / 256; 
    
    // OCC_scale_col
    int OCC_scale_col = (val & 0x00f0) / 16;  
    
    for (row=0; row<MLX_90640_ROW; row++) {
		// OCC_row
		adr = (((int)row / 4) + 0x2412) - 0x2400;
		val = ((int)EEPROMData[adr*2])*256+(int)EEPROMData[adr*2+1];
		int OCC_row;
		switch(row % 4){
			case 0:
				OCC_row = val & 0x000f;
				break;
			case 1:
				OCC_row = val & 0x00f0;
				OCC_row = OCC_row / 16;
				break;
			case 2:
				OCC_row = val & 0x0f00;
				OCC_row = OCC_row / 256;
				break;
			case 3:
				OCC_row = val & 0xf000;
				OCC_row = OCC_row / 4096;
				break;
			default :
				OCC_row = val & 0x000f;
				Serial.println("OCC_row->this default can not!!");
				break;
		}
		
		if ( OCC_row > 7) OCC_row -= 16;
		
		for (col=0; col<MLX_90640_COL; col++) {
			
			#ifdef DISPLAY_OFFSET_PER_PIXEL
				Serial.println("row address[" + String(row) + "][" + String(col) + "] = " + String(adr+0x2400));
			#endif

			// OCC_col
			adr1 = (((int)col / 4) + 0x2418) - 0x2400;
			#ifdef DISPLAY_OFFSET_PER_PIXEL
				Serial.println("col address[" + String(row) + "][" + String(col) + "] = " + String(adr1+0x2400));
			#endif
		 
			val = ((int)EEPROMData[adr1*2])*256+(int)EEPROMData[adr1*2+1];
			int OCC_col;
			switch(col % 4){
				case 0:
					OCC_col = val & 0x000f;
					break;
				case 1:
					OCC_col = val & 0x00f0;
					 OCC_col = OCC_col / 16;
					break;
				case 2:
					OCC_col = val & 0x0f00;
					OCC_col = OCC_col / 256;
					break;
				case 3:
					OCC_col = val & 0xf000;
					OCC_col = OCC_col / 4096;
					break;
				default :
					OCC_col = val & 0x000f;
					Serial.println("OCC_col->this default can not!!");
					break;
			}
			if ( OCC_col > 7) OCC_col -= 16;
	 
			// get pix-offset
			adr2 = (row * 32 + col  +0x2440) - 0x2400;          
			val = ((int)EEPROMData[adr2*2])*256+(int)EEPROMData[adr2*2+1];
			int offset_row_col = val & 0xfc00;
			offset_row_col = offset_row_col / 1024;
			if ( offset_row_col > 31) offset_row_col -= 64;
			
			// insert value in array of Pix_os_ref_range_1[MLX_90640_ROW][MLX_90640_COL]
			Pix_os_ref_range_1[row][col] = OFFSET_avg+(float)OCC_row*(float)pow(2.0,(double)OCC_scale_row)+(float)OCC_col*(float)pow(2.0,(double)OCC_scale_col);
			Pix_os_ref_range_1[row][col] = Pix_os_ref_range_1[row][col] + (float)pow(2.0,(double)OCCscaleremnant)*(float)offset_row_col;
			
			#ifdef DISPLAY_OFFSET_PER_PIXEL
				Serial.println("pix-offset address has value [" + String(row) + "][" + String(col) + "] = " + String(offset_row_col, 15));
				Serial.println("Pix_os_ref_range_1[" + String(row) + "][" + String(col) + "] = " + String(Pix_os_ref_range_1[row][col], 15));
			#endif
        } 
    }   
}  

void MLX90640::restoreKV(){

    int val,val1;
    float two_power_kv_scale;
    
    // Kv_scale
    val = ((int)EEPROMData[(0x2438-0x2400)*2])*256+(int)EEPROMData[(0x2438-0x2400)*2+1];
    val = (val & 0x0f00) / 256;
    two_power_kv_scale = (float)pow(2.0, (double)val);
    
    // Kv_odd_odd
    val = ((int)EEPROMData[(0x2434-0x2400)*2])*256+(int)EEPROMData[(0x2434-0x2400)*2+1];
    val1 = (val & 0xf000) / 4096;
    if (val1 > 7) val1 -= 16;
    Kv_odd_odd = (float)val1 / two_power_kv_scale;
	
	#ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("Kv_odd_odd=" + String(Kv_odd_odd, 15));
	#endif
    
    // Kv_even_odd
    val1 = (val & 0x0f00) / 256;
    if (val1 > 7) val1 -= 16;
    Kv_even_odd = (float)val1 / two_power_kv_scale;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kv_even_odd=" + String(Kv_even_odd, 15));
	#endif        
	
    // Kv_odd_even
    val1 = (val & 0x00f0) / 16;
    if (val1 > 7) val1 -= 16;
    Kv_odd_even = (float)val1 / two_power_kv_scale;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kv_odd_even=" + String(Kv_odd_even, 15));
	#endif   
	
    // Kv_even_even
    val1 = (val & 0x000f);
    if (val1 > 7) val1 -= 16;
    Kv_even_even = (float)val1 / two_power_kv_scale;
	
	#ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("Kv_even_even=" + String(Kv_even_even, 15));
	#endif   
}

void MLX90640::restoreKTA(){
	
    int ind=0;
    float Kta_odd_odd, Kta_even_odd, Kta_odd_even, Kta_even_even;
    int val,val1, tmp_Kta;
    int two_power_kta_scale1, two_power_kta_scale2;
    
    val = ((int)EEPROMData[(0x2438-0x2400)*2])*256+(int)EEPROMData[(0x2438-0x2400)*2+1];
	
	// Kta_scale1
    val1 = (val & 0x00f0) / 16 + 8;
    two_power_kta_scale1 = (float)pow(2.0, (double)val1);
    
    // Kta_scale2
	val1 = (val & 0x000f);
	two_power_kta_scale2 = (float)pow(2.0, (double)val1);
    
    // Kta_odd_odd
    val = ((int)EEPROMData[(0x2436-0x2400)*2])*256+(int)EEPROMData[(0x2436-0x2400)*2+1];
    val1 = (val & 0xff00)/256;
    if (val1 > 127) val1 -= 256;
    Kta_odd_odd = (float)val1;
	
	#ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("Kta_odd_odd=" + String(Kta_odd_odd, 15));
	#endif    
	
    // Kta_even_odd
    val1 = (val & 0x00ff);
    if (val1 > 127) val1 -= 256;
    Kta_even_odd = (float)val1;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kta_even_odd=" + String(Kta_even_odd, 15));
	#endif       
	
    // Kta_odd_even
    val = ((int)EEPROMData[(0x2437-0x2400)*2])*256+(int)EEPROMData[(0x2437-0x2400)*2+1];
    val1 = (val & 0xff00) / 256;
    if (val1 > 127) val1 -= 256;
    Kta_odd_even = (float)val1;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kta_odd_even=" + String(Kta_odd_even, 15));
	#endif   
	
    // Kta_even_even
    val1 = (val & 0x00ff);
    if (val1 > 127) val1 -= 256;
    Kta_even_even = (float)val1;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kta_even_even=" + String(Kta_even_even, 15));  
	#endif
    
    for (int row=0; row<MLX_90640_ROW; row++) {
		for (int col=0; col<MLX_90640_COL; col++) {
			
			val1 = (row * 32 + col + 0x2440) - 0x2400;
			val = ((int)EEPROMData[val1*2])*256+(int)EEPROMData[val1*2+1];
			
			#ifdef DISPLAY_KTA_PER_PIXEL
				Serial.println("Kta_address[" + String(row) + "][" + String(col) + " = " + String(val1+0x2400) + ", content=" + String(val));
			#endif           
        
			int Kta = val & 0x000e; 
	
			Kta /= 2;
			if (Kta > 3) Kta -= 8;
			
			val1 = getOddEven(row + 1, col + 1);
			switch (val1){
				case 0:
					tmp_Kta = Kta_odd_odd;
					#ifdef DISPLAY_KTA_PER_PIXEL  
						Serial.println("[" + String(row) + "][" + String(col) + "]" + " = Kta_odd_odd");
					#endif
					break;
				case 1:
					tmp_Kta = Kta_even_odd;
					#ifdef DISPLAY_KTA_PER_PIXEL  
						Serial.println("[" + String(row) + "][" + String(col) + "]" + "= Kta_even_odd");
					#endif
					break;
				case 2:
				    tmp_Kta = Kta_odd_even; 
					#ifdef DISPLAY_KTA_PER_PIXEL 
						Serial.println("[" + String(row) + "][" + String(col) + "]" + "= Kta_odd_even\n\r");
				    #endif
				    break;
				case 3:
				    tmp_Kta = Kta_even_even;
				    #ifdef DISPLAY_KTA_PER_PIXEL  
						Serial.println("[" + String(row) + "][" + String(col) + "]" + "= Kta_even_even\n\r");
				    #endif
				    break;
				default:
					tmp_Kta = Kta_odd_odd;
					Serial.println("Kta-> this may not happen\n\r");
					break;
			}
			
			Kta_range_1[row][col]= (tmp_Kta + (float)Kta * two_power_kta_scale2) / two_power_kta_scale1;
         
			#ifdef DISPLAY_KTA_PER_PIXEL
				Serial.println("Kta[" + String(row) + "][" + String(col) + "]" + " = " + String(Kta_range_1[row][col], 15)); 
			#endif
		}
    }
}

void MLX90640::restoreOFFSETCP(){

    int val,val1;
    
    // OFFSET_CP_subpage_0
    val = ((int)EEPROMData[(0x243a-0x2400)*2])*256+(int)EEPROMData[(0x243a-0x2400)*2+1];
    val1 = (val & 0x03ff); 
    if (val1 > 511) val1 -= 1024;
    OFFSET_CP_subpage_0 = (float)val1;
    
    //OFFSET_CP_subpage_1
    val1 = (val & 0xfc00) / 1024; 
    if (val1 > 31) val1 -= 64;
    OFFSET_CP_subpage_1 = OFFSET_CP_subpage_0 + (float)val1;
	
    #ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("OFFSET_CP_subpage_0 = " + String(OFFSET_CP_subpage_0, 15));
		Serial.println("OFFSET_CP_subpage_1 = " + String(OFFSET_CP_subpage_1, 15));
    #endif
}

void MLX90640::restoreKVCP(){
    int val,val1;
    
    // KVcp_EE
    val = ((int)EEPROMData[(0x243b-0x2400)*2])*256+(int)EEPROMData[(0x243b-0x2400)*2+1];
    val1 = (val & 0xff00) / 256; 
    if (val1 > 127) val1 -= 256; 
	
    // Kv scale
    val = ((int)EEPROMData[(0x2438-0x2400)*2])*256+(int)EEPROMData[(0x2438-0x2400)*2+1];
    val = (val & 0x0f00) / 256;  
	
	// Kv cp
    KVcp = (float)val1 / (float)pow(2.0, (double)val);
	
    #ifdef DISPLAY_RESTORE_DEBUG 
		Serial.println("KVcp = " + String(KVcp, 15));
    #endif
}

void MLX90640::restoreKTACP(){
    int val,val1;
    
    // KTacp_EE
    val = ((int)EEPROMData[(0x243b-0x2400)*2])*256+(int)EEPROMData[(0x243b-0x2400)*2+1];
    val1 = (val & 0x00ff); 
    if (val1 > 127) val1 -= 256; 
    
    // KTa_scale
    val = ((int)EEPROMData[(0x2438-0x2400)*2])*256+(int)EEPROMData[(0x2438-0x2400)*2+1];
    val = (val & 0x00f0) / 16 + 8; 
    
    Kta_cp = (float)val1 / (float)pow(2.0, (double)val);
    
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Kta_cp = " + String(Kta_cp, 15));
	#endif
}

void MLX90640::restoreSENSITIVITY(){

    int val,val1;
    int adr, adr1, adr2;
    int ACC_row, ACC_col;
	int row, col;
	int alpha_pixel_row_col;
    
    val = ((int)EEPROMData[(0x2420-0x2400)*2])*256+(int)EEPROMData[(0x2420-0x2400)*2+1];
    Alpha_scale = (val & 0xf000) / 4096 + 30;
    
    ACC_scale_row = (val & 0x0f00) / 256;
	
    ACC_scale_col = (val & 0x00f0) / 16;
	
    ACC_scale_remnant = val & 0x000f; 
	
	val1 = ((int)EEPROMData[(0x2421-0x2400)*2])*256+(int)EEPROMData[(0x2421-0x2400)*2+1];
    Alpha_ref = (float)val1;
	
    #ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("Alpha_scale=" + String(Alpha_scale));
		Serial.println("Alpha_ref=" + String(Alpha_ref, 15));
		Serial.println("ACC_scale_row=" + String(ACC_scale_row));
		Serial.println("ACC_scale_col=" + String(ACC_scale_col));	
		Serial.println("ACC_scale_remnant=" + String(ACC_scale_remnant));
    #endif  
	
    for (row=0; row<MLX_90640_ROW; row++) {
		// ACC_row
		adr = (((int)row / 4) + 0x2422) - 0x2400;          
		val = ((int)EEPROMData[adr*2])*256+(int)EEPROMData[adr*2+1];      
		switch(row % 4) {
			case 0:
				ACC_row = val & 0x000f;
				break;
			case 1:
				ACC_row = val & 0x00f0;
				ACC_row = ACC_row / 16;
				break;
			case 2:
				ACC_row = val & 0x0f00;
				ACC_row = ACC_row / 256;
				break;
			case 3:
				ACC_row = val & 0xf000;
				ACC_row = ACC_row / 4096;
				break;
			default :
				ACC_row = val & 0x000f;
				Serial.println("ACC OCC_row->this default can not!!");
				break;
		}
       
		if ( ACC_row > 7) ACC_row -=16;
 
		for (col=0; col<MLX_90640_COL; col++) {
			
			#ifdef DISPLAY_SENSITIVITY_PER_PIXEL
				Serial.println("Restore_SENSITIVITY_param for pix[" + String(row) + "][" + String(col) + "]");
				Serial.println("row address[" + String(row) + "] [" + String(col) + "] = " + String(adr + 0x2400) + " , content :" + String(val));
				Serial.println("ACC_row[" + String(row) + "] = " + String(ACC_row));         
			#endif
			
			// ACC_col
			adr1 = (((int)col / 4) + 0x2428) - 0x2400;
			val = ((int)EEPROMData[adr1*2])*256+(int)EEPROMData[adr1*2+1];
			
			switch(col % 4) {
				case 0:
					ACC_col = val & 0x000f;
					break;
				case 1:
					ACC_col = val & 0x00f0;
					ACC_col = ACC_col / 16;
					break;
				case 2:
					ACC_col = val & 0x0f00;
					ACC_col = ACC_col / 256;
					break;
				case 3:
					ACC_col = val & 0xf000;
					ACC_col = ACC_col / 4096;
					break;
				default :
					ACC_col = val & 0x000f;
					Serial.println("ACC_col->this default can not!!");
					break;
			}
        
			if ( ACC_col > 7) ACC_col -= 16;
        
			adr2 = ((row) * 32 + (col) + 0x2440) - 0x2400;   
			
			val = ((int)EEPROMData[adr2*2])*256+(int)EEPROMData[adr2*2+1];
			alpha_pixel_row_col = val & 0x03f0;
        
			alpha_pixel_row_col =  alpha_pixel_row_col / 4;
			if ( alpha_pixel_row_col > 31) alpha_pixel_row_col -= 64; 
			
			Alpha[row][col] = (float)ACC_row * (float)pow(2.0, (double)ACC_scale_row) + (float)ACC_col * (float)pow(2.0, (double)ACC_scale_col);
			Alpha[row][col] = Alpha[row][col] + (float)pow(2.0, (double)ACC_scale_remnant) * (float)alpha_pixel_row_col;
			Alpha[row][col] = (Alpha[row][col] + (float)Alpha_ref) / (float)pow(2.0, (double)Alpha_scale);
         
			#ifdef DISPLAY_SENSITIVITY_PER_PIXEL
				Serial.println("alpha_pixel_row_col["+ String(row) + "][" + String(col) + "] = " + String(alpha_pixel_row_col)); 
				Serial.println("ACC_row : " + String(ACC_row, 15));
				Serial.println("ACC_scale_row : " + String(ACC_scale_row, 15));
				Serial.println("ACC_row : " + String(ACC_row, 15));
				Serial.println("ACC_scale_col : " + String(ACC_scale_col, 15));
				Serial.println("ACC_col : " + String(ACC_col, 15));
				Serial.println("ACC_scale_remnant : " + String(ACC_scale_remnant, 15));
				Serial.println("Alpha_ref : " + String(Alpha_ref, 15));
				Serial.println("Alpha_scale : " + String(Alpha_scale, 15));
				Serial.println("Alpha[" + String(row) + "][" + String(col) + "] = " + String(Alpha[row][col], 15));
			#endif
        }
    }
}

// not finisched yet
void MLX90640::restoreRESOLUTION(){
	int val;
	
	val = ((int)EEPROMData[(0x2438-0x2400)*2])*256+(int)EEPROMData[(0x2438-0x2400)*2+1];
    RES_EE = (val & 0x3000) / 4096;
	
	#ifdef DISPLAY_RESTORE_DEBUG
		Serial.println("RES_EE=" + String(RES_EE));
    #endif
}