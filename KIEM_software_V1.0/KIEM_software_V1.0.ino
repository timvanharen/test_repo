/*
 * Title    : Hotspot_recognition_v2.0_MLX 
 * Version  : 2.0
 * Author   : Tim van Haren (timvanharen@live.com)
 * Date     : 2-11-2018
 * Project  : KIEM
 * 
 * Discription:
 * 
 * clone espressif hardware from git
 * 
 * This software is used to detect the amount of humans in a room by measuring a infrared image and detecting human body temperature as blobs with blobdetection. 
 * This image is captured using the MLX90640 FIR sensor array. The sensor has a 32x24 pixels resolution and a angle of 90 degrees horizontaly and 120 vertically.
 * The sensor communicates with I2C, and is equipped with EEPROM, RAM and ROM. the MLX90640 can be configurated by setting specific bytes in the configuration register at address()
 * To control measurements and check when a measurement has completed the status register is accessed.
 * When a measurement is completed, the ESP32 requests the amount of bytes of the full RAM register. It is IMPORTANT that your I2C libraries can handle inputbuffers of 1664 bytes.
 * This is important because all the data has to be read at once. The standard inputbuffer length of the espressif modules is 256, this problem has caused great delay so it has to be taken seriously.
 * The measured infrared matrix alone will not be accurate enough to detect humans reliably. The resolution is quite low and without calibration the output is too noisy.
 * Although this is a limiting factor, there are many image processing techniques for creating a clearer image.
 * In order to get a reliable detection 5 mechanisms are used:
 *      Mean filtering.
 *      static offset correction.
 *      contrast inscreasing.
 *      Rescaling.
 *      thresholding.
 * 
 * 
 * The libraries that are used are:
 *    Wire.h             -    This library enables I2C communication
 *    MLX90640.h         -    This library uses I2C communication to read the temperature matrix measured by the MLX90640 Far Infrared Temperature Sensor Array
 *    Blob_detection.h   -    This library contains algoritms to detect multiple blobs in a small resolution images. 
 *    
 *    Docucument the system in a report. ( don't forget to set the I2C buffer length to 1664.
 */
 
 /* Includes */
#include <Wire.h>
#include "MLX90640.h"
#include "Blob_detection.h"

/* Defines (Defined by user) */
// Sensor config
#define MLX_POWER_SWITCH_PIN                17
#define MLX_SAMPLE_FREQ                     32                          // Available rates: 0.5, 1, 2, 4, 8, 16, 32, 64
#define MLX_MEASUREMENT_PATTERN             1                           // 0 is interleaved mode, 1 is chess pattern.
#define MLX_ADC_RESOLUTION                  18                          // Available resolutions: 16, 17, 18, 19
#define MLX_EMISSIVITY_COMPENSATION         1                           // User defined emissivity compenstation value.
#define MLX_REFRESH_RATE                    1000/MLX_SAMPLE_FREQ        // Wait time for the sensor between measurements

/* By setting these defines specific parts of this application are activated. 
      Display functions are used for debugging.
      Send functions are for communication with a python script, running on the computer thats runnig this.
      The calibration and compensate functions are used to compensate for any offset in corners.
      To calibrate the sensor:
        1. Enable COMPENSATE_CORNER_OFFSET
        2. Place an object that has an even temperature distribution upon its surface.
        3. Upload the software to the microcontroller.
        4. Type 'C' in the serial moniter.
*/
#define DISPLAY_MATRIX                     1
//#define DISPLAY_BLOB_DATA                  1

//#define SEND_MATRIX_TO_DISPLAY              1
//#define SEND_BLOB_DATA_TO_DISPLAY           1

//#define CALIBRATE_CORNER_OFFSET             1
//#define COMPENSATE_CORNER_OFFSET            1

// Blob detection config
#define MAX_NUM_OF_BLOBS                    10
#define IR_THRESHOLD                        550
#define MIN_BLOB_SIZE                       2
#define MIN_BLOB_DISTANCE                   3

// Max and min temperature that is used in video processing. 
// This prevents that objects such as heaters or lamps mess up the temperature range. You could perhaps used a higher limit to detect fires.
#define MAX_TEMP                            4000
#define MIN_TEMP                            0

// Interpolation config
#define POINTS_IN_BETWEEN                   2

// Sleep mode config
#define TIME_TO_SLEEP                       5
#define uS_TO_S_FACTOR                      1000000

// 2D median filter config
#define KERNEL_SIZE                         2 //number describes width and height of filter kernel

/* Global typedefs */
MLX90640            MLXSensor(MLX_POWER_SWITCH_PIN, MLX_SAMPLE_FREQ, MLX_MEASUREMENT_PATTERN, MLX_ADC_RESOLUTION, MLX_EMISSIVITY_COMPENSATION);

/* Global variables */
RTC_DATA_ATTR int   bootCount = 0;  
int                 i,j,k,m,l;

int                 infraRedImage[MLX_90640_ROW][MLX_90640_COL];
signed int          cornerOffset[MLX_90640_ROW][MLX_90640_COL];

int                 maxTemp = MIN_TEMP;
int                 minTemp = MAX_TEMP;

int                 blobCounter = 0;
int                 blobsCounted = 0;
Blobs               b[50];
int                 significantBlobs[MAX_NUM_OF_BLOBS];

void setup() {
  
  //Join Serial bus
  Serial.begin(115200);
  Serial.println("Serial bus joined");
   
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  
  //Configure the wake up source
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  
  //Boot sensor
  MLXSensor.wakeupSensor(TRUE);
  Serial.println("Sensor power on");
  
  // Join wire bus
  Wire.begin();
  Serial.println("Wire bus joined");
  
  // Wait refresh rate + 20% of refreshrate on powerUp
  delay(MLX_REFRESH_RATE + MLX_REFRESH_RATE/5);
  
  // Configurate the sensor
  MLXSensor.initialize();
}

void loop() {

  Serial.println(FALSE);
  bool done = FALSE;
  bool calibrated = FALSE;
  String flag;
  
  float contrastSweep = 1;

  while(done == FALSE) {
    
    #ifdef CALIBRATE_CORNER_OFFSET
      while(calibrated == FALSE) {
        if(Serial.available() > 0) {
      
          // Read the incoming byte:
          flag = Serial.readStringUntil('\n');

          if(flag == "C") {
            // Get sensor data
            MLXSensor.capture(0);
        
            // Reset lowest and highest measured temperature.
            maxTemp = MLXSensor.getMaxTemp();
            minTemp = MLXSensor.getMinTemp();
            
            // Convert data string to temperature matrix form  
            getInfraRedImage(FALSE);
            meanFilterImage();
            #ifdef DISPLAY_MATRIX
              // Print sensor data
              Serial.println("calibration measurement");
              Serial.println("Mintemp: " + String(minTemp, DEC));
              Serial.println("Maxtemp: " + String(maxTemp, DEC));
              for(i=0;i<MLX_90640_ROW;i++) {
                for(j=0;j<MLX_90640_COL;j++) {
                  Serial.print(infraRedImage[i][j], DEC);
                  Serial.print(", ");
                }
                Serial.println("");
              }
              Serial.println("");
            #endif
            calibrate_corner_offset();
            calibrated = TRUE;
            Serial.print("R");
            } 
        } else {
          Serial.print("C");
          delay(10);
        }
      }
    #endif
    
    // Get sensor data
    MLXSensor.capture(0);

    // Reset lowest and highest measured temperature.
    maxTemp = MLXSensor.getMaxTemp();
    minTemp = MLXSensor.getMinTemp();
    
    // Convert data string to temperature matrix form  
    getInfraRedImage(TRUE);

    #ifdef DISPLAY_MATRIX
      // Print sensor data
      Serial.println("Measured temperature matrix: ");
      Serial.println("Mintemp: " + String(minTemp, DEC));
      Serial.println("Maxtemp: " + String(maxTemp, DEC));
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(infraRedImage[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    #endif

    // Filter image to reduce noise and smooth hotspots
    meanFilterImage();
    findMinMaxTemp();
    
    #ifdef DISPLAY_MATRIX
      //print sensor data
      Serial.println("Filtered temperature matrix: ");
      Serial.println("Mintemp: " + String(minTemp, DEC));
      Serial.println("Maxtemp: " + String(maxTemp, DEC));
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(infraRedImage[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    #endif

   
    compensate_corner_offset();
    #ifdef DISPLAY_MATRIX
      // Print sensor data
      Serial.println("Corner offset matrix: ");
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(cornerOffset[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    
      Serial.println("Measured temperature matrix: ");
      Serial.println("Mintemp: " + String(minTemp, DEC));
      Serial.println("Maxtemp: " + String(maxTemp, DEC));
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(infraRedImage[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    #endif
    
    //Make hotpoints hotter and cold points colder.
    increaseContrast(contrastSweep, 100);
    //contrastSweep += 0.01;
    findMinMaxTemp();
    #ifdef DISPLAY_MATRIX
      //print sensor data
      Serial.println("Contrast correction temperature matrix: ");
      Serial.println("Mintemp: " + String(minTemp, DEC));
      Serial.println("Maxtemp: " + String(maxTemp, DEC));
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(infraRedImage[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    #endif
    
    //Find hotspots by detecting blobs of hotspots in image
    findBlobs();
    
    //rescale the IRImage for transportation
    rescaleIRImage();
    
    findMinMaxTemp();
    #ifdef DISPLAY_MATRIX
      //print sensor data
      Serial.println("Rescaled temperature matrix: ");
      Serial.println("Mintemp: " + String(minTemp, DEC));
      Serial.println("Maxtemp: " + String(maxTemp, DEC));
      for(i=0;i<MLX_90640_ROW;i++) {
        for(j=0;j<MLX_90640_COL;j++) {
          Serial.print(infraRedImage[i][j], DEC);
          Serial.print(", ");
        }
        Serial.println("");
      }
      Serial.println("");
    #endif
    
    #ifdef SEND_MATRIX_TO_DISPLAY
       //Send start command
      Serial.print("R");
       
      //Check if a start command is acknowledged
      if(Serial.available() > 0) {
      
        // Read the incoming byte:
        flag = Serial.readStringUntil('\n');
        
        // If the start command is equal to 999 send acknoledge
        if(flag == "R") {
            flag == "";
            Serial.print("S");
            tempMatrixToSerial();
            
            //sendMinMaxTemp();
            
            #ifdef SEND_BLOB_DATA_TO_DISPLAY
              if(blobsCounted > 0) {
                Serial.print("B");
                sendBlobData();
              }
            #endif
            Serial.print("P");
        } else if(flag == "F") {
            flag = "";
            serialFlush();
            Serial.print("A");
        } else if(flag == "X")
          done = TRUE;
      } else {
        delay(10);
      }
    #endif
  }
  Serial.println("Measurement and analysis complete, going to sleep now");
  esp_deep_sleep_start();
}

void serialFlush() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}

void getInfraRedImage(int raw) {
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      infraRedImage[i][j] = MLXSensor.getTempMatrix(i,j);

      if(raw == TRUE) {
        infraRedImage[i][j] -= minTemp; 
        
        if(infraRedImage[i][j] > MAX_TEMP)
          infraRedImage[i][j] = MAX_TEMP;
        if(infraRedImage[i][j] < MIN_TEMP)
          infraRedImage[i][j] = MIN_TEMP;
      }
    }
  }
}

void calibrate_corner_offset() {
  // Ambient Temperature
  int Ta = 0;
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      Ta += infraRedImage[i][j];
    }
  }
  Ta /= (MLX_90640_ROW * MLX_90640_COL);
  Serial.println("Ta  : " + String(Ta));

  // Calculate offset per pixel.
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
        cornerOffset[i][j] = Ta - infraRedImage[i][j];
    }
  }
}
   
void compensate_corner_offset() {
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      infraRedImage[i][j] +=  cornerOffset[i][j];
    }
  }
}

void meanFilterImage() {
  long totalVal;
  
  for(i=0;i < MLX_90640_ROW;i++) {
    if(i > (MLX_90640_ROW - KERNEL_SIZE)) {
      for(j=0;j < MLX_90640_COL;j++) {
        if(j > (MLX_90640_COL - KERNEL_SIZE)) {
          totalVal=0;
          for(k=0;k < (MLX_90640_ROW - i);k++) {
            for(m=0;m < (MLX_90640_COL - j);m++) {
              totalVal = totalVal + infraRedImage[i+k][j+m];
            }
          }
          infraRedImage[i][j] = totalVal/((MLX_90640_ROW - i) * (MLX_90640_COL - j));        
        } else {
          totalVal=0;
          for(k=0;k < (MLX_90640_ROW - i);k++) {
            for(m=0;m < KERNEL_SIZE;m++) {
              totalVal = totalVal + infraRedImage[i-k][j+m];
            }
          }
          infraRedImage[i][j] = totalVal/((MLX_90640_ROW - i) * KERNEL_SIZE);
        }
      }
    } else {
      for(j=0;j < MLX_90640_COL;j++) {
        if(j > (MLX_90640_COL - KERNEL_SIZE)) {
          totalVal=0;
          for(k=0;k < KERNEL_SIZE;k++) {
            for(m=0;m < (MLX_90640_COL - j);m++) {
              totalVal = totalVal + infraRedImage[i+k][j-m];
            }
          }
          infraRedImage[i][j] = totalVal/(KERNEL_SIZE * (MLX_90640_COL - j));
        } else {
          totalVal=0;
          for(k=0;k<KERNEL_SIZE;k++) {
            for(m=0;m<KERNEL_SIZE;m++) {
              totalVal = totalVal + infraRedImage[i+k][j+m];
            }
          }
          infraRedImage[i][j] = totalVal/(KERNEL_SIZE*KERNEL_SIZE);
        }
      }
    }
  }
}

void rescaleIRImage() {
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      infraRedImage[i][j] = map(infraRedImage[i][j], minTemp, maxTemp, 1000, 9999); //rescale the infra red image
      if(infraRedImage[i][j] > 9999)
        infraRedImage[i][j] = 9999;
      if(infraRedImage[i][j] < 1000)
        infraRedImage[i][j] = 1000;
    }
  }
}

void findMinMaxTemp() {
  maxTemp = -32768;
  minTemp = 32768;

  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {      
      maxTemp = max(infraRedImage[i][j], maxTemp);    
      minTemp = min(infraRedImage[i][j], minTemp);
    }
  }
}

void tempMatrixToSerial() {
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      Serial.print(infraRedImage[i][j], DEC); //rescale the infra red image
      infraRedImage[i][j]=0;
    }
  }
}

void sendBlobData() {
  Serial.print((blobsCounted + 10), DEC);
  for(i=0;i<blobsCounted;i++) {
    Serial.print((b[significantBlobs[i]].minX() + 10), DEC);
    Serial.print((b[significantBlobs[i]].maxX() + 10), DEC);
    Serial.print((b[significantBlobs[i]].minY() + 10), DEC);
    Serial.print((b[significantBlobs[i]].maxY() + 10), DEC);
    b[significantBlobs[i]].clearBlob();
  }
}

void increaseContrast(float contrastSensivity, int contrastOffset) {
  for(i=0;i<MLX_90640_ROW;i++) {
    for(j=0;j<MLX_90640_COL;j++) {
      infraRedImage[i][j] *= infraRedImage[i][j];
      infraRedImage[i][j] /= (maxTemp * contrastSensivity - (contrastOffset * contrastSensivity));
    }
  }
}

void findBlobs() {
  
  int found = 0;
  int cnt = 0;

  blobsCounted = 0;
  
  for(i=0;i<24;i++) {
    for(j=0;j<32;j++) {
      if(infraRedImage[i][j] > IR_THRESHOLD) {
          found = 0;
          
          //add point to existing blobs
          for(k=blobsCounted;k>0;k--) {
            if(b[k].isClose(i,j) & found == 0) {
              b[k].add(i,j);
              found = 1;
              
              #ifdef DISPLAY_BLOB_DATA
                Serial.println("[" + String(i) + "][" + String(j) + "] with value : " + String(infraRedImage[i][j]));
                Serial.print("Point added to blob: " + String(k) + " size of this blob: : " + String(b[k].blobSize()) + " : ");
              #endif
            }
          }
          //make a new blob
          if(found == 0) {
            b[blobsCounted].make(i,j, MIN_BLOB_DISTANCE);
            blobsCounted++;
            
            #ifdef DISPLAY_BLOB_DATA
              Serial.print("Blob " + String(blobCounter) + " : ");
            #endif
          }       
        }
      }
    }

  //separate the big blobs from the small
  for(i=0;i<blobsCounted+1;i++) {
    if(b[i].blobSize() > MIN_BLOB_SIZE) {
      significantBlobs[cnt] = i;
      cnt++;
    } else {
      b[i].clearBlob();
    }
  }
 
  #ifdef DISPLAY_BLOB_DATA
    Serial.println("\nAmount of blobs found : " + String(blobsCounted));
    Serial.println("Significant blobs found : " + String(cnt));
  #endif
  
  blobsCounted = cnt;
}
