
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "rplidar_driver.h"
#include "../../include/lidar/rplidar_cmd.h"
RPLidar rplidar;
#define RPLIDAR_MOTOR 5
#define size_tab 360
#define MAX_POINT_COUNT size_tab
#define MAX_DISTANCE 1000
#include "ObstacleHandler.h"
#include "CommunicationHandler.h"

ObstacleHandler obstacleHandler;
CommunicationHandler communicationHandler(&obstacleHandler);


unsigned long time_prev_acquisition;        // Temps auquel la précédente acquisition a été réalisée.
unsigned long delay_max_acquisition = 100;  // Temps maximum pour réaliser l'acquisition de detect_tab[]

byte score = 0;
volatile bool updated = false;

void onDataRequest();
void onReceive(int);
void printInfo();
void lidar_traitement();

uint data_index = 0;
unsigned long init_time;

char report[80];

LiquidCrystal_I2C lcd(0x20, 20, 4);


void setup() {
    Serial.begin(250000);
    Wire.begin(14);
    rplidar.begin();
    Wire.onRequest(onDataRequest);
    Wire.onReceive(onReceive);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    delay(1000);
    Wire.end();
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("     My name is      ");
    lcd.setCursor(0, 1);
    lcd.print("    BlueNightBot    ");
    lcd.setCursor(0, 2);
    lcd.print("    My score is     ");
    lcd.setCursor(0, 3);
    lcd.print("         0          ");
    Wire.end();
    Wire.begin(14);
}

void onDataRequest(){
  Wire.write(communicationHandler.getValue());
}

void onReceive(int howMany){
  score = Wire.read();
  updated = true;
}

void printInfo(){
    rplidar_response_device_info_t info;
    rplidar.getDeviceInfo(info);

    snprintf(report, sizeof(report), "model: %d firmware: %d.%d", info.model, info.firmware_version/256, info.firmware_version%256);
    Serial.println(report);
    delay(1000);
}
void printSampleDuration(){
    rplidar_response_sample_rate_t sampleInfo;
    rplidar.getSampleDuration_uS(sampleInfo);

    snprintf(report, sizeof(report), "TStandard: %d[us] TExpress: %d[us]", sampleInfo.std_sample_duration_us, sampleInfo.express_sample_duration_us);
    delay(1000);
}
void loop() {
    if (!rplidar.isScanning()){
        Serial.println("NOT scanning");
        rplidar.startScanExpress(true, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS);
        digitalWrite(RPLIDAR_MOTOR, 120); // turn on the motor
        delay(10);
        init_time = millis();
        time_prev_acquisition = millis();
    }
    else{
      //Serial.println("Rework");
        // loop needs to be send called every loop
        rplidar.loopScanExpressData();

        // create object to hold received data for processing
        size_t nodeCount = 512;
        rplidar_response_measurement_node_hq_t nodes[nodeCount];
         // variable will be set to number of received measurement by reference
        u_result ans = rplidar.grabScanExpressData(nodes, nodeCount);
        if (IS_OK(ans)){
            for (size_t i = 0; i < nodeCount; ++i){
                // convert to standard units
                float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1<<14);
                float distance = nodes[i].dist_mm_q2 / (1 << 2);
                if(distance <100)
                    continue;
                if(distance > MAX_DISTANCE)
                    continue;
                obstacleHandler.addPoints(angle_in_degrees, distance);
                //Serial.println(report);
                data_index++;
                ulong delta = millis() - init_time;
                if(delta > 10000){
                    Serial.println((double)data_index * 1000.0f/((double)delta));
                    data_index = 0;
                    init_time = millis();
                }
            }

        }else{
          Serial.println("Not ok");
        }
    }

    if(millis() - time_prev_acquisition > delay_max_acquisition){
        time_prev_acquisition = millis();
        obstacleHandler.evaluate();
        //obstacleHandler.printPoints();
        obstacleHandler.printObstacles();
        obstacleHandler.increaseRollingIndex();
        obstacleHandler.resetPoints();
    }

    if(updated){
      Serial.println("Waiting to update");
        Wire.end();
        lcd.init();
        lcd.backlight();
        lcd.setCursor(0,0);
        lcd.print("     My name is     ");
        lcd.setCursor(0, 1);
        lcd.print("    BlueNightBot    ");
        lcd.setCursor(0, 2);
        lcd.print("    My score is     ");
        lcd.setCursor(9, 3);
        lcd.print(score);
        Wire.end();
        Wire.begin(14);
        updated = false;
      

    }

}
