
#include "WiFi.h"
#include "esp_now.h"
#include <mpu6050_FastAngles.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>


//making an object for the mpu
mpu6050_FastAngles sens;


int mot1= 8;
int mot2= 10;
int mot3=4;
int mot4= 3;



int mot1_th;
int mot2_th;
int mot3_th;
int mot4_th;
 



uint8_t broadcastAddress[] = {0xFC, 0xF5, 0xC4, 0x01, 0x55, 0x7C};

typedef struct {
  int com;
  int roll;
  int pitch;
  int yaw;
  int thrust;
  float p;
  float i;
  float d;
  char arm='d';
}data_construct;
data_construct data;
data_construct send_data;


//deafult thing .. i didnt make it
esp_now_peer_info_t peerInfo;

typedef struct  {
int roll_error;
int pitch_error;
int yaw_error;
float actual_roll;
float actual_pitch;
float actual_yaw;
float roll_intigrator;
float pitch_intigrator;
float yaw_intigrator;
int prev_roll_error;
int prev_pitch_error;
int prev_yaw_error;
float roll_pid;
float pitch_pid;
float yaw_pid;
}  calc;
float bat_vol;


float kp=0;
float ki=0;
float kd= 0;
float y_ki,y_kp,y_kd;
calc c;
float dt=0.005;
unsigned long t, time_step;
float kd_dt;
float ki_dt;
unsigned long last_time;
int interval=0;
float emg_thrust;
const char arm_='a';
const char dis_arm='d';
const int freq=10000;


void restartMPU6050() {
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);            
  Wire.write(0x80);            
  Wire.endTransmission(true); 
  sens.begin(MPU_MODE_500); 

                  
}




float movingrate(float new_val) {
  static float buffer[2] = {0};
  buffer[1]= (new_val- buffer[0])/0.005;
  buffer[0]=new_val;
  
  return buffer[1] ;
}





void on_callback(const esp_now_recv_info_t *recv_info,const uint8_t *incom,int size){

  t=millis();
  memcpy(&data,incom,sizeof(data));
  esp_now_send(broadcastAddress,(uint8_t *)&send_data, sizeof(send_data));

  //Serial.println(data.message);

}


float battery_vol(){
  int vol= analogRead(2);
  bat_vol=(vol*0.00139860139)+0.2;
 return bat_vol;
}


void setup(){
  //preventing form un_precicented occurance of the run of the motors
  analogWrite(10,0);
  analogWrite(3,0);
  analogWrite(4,0);
  analogWrite(8,0);

  //ledc functions for better resolution and low cutteny consuption 
  ledcAttach(mot1,freq,12);
  ledcAttach(mot2,freq,12);
  ledcAttach(mot3,freq,12);
  ledcAttach(mot4,freq,12);

  ledcWrite(mot1,0);
  ledcWrite(mot2,0);
  ledcWrite(mot3,0);
  ledcWrite(mot4,0);

  //set up the wire for digital low pass filter
  Wire.begin();
  


  //resolutons
  /*analogWriteResolution(10,10);
  analogWriteResolution(3,10);
  analogWriteResolution(4,10);
  analogWriteResolution(8,10);*/
  //analogReadResolution(2,10);
  Serial.begin(115200);
  //initializing the mpu
  sens.begin(MPU_MODE_500);
  //if the calibration is not loaded then
  if(!sens.loadCalibration()){
    sens.calibrateGyro();
    sens.saveCalibration();
  }
  sens.setKalmanQangle(0.005);
  sens.setKalmanQbias(0.007);
  sens.setKalmanRmeasure(0.015);

  //digital low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x02);
  Wire.endTransmission(true);
  //setting the kalman biases



  
  delay(100);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setChannel(4);
  delay(10);
  if (esp_now_init() != ESP_OK) {

    Serial.println("Error initializing ESP-NOW");

    return;
  } 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 4;
  peerInfo.encrypt = false;

  Serial.println(WiFi.macAddress());
  Serial.println(xPortGetCoreID());
  Serial.println(sizeof(data));

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }
  //callback interupt
  esp_now_register_recv_cb(on_callback);
  
  //precalculation for better processing

  kd_dt= data.d/dt;
  ki_dt= data.i*dt;
  kp=data.p;

  while(battery_vol()<3.5){
    Serial.println(bat_vol);

  }
  Serial.println(bat_vol);
  data.com=0;

  y_kp=2;
  y_ki=0.1;
  y_kd=0.4;
  
  
} 


void pid_calculation(){
  


  c.roll_error=data.roll-c.actual_roll;
  c.pitch_error=data.pitch-c.actual_pitch;
  c.yaw_error= data.yaw-c.actual_yaw;
  if(abs(c.roll_error)<7){
    c.roll_intigrator+=c.roll_error*ki_dt;
  }

  if(abs(c.pitch_error)<7){
    c.pitch_intigrator+= c.pitch_error*ki_dt;
  }
  if(abs(c.yaw_error)<10){
    c.yaw_intigrator+= c.yaw_error*y_ki*dt;
  }

  

 /*c.roll_intigrator=constrain(c.roll_intigrator,-63,63);
  c.pitch_intigrator=constrain(c.pitch_intigrator,-63,63);
  c.yaw_intigrator=constrain(c.yaw_intigrator,-63,63);
  */

  c.roll_pid= kp*(c.roll_error)+  kd_dt*(c.roll_error-c.prev_roll_error)+ c.roll_intigrator;
  c.prev_roll_error= c.roll_error;

  c.pitch_pid= kp*(c.pitch_error)+ kd_dt*(c.pitch_error-c.prev_pitch_error)+c.pitch_intigrator;
  c.prev_pitch_error= c.pitch_error;

  c.yaw_pid= y_kp*(c.yaw_error) + y_kd*dt*(c.yaw_error-c.prev_yaw_error)+ c.yaw_intigrator;
  c.prev_yaw_error= c.yaw_error; 
  c.roll_pid=constrain(c.roll_pid,-63,63);
  c.pitch_pid= constrain(c.pitch_pid,-63,63);
  c.yaw_pid=constrain(c.yaw_pid,-20,20);


  if(data.thrust>30){
  mot1_th= data.thrust -c.roll_pid -c.pitch_pid - c.yaw_pid;
  mot2_th= data.thrust - c.roll_pid + c.pitch_pid + c.yaw_pid;
  mot3_th = data.thrust + c.roll_pid + c.pitch_pid - c.yaw_pid+5;
  mot4_th= data.thrust + c.roll_pid -c.pitch_pid+ c.yaw_pid+5;
  }else{
    mot1_th=data.thrust;
    mot2_th=data.thrust;
    mot3_th=data.thrust;
    mot4_th=data.thrust;
  }
  mot1_th=mot1_th;
  mot2_th= mot2_th;
  mot3_th=mot3_th;
  mot4_th=mot4_th;
}
  
void loop(){
  kp=data.p;
  kd= data.d;
  ki=data.i;
  send_data.d= dt;
  /*if (dt < 0.004) dt = 0.004;
  if (dt > 0.006) dt = 0.006;*/
  ki_dt= ki*dt;
  kd_dt= data.d/dt;
  ki_dt= data.i*dt;
  

  //assigning the actual roll pitch and yaw
  c.actual_pitch= sens.getAngle('Y',KALMAN);
  c.actual_roll= sens.getAngle('X',KALMAN);
  
  c.actual_yaw=movingrate(sens.getAngle('Z',KALMAN));
  if(c.actual_pitch>1000){
    restartMPU6050();
    sens.resetSensor();
    sens.loadCalibration();
    c.actual_pitch= data.roll-c.pitch_error;

  }
  send_data.roll=c.actual_roll;
  send_data.pitch=c.actual_pitch;
  send_data.yaw=c.actual_yaw;
  //send_data.com= dt;
  
  pid_calculation();
 
if(battery_vol()>3.5f){
switch(data.arm){
  case arm_:
       if(last_time<500){



  
         
           ledcWrite(mot1,mot1_th);
           ledcWrite(mot2,mot2_th);
           ledcWrite(mot3,mot3_th);
           ledcWrite(mot4,mot4_th);
           //Serial.println("the motors were given the input");
           emg_thrust=data.thrust;
      
      }else{
        
        emg_thrust= emg_thrust-0.5;
        delay(1);
        emg_thrust= constrain(emg_thrust,0,150);
        data.roll=0;
        data.pitch=0;
        data.yaw=0;
        if(data.arm=='a'){
        ledcWrite(mot1,emg_thrust);
        ledcWrite(mot2,emg_thrust);
        ledcWrite(mot3,emg_thrust);
        ledcWrite(mot4,emg_thrust);
        }
        Serial.println("emg_thrust on");

      }break;
  case dis_arm:
       
          
         ledcWrite(mot1,0);
         ledcWrite(mot2,0);
         ledcWrite(mot3,0);
         ledcWrite(mot4,0);
       break;
}} else{

   ledcWrite(mot1,0);
   ledcWrite(mot2,0);
   ledcWrite(mot3,0);
   ledcWrite(mot4,0);
}


  
//battery_vol();

  
  /*Serial.print(c.actual_pitch);
  Serial.print(" ,");
  Serial.print(c.actual_roll);
   Serial.print(" ,");
  Serial.print(c.actual_yaw);
  Serial.print(" ,");
  Serial.print(c.yaw_error);
  Serial.print(" ,");
  Serial.println(data.arm);*/
  //loop_time= millis();
 /* Serial.print(mot1_th);
  Serial.print(" ,");
  Serial.print(mot2_th);
   Serial.print(" ,");
  Serial.print(mot3_th);
  Serial.print(" ,");
  Serial.print(mot4_th);
  Serial.print(" ,");*/
  //Serial.println(data.arm);
  

    
  last_time=millis()-t;
  //dt=0.005;
  dt=(millis()-time_step)/1000.0f;
  time_step= millis();
  

}



 
