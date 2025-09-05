#include "WiFi.h"
#include "esp_now.h"
#include <mpu6050_FastAngles.h>
#include <ArduinoJson.h>

mpu6050_FastAngles mpu;


uint8_t broadcastAddress[] = {0x98, 0x3D, 0xAE, 0xA9, 0xE8, 0xB0};

typedef struct {
  int  com;
  int roll=0;
  int pitch=0;
  int yaw=0;
  int thrust=0;
  float p=0; 
  float i=0;
  float d=0;
  char arm='d';
}data_construct;
data_construct data;
data_construct recv_data;
//data_construct recv;
uint8_t *point= (uint8_t *)&data;

//peer class not mine 
esp_now_peer_info_t peerInfo = {};
unsigned long time_=0;
unsigned long time_step;
unsigned long pre_time=0;
int pre_thrust;
int touch;
int joyX = 0, joyY = 0;
 int x;
    int y;
unsigned long last_time;
float dt=0.015;

/// this section is for the twiddle optimization algorithm
// PID gains


// Twiddle parameters
float p[3] = {3, 0.15, 1.5};       // Kp, Ki, Kd initial values
float dp[3] = {0.00, 0.00, 0.00}; // increments for Kp, Ki, Kd

float best_error = 1e6;
unsigned long trial_start_time = 0;
const unsigned long trial_duration = 3000;  // trial time in ms

// Twiddle states
enum TwiddleState { INIT, INCREASE, DECREASE, WAITING };
TwiddleState twiddle_state = INIT;

int param_index = 0;

// For error accumulation
float total_error = 0;
int sample_count = 0;

void resetError() {
  total_error = 0;
  sample_count = 0;
}
void accumulateError(float actual_roll, float desired_roll) {
  float err = actual_roll - desired_roll;
  total_error += err * err;
  sample_count++;
}

// the twiddle function
void twiddleStep(float desired_roll = 0) {
  unsigned long now = millis();

  // Send current PID params to flight controller
  data.p = p[0];
  data.i = p[1];
  data.d = p[2];

  if (twiddle_state == INIT) {
    twiddle_state = INCREASE;
    p[param_index] += dp[param_index];
    resetError();
    trial_start_time = now;
  }

 
  if (now - trial_start_time < trial_duration) {
    // still collecting error...
    return;
  }

  // Calculate average error
  float avg_error = (sample_count > 0) ? total_error / sample_count : 1e6;

  if (twiddle_state == INCREASE) {
    if (avg_error < best_error) {
      best_error = avg_error;
      dp[param_index] *= 1.1;
      param_index = (param_index + 1) % 3;
      p[param_index] += dp[param_index];
      resetError();
      trial_start_time = now;
    
    } else {
      
      p[param_index] -= 2 * dp[param_index];
      twiddle_state = DECREASE;
      resetError();
      trial_start_time = now;
    }

  } else if (twiddle_state == DECREASE) {
    if (avg_error < best_error) {
      best_error = avg_error;
      dp[param_index] *= 1.1;
    } else {
      p[param_index] += dp[param_index];
      dp[param_index] *= 0.9;
    }
    
    param_index = (param_index + 1) % 3;
    p[param_index] += dp[param_index];
    twiddle_state = INCREASE;
    resetError();
    trial_start_time = now;
  }
}


//callback function


int pin_state;
int last_state;

void on_callback(const esp_now_recv_info_t *recv_info,const uint8_t *incom,int size){

  memcpy(&recv_data,incom,sizeof(recv_data));
  accumulateError(recv_data.roll, data.roll) ;


  //Serial.println(data.message);

}

void setup(){
  Serial.begin(115200);
  pinMode(25,INPUT);
  data.arm='d';
  
  while(!WiFi.mode(WIFI_MODE_STA)){
  delay(10);
  }
  WiFi.setChannel(4);
  if (esp_now_init() != ESP_OK) {

    Serial.println("Error initializing ESP-NOW");

   return;
  }
  Serial.printf(esp_now_init() == ESP_OK ? "INITALIZED" : "COULD NOT BE INITALIZED");
  Serial.println(WiFi.macAddress());
  Serial.println(xPortGetCoreID());


 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.ifidx=WIFI_IF_STA;
  peerInfo.channel = 4;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  mpu.begin();

  
  Serial.println(WiFi.macAddress());
  data.thrust=0;
  data.roll=0;
  data.pitch=0;
  data.p=0;
  esp_now_register_recv_cb(on_callback);
  
  
  
  
    
  

  
}
 
void loop(){
 
  

  int yaw=0;
  arm_logic();
  
 
  
  data.yaw= constrain(yaw,-180,180);
  data.thrust=analogRead(39)*0.0625*0.6+pre_thrust*0.4;
  pre_thrust=data.thrust;


  
  
  //Serial.printf("this is the stupid message: %f\n",data.roll);
  //Serial.println(data.yaw);
    // Check if serial data is available

    twiddleStep();
    StaticJsonDocument<128> txDoc;
  txDoc["type"] = "thrust";
  txDoc["value"] = data.thrust;
  //serializeJson(txDoc, Serial);
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    // Parse JSON
    /*StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, input);
    if (err) {
      //Serial.print("JSON error: ");
      //Serial.println(err.c_str());
      return;
    }

    // Type field: pid or joystick
    String type = doc["type"];

    if (type == "pid") {
      String param = doc["param"];
      float value = doc["value"];

      if (param == "kp") data.p= value;
      else if (param == "ki") data.d = value;
      else if (param == "kd") data.i= value;

    }

    else if (type == "joystick") {
      joyX = doc["x"];
      joyY = doc["y"];

  
    }
    else{
       String str=Serial.readStringUntil('\n');
       data.pitch= str.toFloat();
    }*/
  }

  Serial.print(data.arm);
    Serial.print(" , ");
  Serial.print(data.p);
    Serial.print(" , ");
    Serial.print(data.i);
  Serial.print(" , ");
    Serial.print(data.d);

      Serial.print(" , ");
  Serial.print(recv_data.roll); 
    Serial.print(" , ");
    Serial.print(recv_data.pitch);
  Serial.print(" , ");
    Serial.print(recv_data.yaw);
    Serial.print(" , ");
    Serial.print(recv_data.d);
    Serial.print(" , ");
  
 // Serial.printf(" ,kp: %.2f ,kd: %.2f ,ki: %.2f , ",data.p,data.d,data.i);
  Serial.println(data.thrust);
  

  dt=(micros()-time_step)/1000000.0f;
  if(dt>0.014){
  esp_now_send(broadcastAddress,(uint8_t *)&data, sizeof(data));
  time_step= micros();
  }



 
}

void toggle(){
   pin_state= digitalRead(25);
   delay(30);
  if(pin_state && !last_state){
   touch=1;
  }else{
    touch=0;
  }
  last_state= pin_state;
}
 

void arm_logic(){
    int i=0;
   
    x=analogRead(33);
    y=analogRead(32);
    while(x<20&&y<30&&data.arm=='d'){
      i++;
      if(i>20000){
        data.arm='a';
       break;
      }
      x=analogRead(33);
      y=analogRead(32);
      data.thrust=0;
      
    }while(x<30&&y>3800&&data.arm=='a'){
      i++;  if(i>10000){
        data.arm='d';
       break;
      }
      data.yaw=0;
      x=analogRead(33);
      y=analogRead(32);
      
      esp_now_send(broadcastAddress,(uint8_t *)&data, sizeof(data));

    }


}