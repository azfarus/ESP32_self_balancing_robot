

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <ArduinoJson.h>


#define MSG_BUFFER_SIZE (128)




MPU6050 mpu(Wire);
AccelStepper motor_right(AccelStepper::DRIVER, 12, 13);
AccelStepper motor_left(AccelStepper::DRIVER, 23, 27);

const char* ssid = "drazam_plus";
const char* password = "fallguys";
const char* mqtt_server = "192.168.1.72";
const char* subscribe = "inTopic";
const char* publish = "outTopic";
const char* pid_topic = "pidTopic";
char msg[MSG_BUFFER_SIZE];

WiFiClient espClient;
PubSubClient client(espClient);
StaticJsonDocument<300> doc;


long long timer = 0;
double accX, accY, accZ;
double jerkX, jerkY, jerkZ;
double angle_x, acc_calib, acc_mag, acc_angle_x, gyro_angle_x;
double k;
double delta_t, sig_val;
double turn = 0;

//pid variables

double Kp = 127.0, Ki = 4.7, Kd = 0.0295;
double increment = 0.1;
bool go = false;
double setpoint = 0 , error , pid_output , i_sum = 0 , lasterror , d_error ,move_setpoint;


hw_timer_t* new_timer = NULL;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  switch (payload[0]) {
    case 'P':
      Kp += increment;
      break;
    case 'p':
      Kp -= increment;
      break;
    case 'I':
      Ki += increment;
      break;
    case 'i':
      Ki -= increment;
      break;
    case 'D':
      Kd += increment;
      break;
    case 'd':
      Kd -= increment;
      break;
    case 'S':
      increment *= 10;
      break;
    case 's':
      increment /= 10;
      break;
    case 'L':
      //turn = -abs(turn);
      turn-=200;
      break;
    case 'R':
      //turn = abs(turn);
      turn+=200;
      break;
    case 'F':
      //turn = -abs(turn);
      move_setpoint-=1.5;
      break;
    case 'B':
      //turn = abs(turn);
      move_setpoint+=1.5;
      break;
    case 'O':
      turn =0;
      move_setpoint = 0;
      break;
    case 'C':
      setpoint = angle_x;
      go = !go;
      pid_output = 0;
      i_sum = 0;
      turn = 0;
      move_setpoint = 0;
    default:
      break;
  }

  doc["kp"] = abs(Kp) < .0001 ? 0.0 : Kp;
  doc["ki"] = abs(Ki) < .0001 ? 0.0 : Ki;
  doc["kd"] = abs(Kd) < .0001 ? 0.0 : Kd;
  doc["inc"] = increment;

  serializeJson(doc, msg);
  Serial.println(msg);
  Serial.println(turn);
  client.publish(pid_topic, msg);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "Balance_bot";

    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(publish, "hello world");
      // ... and resubscribe
      client.subscribe(subscribe);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(portTICK_PERIOD_MS * 5000);
    }
  }
}

void refresh(void* x) {
  while (true) {
    //Serial.printf("%f\t%f\t%f\t%f\n" , angle_x , gyro_angle_x , delta_t ,k);

    if (!client.connected()) {
      reconnect();
    }
    client.loop();


    snprintf(msg, MSG_BUFFER_SIZE, "%.2f", setpoint);
    client.publish(publish, msg);
    snprintf(msg, MSG_BUFFER_SIZE, "%.2f", angle_x);
    client.publish("angleTopic", msg);

    vTaskDelay(portTICK_PERIOD_MS * 50);
  }
}

double sigmoid(double x, double b, double c) {  //b has to be negative
  return pow(2.71828, b * (x - c) * (x - c));
}



void setup() {

  Wire.begin();
  Wire.setClock(1600000);
  byte status = mpu.begin();

  Serial.begin(115200);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.printf("%f %f %f\n" , Kp , Ki , Kd);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // gyro and accelero
  Serial.println("Done!\n");


  xTaskCreatePinnedToCore(refresh, "refresh", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);

  pinMode(2 , OUTPUT);
  for(int i = 0 ; i < 16 ; i++){
    digitalWrite(2 , !digitalRead(2));
    delay(500);
  }


  for (int i = 0; i < 1000; i++) {
    mpu.update();
    accX += mpu.getAccX();
    accY += mpu.getAccY();
    accZ += mpu.getAccZ();
  }

  accX /= 1000;
  accY /= 1000;
  accZ /= 1000;



  acc_calib = accX * accX + accY * accY + accZ * accZ;

  for(int i = 0 ; i < 16 ; i++){
    digitalWrite(2 , !digitalRead(2));
    delay(100);
  }

  motor_right.setMaxSpeed(150000000000);
  motor_left.setMaxSpeed(150000000000);
  motor_left.setAcceleration(2000000000000);
  motor_right.setAcceleration(2000000000000);
  timer = micros();
}

void loop() {

  double strt = micros();
  mpu.update();


  delta_t = micros() - timer;
  angle_x += mpu.getGyroX() * delta_t * 1e-6;
  gyro_angle_x += mpu.getGyroX() * delta_t * 1e-6;
  timer = micros();

  acc_angle_x = atan2(mpu.getAccY(), mpu.getAccZ()) * RAD_2_DEG;
  k = 0.05;
  jerkX = (mpu.getAccX() - accX);
  jerkY = (mpu.getAccY() - accY);
  jerkZ = (mpu.getAccZ() - accZ);
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();
  acc_mag = accX * accX + accY * accY + accZ * accZ;


  k = k * sigmoid(acc_mag, -450, acc_calib) * sigmoid(jerkX + jerkY + jerkZ, -30000, 0);
  angle_x = angle_x * (1 - k) + acc_angle_x * k;

  error = angle_x - setpoint - move_setpoint;
  d_error = error - lasterror;
  lasterror = error;
  i_sum+= Ki*error;

  pid_output = Kp*error + i_sum - Kd*d_error ;

  if(pid_output < -3250){
    pid_output= -3250;
  }
  if(pid_output > 3250){
    pid_output= 3250;
  }

  

  motor_right.setSpeed(-(pid_output+turn)*go);
  motor_left.run();
  motor_left.setSpeed((pid_output-turn)*go);
  motor_right.run();


  
  
  while (micros() - strt < 992)
    ;
}
