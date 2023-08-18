
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <Arduino.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include<HttpClient.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_ADDRESS    0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);


HardwareSerial Serial2(USART2); // XIAO ESP32 C3


byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC address of the W5300 module

const char host[]="maker.ifttt.com";
const char apikey[]="bkcLEufgjqq-26NX0uvo-j";

const char* mqtt_server = "mqtt.cloud.kaaiot.com";
uint16_t mqtt_port = 1883;

const String TOKEN = "vK4gnw9QZA";        // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = "cjdpme4tethc73eft5og-v1";  // Application version - you specify it during device provisioning

EthernetClient ethClient;
PubSubClient client(ethClient);


// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
void Pulse_oximeter( void *pvParameters );
void ECG( void *pvParameters );
void SerialRead(void *pvParameters);

int32_t spo2; //SPO2 value
int32_t heartRate; //heart rate value
float temperature;

bool flag = false; 
char receivedData = 0;


// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Ethernet.begin(mac);
  client.setServer(mqtt_server, mqtt_port);
  client.setServer(mqtt_server, 1883);
  if (!client.connected()) 
  {
    reconnect();
  }
  
  //pinMode(PA0,INPUT);
  
  /*while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }*/
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();

  /* Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
   because it is sharing a resource, such as the Serial port.
   Semaphores should only be used whilst the scheduler is running, but we can set it up here. */
  
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up three Tasks to run independently.
  xTaskCreate(
    Pulse_oximeter
    ,  (const portCHAR *)"PulseOximeter"  // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    ECG
    ,  (const portCHAR *) "E.C.G"
    ,  128  // Stack size
    ,  NULL
    ,  2 // Priority
    ,  NULL );
    
  xTaskCreate(
  SerialRead,
  (const portCHAR *)"XIAO",
  128,
  NULL,
  1,  // Priority (higher than the other tasks)
  NULL  );


  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
  
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Pulse_oximeter( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {  
        client.loop();
        MAX30102.getHeartbeatSPO2();
        vTaskDelay(10);
        spo2 = MAX30102._sHeartbeatSPO2.SPO2;
        vTaskDelay(10);
        heartRate = MAX30102._sHeartbeatSPO2.Heartbeat;
        vTaskDelay(10);
        temperature = MAX30102.getTemperature_C();
        temperature = temperature*(1.8) + 32;
        vTaskDelay(10);
        // Sending the data as JSON
        DynamicJsonDocument Pulsetelemetry(1023);
        Pulsetelemetry.createNestedObject();
        
        Pulsetelemetry[0]["temperature"] = temperature;
        Pulsetelemetry[0]["heartrate"] = heartRate;
        Pulsetelemetry[0]["sp02"] = spo2;

        /*if (heartRate<60){
          Alert("BPM",String(heartRate));
        }
        if (spo2<80){
          Alert("SPO2", String(spo2));
        }*/
  
        String topic1 = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
        String topic2 = "kp1/" + APP_VERSION + "/ecr/" + TOKEN + "/json";
        
        client.publish(topic1.c_str(), Pulsetelemetry.as<String>().c_str());
        client.publish(topic2.c_str(), Pulsetelemetry.as<String>().c_str());
        //Serial.println("Published on topic: " + topic1);*/
    
   /* xSemaphore is used for serial debugging only uncomment if necessary 
   
   if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
        Serial.print("temperatureC=");
        Serial.print(temperature, 4);
        Serial.println();
        
        Serial.print("HR=");
        Serial.print(heartRate, DEC);
        Serial.println();
        
        Serial.print("SPO2=");
        Serial.print(spo2, DEC);
        Serial.println();
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    } */

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
  
}

void ECG( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  { 
    //client.loop();
    // read the input on analog pin 0:
    int sensorValue = analogRead(PA0);
    
    //Serial.println(sensorValue);
    DynamicJsonDocument ECGtelemetry(1023);
    ECGtelemetry.createNestedObject();
    ECGtelemetry[0]["ElectroCardioGram"] = sensorValue;
    
    String topic2 = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
    client.publish(topic2.c_str(), ECGtelemetry.as<String>().c_str());

    /*If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free
      So uncomment if necessary*/
     
    /*if (xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println("Published on topic: " + topic2);
      Serial.println(sensorValue);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }*/
    vTaskDelay(50);  // one tick delay (15ms) in between reads for stability
  }
}
void SerialRead(void *pvParameters)
{
  for (;;)
  {
        
    // Read the data from the serial port (assuming data is in ASCII)
    // and convert it to an integer.
    if(Serial2.available() > 0)
   {
    char receivedData = Serial2.read();
    if(receivedData == 'C')
    {   
       /*If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free
        So uncomment if necessary*/
       if (xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE)
        {
               Serial.println(receivedData);
               Serial.println("Alert");
               nurseAlert();
               xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
        
        }
    }
    else if(receivedData == 'A')
    {
          Serial.println("Anomaly");
          nurseAlert();
      
    }
    
    // Delay before reading again.
  }
  vTaskDelay(10); // Adjust the delay time as needed.
}
}

void reconnect() 
{
  while (!client.connected()) 
  {
    Serial.println("Attempting MQTT connection...");
    char *client_id = "client-id-123ab";
    if (client.connect(client_id)) 
    {
      Serial.println("Connected to WiFi");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void nurseAlert()
{
    EthernetClient client1;
    HttpClient http(client1);

    char path[200];
    strcpy(path, "/trigger/nurse_alert/with/key/");
    strcat(path, apikey);
  
    int err = http.get(host, path);
    if (err == 0)
    {
      Serial.println("Nurse Alert Sent");
    }
      else
    {
        Serial.println("Connection failed");
    }
}

void Alert(String variable, String values)
{
    EthernetClient client1;
    HttpClient http(client1);

    char path[200];
    strcpy(path, "/trigger/alert/with/key/");
    strcat(path, apikey);
    strcat(path, "?value1=");
    strcat(path, variable.c_str()); 
    strcat(path, "&value2=");
    strcat(path, values.c_str());
  
    int err = http.get(host, path);
    if (err == 0)
    {
      Serial.println("Alert Sent");
    }
      else
    {
        Serial.println("Connection failed");
    }
}
