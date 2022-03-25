
#include <rpcWiFi.h>
//
#include"TFT_eSPI.h"
#include <PubSubClient.h>

#include <Seeed_FS.h>
#include "SD/Seeed_SD.h"
#include <SPI.h>

#include <arduino.h>

#define PIC_PKT_LEN    2048                  //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    5
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define CAM_SERIAL     Serial

#define PIC_FMT        PIC_FMT_VGA


const char* mqtt_server = "192.168.1.11";  // MQTT Broker URL

WiFiClient wioClient;
File cred;

PubSubClient client(wioClient);


const byte cameraAddr = (CAM_ADDR << 5);
const int buttonPin = A5;                 // the number of the pushbutton pin
unsigned long picTotalLen = 0;            // picture length
int picNameNum = 0;

String MQTT_USER,MQTT_PASS,SSID,WIFI_PASS;

/*********************************************************************/
void setup()
{
    Serial.begin(115200);
    while (!Serial) {};
    Serial1.begin(115200);
    while (!Serial1) {};
//    pinMode(WIO_KEY_A, INPUT_PULLUP);    // initialize the pushbutton pin as an input
    Serial.println("Initializing SD card....");
    while (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI)) {};

    if (cred = SD.open("cred.txt", FILE_READ)) {
        credReader(cred,&MQTT_USER,&MQTT_PASS,&SSID,&WIFI_PASS);
        Serial.println(MQTT_USER);
        Serial.println(MQTT_PASS);
        Serial.println(SSID);
        Serial.println(WIFI_PASS);
        cred.close();
    } else {
        Serial.println("credential file does not exists");
    }
    setup_wifi();
    client.setServer(mqtt_server,1883);

    camera_initialize();
}
/*********************************************************************/
void loop()
{

    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    delay(200);
    Capture();
    GetData();


//press button to take a picture does not work currently for test purpose.
/*
    int n = 0;
    while(1){
        Serial.println("\r\nPress the button to take a picture");
        while (digitalRead(WIO_KEY_A) == LOW);      //wait for buttonPin status to HIGH
        if(digitalRead(WIO_KEY_A) == HIGH){
            delay(20);                               //Debounce
            if (digitalRead(WIO_KEY_A) == HIGH)
            {
                Serial.println("\r\nbegin to take picture");
                delay(200);
                Capture();
                GetData();
            }
            Serial.print("\r\nTaking pictures success ,number : ");
            Serial.println(n);
            n++ ;
        }
    }
*/}
/*********************************************************************/
void clearRxBuf()
{
    while (Serial1.available())
    {
        Serial1.read();
    }
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
    for (char i = 0; i < cmd_len; i++) Serial1.print(cmd[i]);
}
/*********************************************************************/
void camera_initialize()
{
    char cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00} ;
    unsigned char resp[6];

    Serial1.setTimeout(500);
    while (1)
    {
        //clearRxBuf();
        sendCmd(cmd,6);
        if (Serial1.readBytes((char *)resp, 6) != 6)
        {
            continue;
        }
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
        {
            if (Serial1.readBytes((char *)resp, 6) != 6) continue;
            if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
        }
    }
    cmd[1] = 0x0e | cameraAddr;
    cmd[2] = 0x0d;
    sendCmd(cmd, 6);
    Serial.println("\nCamera initialization done.");
}
/*********************************************************************/
void Capture()
{
    char cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0};
    unsigned char resp[6];

    Serial1.setTimeout(100);
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (Serial1.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
    }
    cmd[1] = 0x05 | cameraAddr;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (Serial1.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
    }
    cmd[1] = 0x04 | cameraAddr;
    cmd[2] = 0x1;
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (Serial1.readBytes((char *)resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
        {
            Serial1.setTimeout(1000);
            if (Serial1.readBytes((char *)resp, 6) != 6)
            {
                continue;
            }
            if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
            {
                picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
                Serial.print("picTotalLen:");
                Serial.println(picTotalLen);
                break;
            }
        }
    }

}
/*********************************************************************/
void GetData()
{
    unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
    if ((picTotalLen % (PIC_PKT_LEN-6)) != 0) pktCnt += 1;

    char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
    unsigned char pkt[PIC_PKT_LEN];

        Serial1.setTimeout(1000);
        for (unsigned int i = 0; i < pktCnt; i++)
        {
            cmd[4] = i & 0xff;
            cmd[5] = (i >> 8) & 0xff;

            int retry_cnt = 0;
            retry:
            delay(10);
            clearRxBuf();
            sendCmd(cmd, 6);
            uint16_t cnt = Serial1.readBytes((char *)pkt, PIC_PKT_LEN);

            unsigned char sum = 0;
            for (int y = 0; y < cnt - 2; y++)
            {
                sum += pkt[y];
            }
            if (sum != pkt[cnt-2])
            {
                if (++retry_cnt < 100) goto retry;
                else break;
            }
            //Send Image packet to MQTT Broker.
            client.publish("WTout/jpg",(const uint8_t *)&pkt[4],cnt-6);
        }
        cmd[4] = 0xf0;
        cmd[5] = 0xf0;
        sendCmd(cmd, 6);
        //Send terminator to write image file.
        client.publish("WTout/jpg/done","0");
}



void setup_wifi() {

  delay(10);


  Serial.println();
  Serial.print("Connecting to ");

  Serial.println(SSID);
  WiFi.begin(SSID.c_str(), WIFI_PASS.c_str()); // Connecting WiFi

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); // Display Local IP Address
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "WioTerminal-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER.c_str(),MQTT_PASS.c_str())) {//MQTT_USER.c_str(),MQTT_PASS.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("WTout", "hello world");
      // ... and resubscribe.
      //No one publish to WTin currently.
      client.subscribe("WTin");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



//Read the text file that contains credential information.
//Line 1 is MQTT Username.
//Line 2 is MQTT Password.
//Line 3 is SSID.
//Line 4 is WiFi Password.
void credReader(File file,String *mqtt_user, String *mqtt_pass , String *ssid, String *wifi_pass) {
  
  int i = 0;
  char buffer[64];
  
  while (file.available()) {
   int l = file.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
   buffer[l] = 0;
   switch (i)
   {
    case 0:
        *mqtt_user = buffer;
        break;
    case 1:
        *mqtt_pass = buffer;
        break;
    case 2:
        *ssid = buffer;
        break;
    case 3:
        *wifi_pass = buffer;
        break;
   default:
       break;
   }
   i++;
  }

}
