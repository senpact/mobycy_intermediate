#include <Base64.h>
#include <SoftwareSerial.h>
#include "SparkFunLIS3DH.h"
#include "Wire.h"
#define LOCKED 0
#define UNLOCKED 1
#define MAINTENANCE 2
#define PAUSE_N_RESUME 3
#define BUZZER 9
#define ALARM_ON 991
#define ALARM_OFF 990
#define DEBUG true
#define TOLERANCE  0.05
#define MAX_DEFLECTION_ALLOWED  0.05

SoftwareSerial blueSerial(10, 4);
LIS3DH SensorTwo( I2C_MODE, 0x18 );

long previous_time, current_time, previous_sms_time, current_sms_time; 
const long lock_interval = 1800000, unlock_interval = 60000, pause_interval = 300000, message_interval = 5000;
String vehicle_id = "V0003", tx_data , gps_state, gps_time, gps_lat, gps_lon, sendData(String, const int, boolean);
short ledpin1 = 12, ble_interrupt = 7, blue_flag, state, calib_cnt;
char input_String[80], encoded_String[80], decoded_String[80];
float x_min_val, x_max_val, x_curr_val, x_pre_val, y_min_val, y_max_val, y_curr_val, y_pre_val, z_min_val, z_max_val, z_curr_val, z_pre_val, per;
bool pos_flag, mov_flag, acc_flag, accelero(), moving(), alarm_flag;

void blue_intr_service() {
  blue_flag = true;
}

void enc64(char input_String[]) {
  int input_StringLength = 50;
  int encodedLength = Base64.encodedLength(input_StringLength);
  Base64.encode(encoded_String, input_String, input_StringLength);
}
void dec64(char encoded_String[]) {
  int input_StringLength = 80;
  int decodedLength = Base64.decodedLength(encoded_String, input_StringLength);
  Base64.decode(decoded_String, encoded_String, input_StringLength);
}
void calibrate() {
  if (!blue_flag)
  {
//    Serial.println("\n CALIBRATING");
//    for (int i = 0; i < 1; i++)
//    {
//      x_curr_val = SensorTwo.readFloatAccelX();
//      x_min_val = x_curr_val; x_max_val = x_curr_val;
//
//      y_curr_val = SensorTwo.readFloatAccelY();
//      y_min_val = y_curr_val; y_max_val = y_curr_val;
//
//      z_curr_val = SensorTwo.readFloatAccelZ();
//      z_min_val = z_curr_val; z_max_val = z_curr_val;
//    }
    for (int j = 0; j < 100; j++)
    {
      delay(100);

      x_curr_val = SensorTwo.readFloatAccelX();
      if (x_curr_val > x_max_val)
      {
        x_max_val = x_curr_val;
      }
      else if (x_curr_val < x_min_val)
      {
        x_min_val = x_curr_val;
      }

      y_curr_val = SensorTwo.readFloatAccelY();
      if (y_curr_val > y_max_val)
      {
        y_max_val = y_curr_val;
      }
      else if (y_curr_val < y_min_val)
      {
        y_min_val = y_curr_val;
      }

      z_curr_val = SensorTwo.readFloatAccelZ();
      if (z_curr_val > z_max_val)
      {
        z_max_val = z_curr_val;
      }
      else if (z_curr_val < z_min_val)
      {
        z_min_val = z_curr_val;
      }
    }
  }
  pos_flag = 0;
}
bool accelero() {
  if (!blue_flag)
  {
    x_curr_val = SensorTwo.readFloatAccelX();
    y_curr_val = SensorTwo.readFloatAccelY();
    z_curr_val = SensorTwo.readFloatAccelZ();

    float x_max_deflection, y_max_deflection, z_max_deflection, x_min_deflection, y_min_deflection, z_min_deflection;
    //-----------------------------------
    if (x_curr_val > x_max_val) {
      x_max_deflection = x_curr_val - x_max_val;
    }
    else {
      x_max_deflection = x_max_val - x_curr_val;
    }
    if (y_curr_val > y_max_val) {
      y_max_deflection = y_curr_val - y_max_val;
    }
    else {
      y_max_deflection = y_max_val - y_curr_val;
    }
    if (z_curr_val > z_max_val) {
      z_max_deflection = z_curr_val - z_max_val;
    }
    else {
      z_max_deflection = z_max_val - z_curr_val;
    }

    if (x_curr_val > x_min_val) {
      x_min_deflection = x_curr_val - x_max_val;
    }
    else {
      x_min_deflection = x_max_val - x_curr_val;
    }
    if (y_curr_val > y_min_val) {
      y_min_deflection = y_curr_val - y_max_val;
    }
    else {
      y_min_deflection = y_max_val - y_curr_val;
    }
    if (z_curr_val > z_min_val) {
      z_min_deflection = z_curr_val - z_max_val;
    }
    else {
      z_min_deflection = z_max_val - z_curr_val;
    }
    ////--------------

    //    (x_curr_val > x_max_val) ?  (x_min_deflection = x_curr_val - x_max_val) : (x_min_deflection = x_max_val - x_curr_val);
    //    (y_curr_val > y_max_val) ?  (y_min_deflection = y_curr_val - y_max_val) : (y_min_deflection = y_max_val - y_curr_val);
    //    (z_curr_val > z_max_val) ?  (z_min_deflection = z_curr_val - z_max_val) : (z_min_deflection = z_max_val - z_curr_val);
    //
    //    (x_curr_val > x_min_val) ?  (x_min_deflection = x_curr_val - x_min_val) : (x_min_deflection = x_min_val - x_curr_val);
    //    (y_curr_val > y_min_val) ?  (y_min_deflection = y_curr_val - y_min_val) : (y_min_deflection = y_min_val - y_curr_val);
    //    (z_curr_val > z_min_val) ?  (z_min_deflection = z_curr_val - z_min_val) : (z_min_deflection = z_min_val - z_curr_val);

    //-----------
    if ( (x_min_deflection > TOLERANCE) || (x_max_deflection > TOLERANCE) ) {
//      Serial.println("x failed");
      pos_flag = 1;
    }
    if ( (y_min_deflection > TOLERANCE) || (y_max_deflection > TOLERANCE) ) {
//      Serial.println("y failed");
      pos_flag = 1;
    }
    if ( (z_min_deflection > TOLERANCE) || (z_max_deflection > TOLERANCE) ) {
//      Serial.println("z failed");
      pos_flag = 1;
    }
  }
}
bool moving() {
  mov_flag = 0;
  if (!blue_flag)
  {
    float x_deflection, y_deflection, z_deflection, z = 10000.00;
    x_curr_val = SensorTwo.readFloatAccelX();
    y_curr_val = SensorTwo.readFloatAccelY();
    z_curr_val = SensorTwo.readFloatAccelZ();

//    if (x_pre_val >= x_curr_val)
//    {
//      x_deflection = x_pre_val - x_curr_val;
//    }
//    if (x_pre_val < x_curr_val)
//    {
//      x_deflection = x_curr_val - x_pre_val;
//    }
//
//    if (y_pre_val >= y_curr_val)
//    {
//      y_deflection = y_pre_val - y_curr_val;
//    }
//    if (y_pre_val < y_curr_val)
//    {
//      y_deflection = y_curr_val - y_pre_val;
//    }
//
//    if (z_pre_val >= z_curr_val)
//    {
//      z_deflection = z_pre_val - z_curr_val;
//    }
//    if (z_pre_val < z_curr_val)
//    {
//      z_deflection = z_curr_val - z_pre_val;
//    }
//    //----------------

      (x_pre_val >= x_curr_val) ? (x_deflection = x_pre_val - x_curr_val) :  (x_deflection = x_curr_val - x_pre_val) ;
      (y_pre_val >= y_curr_val) ? (y_deflection = y_pre_val - y_curr_val) :  (y_deflection = y_curr_val - y_pre_val) ;
      (z_pre_val >= z_curr_val) ? (z_deflection = z_pre_val - z_curr_val) :  (z_deflection = z_curr_val - z_pre_val) ;
    //----------

    if ((x_deflection >= MAX_DEFLECTION_ALLOWED) ||  (y_deflection >= MAX_DEFLECTION_ALLOWED) ||  (z_deflection >= MAX_DEFLECTION_ALLOWED))
    {
      mov_flag = 1;
    }
    x_pre_val = x_curr_val;
    y_pre_val = y_curr_val;
    z_pre_val = z_curr_val;
  }
}
void buzzer(int tune_no) {
  if (tune_no == 0)
  {
    tone(BUZZER, 2500, 400);    delay(100);
    tone(BUZZER, 2400, 350);    delay(100);
    tone(BUZZER, 2200, 300);    delay(100);
    tone(BUZZER, 2000, 250);
    digitalWrite(BUZZER, LOW);
  }
  else if (tune_no == 1)
  {
    tone(BUZZER, 2000, 250);    delay(100);
    tone(BUZZER, 2200, 300);    delay(100);
    tone(BUZZER, 2400, 350);    delay(100);
    tone(BUZZER, 2500, 400);
    digitalWrite(BUZZER, LOW);
  }
  else if (tune_no == 3)
  {
    tone(BUZZER, 4000, 200);
    digitalWrite(BUZZER, LOW); delay(200);
    tone(BUZZER, 3000, 200);
    digitalWrite(BUZZER, LOW);
  }
  else if (tune_no == 991)
  {
    digitalWrite(BUZZER, HIGH);
  }
  else if (tune_no == 990)
  {
    digitalWrite(BUZZER, LOW);
  }
}
void sendTabData(String command, const int timeout, boolean debug) {
  String latlongtab[5];
  Serial1.println(command);
  long int time = millis();
  int i = 0;

  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c != ',') {
        latlongtab[i] += c;
        delay(100);
      }
      else {
        i++;
      }
      if (i == 5) {
        delay(100);
        goto exitL;
      }
    }
  }
exitL:
  if (debug) {
    gps_state = latlongtab[1];
    gps_time = latlongtab[2];
    gps_lat = latlongtab[3];
    gps_lon = latlongtab[4];
  }
}
String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  Serial1.println(command);
  long time = millis();
  int i = 0;

  while ( (time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }
  if (debug) {
    Serial.print(response);
  }
  return response;
}
void read_sms() {
  Serial1.print("AT+CMGF=1\r");
  delay(200);  Serial.flush();

  Serial1.print("AT+CNMI=2,2,0,0,0\r");
  delay(100);  Serial1.flush();

  String textMessage = "";
  for (int i = 0; i < 5; i++) {
    while (Serial1.available() > 0) {
      textMessage = Serial1.readString();
      delay(10);
    }
  }
  Serial1.flush();
  Serial.println(textMessage);
  if ((textMessage.indexOf("1444w2w4@424953495") > 0) || (textMessage.indexOf("+916363521285") > 0) || (textMessage.indexOf("+918600807701") > 0))
  {
    tx_data = "";
    if (textMessage.indexOf("Find") > 0)
    {
      Serial.println("Find mode");
      buzzer(ALARM_ON);
      tx_data = "";
    }
    else if (textMessage.indexOf("Lock") > 0)
    {
      Serial.println("Relay set to off/lock");
      state = LOCKED;
      Serial.println("Vechicle locked");
      digitalWrite(ledpin1, LOW);
//      delay(50);
      buzzer(LOCKED);
    }
    else if  (textMessage.indexOf("Unlock") > 0)
    {
      Serial.println("Relay set to on/unlock");
      state = UNLOCKED;
      Serial.println("Vechicle unlocked");
      digitalWrite(ledpin1, HIGH);
//      delay(50);
      buzzer(UNLOCKED);
    }
    else if  (textMessage.indexOf("Mains") > 0)
    {
      Serial.println("Relay set to on/unlock/maintenance");
      state = MAINTENANCE;
    }
    else if  (textMessage.indexOf("Pause") > 0)
    {
    Serial.println("Relay set to off/lock/pause");
      state = PAUSE_N_RESUME;
      Serial.println("Vechicle locked");
      digitalWrite(ledpin1, LOW);
//      delay(100);
      buzzer(PAUSE_N_RESUME);
    }

    if (analogRead(A1) < 200) {
      Serial.print("fet feedback:"); Serial.println(analogRead(A1));
      tx_data = ""; tx_data += vehicle_id; tx_data += ",Unlocked"; tx_data += ""; //Serial.print(input_String); 
      tx_data.toCharArray(input_String, 50); enc64(input_String);  send_by_tcpip(encoded_String);
    }
    else if (analogRead(A1) > 200) {
      Serial.print("fet feedback:"); Serial.println(analogRead(A1));
      tx_data = ""; tx_data += vehicle_id; tx_data += ",locked"; tx_data += ""; //Serial.print(input_String); 
      tx_data.toCharArray(input_String, 50); enc64(input_String);  send_by_tcpip(encoded_String);
    }
    Serial.flush();
    Serial1.flush();
  }
}
void send_by_tcpip(char encoded_String[]) {
  if (!blue_flag)
  {
    sendData("AT+CIPSHUT", 1000, DEBUG);
    sendData("AT+CIPMUX=0", 1000, DEBUG);
    sendData("AT+CGATT=1", 1000, DEBUG);
    sendData("AT+CSTT=\"airtelgps.com\",\"\",\"\"", 1000, DEBUG);

    sendData("AT+CIICR", 3000, DEBUG);
    delay(200);
    sendData("AT+CIFSR", 3000, DEBUG);
    delay(200);
    sendData("AT+CIPSTART=\"TCP\",\"13.126.228.36\",\"8888\"", 2000, DEBUG);
  }
  if (!blue_flag)
  {
    delay(1500); 
    sendData("AT+CIPSEND", 2000, DEBUG);
    Serial1.print(encoded_String);
    Serial1.write(0x1A);
    sendData("AT+CIPSHUT", 2000, DEBUG);
    delay(100); 
  }
  memset(encoded_String, 0, 80);
  memset(input_String, 0, 80);
  memset(decoded_String, 0, 80);
  tx_data = "";
}
void readgps()
{
  if (!blue_flag)
  {
    sendData("AT+CGPSPWR=1", 1000, DEBUG);
    sendData("AT+CGPSSTATUS?", 1000, DEBUG);
    sendTabData("AT+CGNSINF", 1000, DEBUG);

    get_battery_level();
    Serial.println("the battery level is ");Serial.println(per);
    int timezonemn = 30, timezonehr = 5;
    String dateS = gps_time.substring(0, 8), hrS = gps_time.substring(8, 10), mnS = gps_time.substring(10, 12), secS = gps_time.substring(12, 14);
    long hr = hrS.toInt(), mn = mnS.toInt();

    mn = mn + timezonemn;
    if (mn > 59) {
      mn = mn - 60;
      hr = hr + 1;
    }
    else {
      if (mn < 0) {
        mn = mn + 60;
        hr = hr - 1;
      }
    }
    hr = hr + timezonehr;
    if (hr > 23) {
      hr = hr - 24;
    }
    else {
      if (hr < 0) {
        hr = hr + 24;
      }
    }
    gps_time = "";
    gps_time += dateS;
    if (hr < 10)
    {
      gps_time += "0";
    }
    gps_time += hr;
    if (mn < 10)
    {
      gps_time += "0";
    }
    gps_time += mn;
    gps_time += secS;
    tx_data += vehicle_id; tx_data += ',';
    tx_data += gps_lat; tx_data += ',';
    tx_data += gps_lon; tx_data += ',';
    tx_data += per; tx_data += ',';
    tx_data += gps_time;
    tx_data.toCharArray(input_String, 50);
    enc64(input_String);
//    Serial.println("local time");
//    Serial.print(hr); Serial.print(":");
//    Serial.print(mn); Serial.print(":");
//    Serial.println(secS);
    send_by_tcpip(encoded_String);
  }
}
float get_battery_level()
{
  unsigned int ADCValue;
  ADCValue = analogRead(A0);
  
    per = (ADCValue - 500.0) / 523.0 * 100.0;
    if(per < 0)
    {
      per *=-1;
    }
    Serial.print("ADCValue="); Serial.println(ADCValue); Serial.println("");
    Serial.print("per="); Serial.println(per); Serial.println("");
  }

void setup()
{
  Serial.begin(9600);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  delay(2000);
  digitalWrite(11, LOW);
  if ( SensorTwo.begin() != 0 )
  {
    Serial.println("Problem starting the sensor at 0x18.");
  }
  else
  {
    Serial.println("Sensor at 0x18 started.");
  }
  //------------------ sim808 -------------------------
  Serial1.begin(9600);
  sendData("AT+CGPSPWR=1", 1000, DEBUG);
  sendData("AT+CGPSSTATUS?", 1000, DEBUG);
  sendData("AT+CGNSSEQ=RMC", 1000, DEBUG);
  //----------------- bluetooth -----------------------
  pinMode(ledpin1, OUTPUT);
  blueSerial.begin(9600);
  pinMode(ble_interrupt, INPUT);
  digitalWrite(ble_interrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(ble_interrupt), blue_intr_service,  RISING);
  //------------------ buzzer ----------------------------
  pinMode(BUZZER, OUTPUT);
  //------------------ battery % -------------------------
  pinMode(A0, INPUT_PULLUP);
  calibrate();
}
void loop()
{
  digitalWrite(11, LOW);
  
  //---------------- bluetooth ------------------------
  
  if (blue_flag)
  {
    String blue_data = "";
    String incoming_char = "";
    blueSerial.listen();
    delay(100);

    if (Serial.available() > 0) {
      blueSerial.write(Serial.read());
    }
    if (blueSerial.available() > 0) {
      for (int i = 0; i < 3; i++) {
        while (blueSerial.available()) {
          incoming_char += (char)blueSerial.read();
        }
      }
    }
    blueSerial.flush();
    Serial.println(incoming_char); 
    incoming_char.toCharArray(encoded_String, 50); 
    dec64(encoded_String);

    for (int i = 0; i < 80; i++)
    {
      blue_data += decoded_String[i];
    }
    Serial.println("blue data");
    Serial.println(blue_data);
    int a = blue_data.indexOf(",");
    int b = blue_data.indexOf(",", a + 1);
    int c = blue_data.indexOf(",", b + 1);
    String parAB = blue_data.substring(c + 1);
    
    if (  (parAB == "AB") && (blue_data.indexOf("AA") == 0)  )
    {
      if ( (blue_data.indexOf("V0001") > 0) )
      {
        if (blue_data.indexOf("Find") > 0)
        {
          blue_flag = 0;
          Serial.println("Find mode");
          buzzer(ALARM_ON);
          blue_data = "";
        }
        else if (blue_data.indexOf("Unlock") > 0)
        {
          blue_flag = 0;
          Serial.println("Vechicle unlocked ");
          digitalWrite(ledpin1, HIGH);
          state = UNLOCKED;
          buzzer(UNLOCKED);
          blue_data = "";
        }

        else if (blue_data.indexOf("Lock") > 0)
        {
          blue_flag = 0;
          Serial.println("Vechicle locked");
          digitalWrite(ledpin1, LOW);
          state = LOCKED;
          buzzer(LOCKED);
          blue_data = "";
        }
        else if (blue_data.indexOf("Mains") > 0)
        {
          blue_flag = 0;
          Serial.println("Maintainence Start mode");
          digitalWrite(ledpin1, HIGH);
          state = MAINTENANCE;
          blue_data = "";
        }
        else if (blue_data.indexOf("Pause") > 0)
        {
          blue_flag = 0;
          Serial.println("PAUSE_N_RESUME mode");
          digitalWrite(ledpin1, LOW);
          state = PAUSE_N_RESUME;
          buzzer(PAUSE_N_RESUME);
          blue_data = "";
        }
        else
        {
          blue_flag = 0;
          blueSerial.flush();
          blue_data = "";
        }
        delay(300);
        if (analogRead(A1) < 200) {
          Serial.println("fet feedback:"); Serial.println(analogRead(A1));
          tx_data = ""; tx_data += vehicle_id; tx_data += ",Unlocked"; tx_data += ""; Serial.println(input_String); tx_data.toCharArray(input_String, 50); enc64(input_String);  blueSerial.println(encoded_String); Serial.print(encoded_String);
        }
        else if (analogRead(A1) > 200) {
          Serial.println("fet feedback:"); Serial.println(analogRead(A1));
          tx_data = ""; tx_data += vehicle_id; tx_data += ",locked"; tx_data += ""; Serial.println(input_String); tx_data.toCharArray(input_String, 50); enc64(input_String);  blueSerial.println(encoded_String); Serial.print(encoded_String);
        }
        blue_flag = 0;
        blueSerial.flush();
        blue_data = "";
      }
    }
  }
  //-------------------------------------------------------
  
  switch (state)
  {
    case LOCKED:
      Serial.println("\n  LOCKED");
      accelero();
      moving();
      Serial.print("moves-----"); Serial.println(mov_flag);
      Serial.print("pos======="); Serial.println(pos_flag );
      if (mov_flag && pos_flag)
      {
        Serial.print("the vehicle is moving");
        readgps();
        buzzer(ALARM_ON);
        acc_flag = 1;
        calib_cnt = 0;
      }
      if (!mov_flag)
      {
        calib_cnt++;
        Serial.println("calib_cnt");
        Serial.println(calib_cnt);
        delay(1000);
      }
      if ( acc_flag && (calib_cnt == 10))
      {
        calibrate();
        acc_flag = 0;
        pos_flag = 0;
        calib_cnt = 0;
        buzzer(ALARM_OFF);
      }
      current_time = millis();
      if ((current_time - previous_time) > lock_interval)
      {
        readgps();
        previous_time = current_time;
      }
      while (1)
      {
        Serial.println("\n  sms");
        current_sms_time = millis();
        if ( ((current_sms_time - previous_sms_time) > message_interval) || (blue_flag == 1) )
        {
          previous_sms_time = current_sms_time;
          break;
        }
        read_sms();
      }
      break;

    case UNLOCKED:
      Serial.println("\n  UN_LOCKED");
      current_time = millis();
      if ((current_time - previous_time) > unlock_interval)
      {
        readgps();
        previous_time = current_time;
      }
      while (1)
      {
        current_sms_time = millis();
        if (  ((current_sms_time - previous_sms_time) > message_interval) || (blue_flag == 1) )
        {
          previous_sms_time = current_sms_time;
          break;
        }
        read_sms();
      }
      break;

    case MAINTENANCE:
      Serial.println("\n MAINTENANCE ");
      while (1)
      {
        current_sms_time = millis();
        if (  ((current_sms_time - previous_sms_time) > message_interval) || (blue_flag == 1) )
        {
          previous_sms_time = current_sms_time;
          break;
        }
        read_sms();
      }
      break;

    case PAUSE_N_RESUME :
      Serial.println("\n PAUSE_N_RESUME ");
      current_time = millis();
      if ((current_time - previous_time) > pause_interval)
      {
        readgps();
        previous_time = current_time;
      }
      while (1)
      {
        current_sms_time = millis();
        if (  ((current_sms_time - previous_sms_time) > message_interval) || (blue_flag == 1) )
        {
          previous_sms_time = current_sms_time;
          break;
        }
        read_sms();
      }
  }
  if (digitalRead(7) == LOW) {
    blue_flag = 0;
  }
}
