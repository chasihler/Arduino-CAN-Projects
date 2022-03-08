#include <SPI.h>
#include <mcp2515.h>
#include <LiquidCrystal.h>
#include "DFRkeypad.h"


struct can_frame canMsg;
MCP2515 mcp2515(49);
enum ePins { LCD_RS=8, LCD_EN=9, LCD_D4=4, LCD_D5=5, LCD_D6=6, LCD_D7=7, LCD_BL=10 }; // define LCD pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the library with the numbers of the interface pins

  unsigned long previous1Millis = 0;    
  const long wait_interval1 = 500; 
  unsigned long previous2Millis = 0;    
  const long wait_interval2 = 1000; 
  
  byte lastKey=DFRkeypad::eINVALID_KEY;
  float sum[DFRkeypad::eNUM_KEYS], sumq[DFRkeypad::eNUM_KEYS];  // arrays to sum up values for mean and deviation
  unsigned int values[DFRkeypad::eNUM_KEYS];                    // counter for number of samples
  unsigned long LastTime;
  enum eSymbols { symPLUSMINUS=0 };                             // deviation LCD Symbol "+/-"
  byte char_plusminus[8]=
  {
  B00100,
  B00100,
  B11111,
  B00100,
  B00100,
  B00000,
  B11111,
  B00000,
  };
  
 void clearStat()   {
  memset(sum, 0, sizeof(sum));
  memset(sumq, 0, sizeof(sumq));
  memset(values, 0, sizeof(values));
}


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, 0x001);
  mcp2515.setFilter(MCP2515::RXF1, false, 0x011);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF2, false, 0x0F6);
  mcp2515.setFilter(MCP2515::RXF3, false, 0x613);
  mcp2515.setFilter(MCP2515::RXF4, false, 0x613);
  mcp2515.setFilter(MCP2515::RXF5, false, 0x613);
  mcp2515.setNormalMode();


  lcd.begin(16, 2);                               // set up the LCD's number of columns and rows (16x2)
  lcd.createChar(symPLUSMINUS, char_plusminus);   // create +/- character

  pinMode(LCD_BL, OUTPUT);                        // pin LCD_BL is LCD backlight brightness (PWM)
  analogWrite(LCD_BL, 255);                       // set the PWM brightness to maximum
  lcd.setCursor(0, 0);
  lcd.print("CAN RDR");

  DFRkeypad::FastADC(true);                       // increase ADC sample frequency
  DFRkeypad::iDEFAULT_THRESHOLD=140;              // maximum threshold acceptable so bounds in DFRkeypad::iARV_VALUES are not overlapping
  clearStat();                                    // clear statistics

  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

 bool non_blocking_wait1 (void) {
  unsigned long currentMillis = millis();
  if (currentMillis - previous1Millis >= wait_interval1) {
    // time passed, save new time.
    previous1Millis = currentMillis;
    return true;
  }
  return false;
}

 bool non_blocking_wait2 (void) {
  unsigned long currentMillis = millis();
  if (currentMillis - previous2Millis >= wait_interval2) {
    // time passed, save new time.
    previous1Millis = currentMillis;
    return true;
  }
  return false;
}

 
byte digits(unsigned int iNum)  {
  byte bDigits=0;
  do
  {
    ++bDigits;
    iNum/=10;
  } while(iNum);
  return bDigits;
}

void printNumber(unsigned int number, byte numd) { //prints fixed width number
  byte d=digits(number);
  for(byte i=d; i<numd; ++i) lcd.print((char)' '); // padding
  lcd.print(number);
}

void babysitting_keypad (byte key_in) {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(key_in);     
  //lcd.print(DFRkeypad::KeyName(key_in));    // print key name
}



void loop() {
  bool dis = false;
  byte key=DFRkeypad::GetKey();                   // read a key identifier
  if(DFRkeypad::eNO_KEY<key && key<DFRkeypad::eINVALID_KEY)   {
    int val=analogRead(KEYPAD);                             // ... get the analog value for it
    sum[key]+=val;                                          // add val into array
    sumq[key]+=(float)val*(float)val;                       // add val^2 into array
    ++values[key];      
    if (non_blocking_wait1()) babysitting_keypad(key);// increase sample counter
  }
    

    
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x011) {
      lcd.setCursor(0, 1); 
      if (non_blocking_wait2()) {
        dis = true;  
        lcd.print("                ");
        //lcd.setCursor(0, 1);
        lcd.setCursor(8, 0);
        lcd.print("        ");
        lcd.setCursor(8, 0);
      }
    }  
    Serial.print(canMsg.can_id, HEX); // print ID
    if (dis) lcd.print(canMsg.can_id, HEX); 
    Serial.print(" "); 
    if (dis) lcd.print(" ");
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    if (dis) lcd.print(canMsg.can_dlc, HEX);
    Serial.print(" ");
    if (dis) lcd.setCursor(0, 1);
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      if (dis) lcd.print(canMsg.data[i], HEX);
      Serial.print(" ");
      if (dis) lcd.print(" ");
    }

    Serial.println();      
  }
  
}
