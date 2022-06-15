#include <HardwareSerial.h>
#include <LinkedList.h> // FR

//LinkedList<int> myList = LinkedList<int>(); // FR

#define MBUS_FRAME_SHORT_START          0x10
#define MBUS_FRAME_LONG_START           0x68
#define MBUS_FRAME_STOP                 0x16

#define MBUS_CONTROL_MASK_SND_NKE       0x40
#define MBUS_CONTROL_MASK_DIR_M2S       0x40
#define MBUS_ADDRESS_NETWORK_LAYER      0xFE

#define MBUS_ACK                        0xE5 (229)

int mbus_scan() {
  unsigned long timer_start = 0;
  
  for (byte address = 1; address <= 250; address++) {
    for (byte retry=1; retry<=2; retry++) {  
      Serial.print("Scanning address: ");
      Serial.println(address);
      mbus_application_reset(address); //FR
      //mbus_normalize(address);  //FR
      timer_start = millis();
      while (millis()-timer_start<256) {
        if (customSerial->available()) {
          byte val = customSerial->read();

          // Collects the found addresses
          if (customSerial->available() == 0 && addressList.get(addressList.size() - 1) != address){
            addressList.add(address);
            int listSize = addressList.size();
            Serial.print("There are ");
            Serial.print(listSize);
            Serial.print(" integers in the list. The last one are: ");
            Serial.print(addressList.get(addressList.size()-1));
          } 
        }
      }
    }
  }
  return -1;
}

// ---------------------------------------------------------------

void mbus_normalize(byte address) {
  mbus_short_frame(address,0x40);
}

void mbus_request_data(byte address) {
  mbus_short_frame(address,0x5b);
}

void mbus_application_reset(byte address) {
  mbus_control_frame(address,0x53,0x50);
}

void mbus_request(byte address,byte telegram) {
 
  byte data[15];
  byte i=0;
  data[i++] = MBUS_FRAME_LONG_START;
  data[i++] = 0x07;
  data[i++] = 0x07;
  data[i++] = MBUS_FRAME_LONG_START;
  
  data[i++] = 0x53;
  data[i++] = address;
  data[i++] = 0x51;

  data[i++] = 0x01;
  data[i++] = 0xFF;
  data[i++] = 0x08;
  data[i++] = telegram;

  unsigned char checksum = 0;
  for (byte c=4; c<i; c++) checksum += data[c];
  data[i++] = (byte) checksum;
  
  data[i++] = 0x16;
  data[i++] = '\0';
  
  customSerial->write((char*)data);
}

void mbus_set_address(byte oldaddress, byte newaddress) {
 
  byte data[13];
 
  data[0] = MBUS_FRAME_LONG_START;
  data[1] = 0x06;
  data[2] = 0x06;
  data[3] = MBUS_FRAME_LONG_START;
  
  data[4] = 0x53;
  data[5] = oldaddress;
  data[6] = 0x51;
  
  data[7] = 0x01;         // DIF [EXT0, LSB0, FN:00, DATA 1 8BIT INT]
  data[8] = 0x7A;         // VIF 0111 1010 bus address
  data[9] = newaddress;   // DATA new address
  
  data[10] = data[4]+data[5]+data[6]+data[7]+data[8]+data[9];
  data[11] = 0x16;
  data[12] = '\0';
  
  customSerial->write((char*)data);
}

void mbus_set_baudrate(byte address, byte baudrate) {
 
  byte data[11];
  byte i=0;
  
  data[i++] = MBUS_FRAME_LONG_START;
  data[i++] = 0x03;
  data[i++] = 0x03;
  data[i++] = MBUS_FRAME_LONG_START;
  
  data[i++] = 0x53;
  data[i++] = address;
  data[i++] = baudrate;
  
  unsigned char checksum = 0;
  for (byte c=4; c<i; c++) checksum += data[c];
  data[i++] = (byte) checksum; 
  
  data[i++] = 0x16;
  data[i++] = '\0';
  
  customSerial->write((char*)data);
}

void mbus_set_id(byte address) {
 
  byte data[16];
  byte i=0;
  
  data[i++] = MBUS_FRAME_LONG_START;
  data[i++] = 0x09;
  data[i++] = 0x09;
  data[i++] = MBUS_FRAME_LONG_START;
  
  data[i++] = 0x53;
  data[i++] = address;
  data[i++] = 0x51;
  
  data[i++] = 0x0C;
  data[i++] = 0x79;
  data[i++] = 0x01; //ID1
  data[i++] = 0x02; //ID2
  data[i++] = 0x03; //ID3
  data[i++] = 0x04; //ID4
    
  unsigned char checksum = 0;
  for (byte c=4; c<i; c++) checksum += data[c];
  data[i++] = (byte) checksum; 
  
  data[i++] = 0x16;
  data[i++] = '\0';
  
  customSerial->write((char*)data);
}

// ---------------------------------------------------------------

void mbus_short_frame(byte address, byte C_field) {
  byte data[6];

  data[0] = 0x10;
  data[1] = C_field;
  data[2] = address;
  data[3] = data[1]+data[2];
  data[4] = 0x16;
  data[5] = '\0';

  customSerial->write((char*)data);
}

void mbus_control_frame(byte address, byte C_field, byte CI_field)
{
  byte data[10];
  data[0] = MBUS_FRAME_LONG_START;
  data[1] = 0x03;
  data[2] = 0x03;
  data[3] = MBUS_FRAME_LONG_START;
  data[4] = C_field;
  data[5] = address;
  data[6] = CI_field;
  data[7] = data[4] + data[5] + data[6];
  data[8] = MBUS_FRAME_STOP;
  data[9] = '\0';

  customSerial->write((char*)data);
}
