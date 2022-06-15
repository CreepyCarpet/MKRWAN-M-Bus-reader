// todo cleanup datalists after binary conversion

#include <MKRWAN.h>
#include <HardwareSerial.h>
#include <LinkedList.h>
#include <math.h>
using namespace std;

//M-Bus constant initalization
#define DEBUG true
#define MBUS_BAUD_RATE 2400
#define MBUS_TIMEOUT 1000 // milliseconds
#define MBUS_DATA_SIZE 255
#define MBUS_GOOD_FRAME true
#define MBUS_BAD_FRAME false
uint8_t MBusSize;
HardwareSerial *customSerial;
LinkedList<int> addressList = LinkedList<int>(); // Linked list that holds all found M-Bus addresses
LinkedList<String> dataList = LinkedList<String>(); // Linked list that holds data from M-Bus requests
LinkedList<byte> dataList2 = LinkedList<byte>(); // Linked list that holds data from M-Bus requests
char packet[64];


//LoRaWAN constant initialization
LoRaModem modem;
String appEui = "5BFB2434447BC080";
String appKey = "7FB0AEBAE01E276BAF05E87E184F534B";
String devAddr;
String nwkSKey;
String appSKey;
int maxPacketSize = 64;

int err = 0;
int usedBytes = 0;
int addressNumber = 0;

void setup() {
  //Lora OTAA connection
  Serial.begin(115200);

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };

  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  
  delay(5000);
  
  float packTime = getPackageTime(7, 125000, 64);
  Serial.println(packTime);

  // Wait random time, then try to join
  loraOTAAJoin(appKey, appEui);

  modem.setADR(true);

  modem.beginPacket();
  int err;
  // Print into data packet
  modem.print("Hi");

  // Send data packet
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
  } 

  //M-Bus serial initialization
  customSerial = &Serial1;
  customSerial->begin(MBUS_BAUD_RATE, SERIAL_8E1); // mbus uses 8E1 encoding
  delay(5000); // Delay to startup. Let the serial initialize, or we get a bad first frame
  mbus_scan(); // First scan to get addresses
  Serial.println("Setup and Scan finished");
}

void loraOTAAJoin(String appKey, String appEui) {
  // Use deviceEUI and fluctuating analog value to generate random seed for random join wait if power cycled to prevent network congestion.
  String deviceEUI = modem.deviceEUI();
  uint32_t deviceSeed = 1; 
  for(int i = 0; i < deviceEUI.length(); i++) {
    char thisChar = deviceEUI.charAt(i);

    for(int j = 0; j < 8; j++) {
      byte bit = bitRead(thisChar, j);
      if (bit == true) {
        deviceSeed = deviceSeed*2;
      }
    }
  }
  Serial.println("deviceSeed is" + deviceSeed);
  randomSeed(deviceSeed + analogRead(0));
  uint32_t randomInt = random(120);
  for (int i = 0; i < randomInt; i++) {
    delay(1000);
    Serial.print(".");
  }

  // Attempt OTAA connection
  bool connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (randomInt < 3600) {
      randomInt = randomInt*2;
      for (int i = 0; i < randomInt; i++) {
        delay(1000);
        Serial.print(".");
      }
      connected = modem.joinOTAA(appEui, appKey);
    }
  }
  return;
}

void mbusRequest(byte address) {
  // Reset variables
  bool mbus_good_frame = false;
  byte mbus_data[MBUS_DATA_SIZE] = { 0 };
  
  // Request and check that frame is valid
  mbus_request_data(address);
  mbus_good_frame = mbus_get_response(mbus_data, sizeof(mbus_data));

  if (mbus_good_frame) {
    Serial.println(F("mbus: good frame: "));
    // Clears dataList, Prints data and enters it in dataList
    MBusSize = print_bytes(mbus_data, sizeof(mbus_data));

  }
  else {
    Serial.print(F("mbus: bad frame: ")); 
    MBusSize = print_bytes(mbus_data, sizeof(mbus_data));
  }
  for (int i = 0;i < dataList2.size(); i++) {
    Serial.print(dataList2.get(i));
  }
}

void getMBusDevices() {
  //Todo
}

void toggleMBusDevice(byte address) {
  //Todo
}

uint16_t setUplinkInterval(u_int16_t interval) {
  uint8_t dataRate = modem.getDataRate();
  uint8_t bandWidth;
  if (dataRate == 6) {
    bandWidth = 250;
  }
  else {
    bandWidth = 125;
  }
  uint8_t spreadingFactor = 12 - dataRate;
  uint8_t minPackageTime = getPackageTime(spreadingFactor, bandWidth, 0);
  if (interval > minPackageTime) {
    interval = minPackageTime;
    Serial.println("Requested interval lower than shortest allowed, setting as shortest allowed");
  }
  return interval;
}

void createCustomUplink() {
  //Todo
}

void getCustomUplinkList() {
  //Todo
}

void powerCycle() {
  //todo
}

void rescanMBus() {
  //todo
}

void forceRejoin() {
  //todo
}

uint8_t getPayloadMaxLength(uint8_t spreadingFactor) {
  //Returns the maximum payload length in bytes for each spreading factor. 
  uint8_t maxPayloadLength;
  switch (spreadingFactor)
  {
  case 7:
    maxPayloadLength = 222;  
    break;
  case 8:
    maxPayloadLength = 222;  
    break;
  case 9:
    maxPayloadLength = 115;  
    break;
  case 10:
    maxPayloadLength = 51;  
    break;
  case 11:
    maxPayloadLength = 51;  
    break;
  case 12:
    maxPayloadLength = 51;
    break;
  }
  return maxPayloadLength;
}

float getPackageTime(uint8_t spreadingFactor, uint32_t bandwidth, uint8_t payloadLength) {
  // See this page for formulas https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
  // Returns the minimum interval between packages of the given specification in ms
  // Use this value for setting a minimum delay after transmission
  // Duty cycle restrictions are 1% in EU868 band, hence packageTime is packageAirTime * 100
  uint8_t codingRate = 1;
  uint8_t preableSymb = 8;
  uint16_t payloadSymb = 8 + ((8 * payloadLength - 4 * spreadingFactor + 28 + 16) / (4 * spreadingFactor)) * (codingRate + 4);
  uint16_t packageSymb = preableSymb + payloadSymb;
  Serial.println(packageSymb);
  float timeSymb = pow(2, spreadingFactor) / (bandwidth/1000);
  float packageAirTime = packageSymb * timeSymb;
  float packageTime = packageAirTime * 100;
  return packageTime;
}

void parseCayenne() {
  //todo
}

void loop() {
  // If no error generate next package
  if (err >= 0) {
    // If telegram all sent, make new MBus request
    if (usedBytes > MBusSize) {
      // Make M-bus request, result is stored in dataList
      Serial.println("requesting M-Bus address:");
      Serial.println(addressList.get(addressNumber));
      mbusRequest(addressList.get(addressNumber));
      modem.setPort(addressList.get(addressNumber)); 
      // Iterate for next request
      addressNumber++;
      usedBytes = 0;
      // Reset number if end of list reached
      if (addressNumber >= addressList.size()) {
        addressNumber = 0;
      }
    }
    // Empty packet variable
    for (int i = 0; i < sizeof(packet); i++) {
      packet[i] = {0};
    }

    for(int i = 0; i < maxPacketSize; i++) {
      packet[i] = dataList2.get(0);
      dataList2.remove(0);
      usedBytes++;
    }
  }

  Serial.println("packet is: ");
  for (int i = 0; i < sizeof(packet); i++) {
    Serial.print(packet[i]);
  }
  // Clear data packet and reset error  
  modem.beginPacket();

  // Print into data packet
  modem.write(packet, sizeof(packet));

  // Send data packet
  err = modem.endPacket(false);
  Serial.println(err);
  if (err > 0) {
      Serial.println("Message sent correctly!");
    } else {
      Serial.println("Error sending message :( retrying");
    } 
  for (int i = 0; i < 20; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
}
