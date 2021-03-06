// todo cleanup datalists after binary conversion
#include <MKRWAN.h>
#include <HardwareSerial.h>
#include <LinkedList.h>
#include <math.h>
#include <secrets.h>
#include <arduino-timer.h>

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
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;
String devAddr;
String nwkSKey;
String appSKey;
int maxPacketSize = 64;

int resetPin = 0;
int err = 0;
int usedBytes = 0;
int addressNumber = 0;
time_t unixTime; 
time_t lastPackageTime;

uint8_t lastPayloadLength;

void setup() {
  //Lora OTAA connection
  Serial.begin(115200);

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };

  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  
  //Test area
  delay(5000);

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
  Serial.println("Scan finished, sending list of devices");

  //Send list of M-Bus devices
  for(uint8_t i = 0; i < addressList.size(); i++) {
    packet[i] = addressList.get(i);
  }
  modem.beginPacket();
  modem.write(packet, sizeof(packet));
  err = modem.endPacket(false);
  Serial.println(err); 
  modem.
}

class UpLink {
  public:
    void Uplink(uint8_t address) {
      // Sets the M-Bus-address for the device
      addressMBus = address;
      return;
    }

    LinkedList<byte> makeUplink() {
      LinkedList<byte> uplink;
      LinkedList<byte> package;
      uplink.add(getAddress());
      package = getPackage();
      for (uint8_t i ;i < package.size(); i++) {
        uplink.add(package.get(i));
      }
      return uplink;
    }

    LinkedList<byte> getPackage() {
      // Return customPackage based on the recipe
      LinkedList<byte> package;
      package = createPackage(recipe);
      return package;
    }

    uint8_t getAddress() {
      // Gets the address of the custom uplink
      return addressMBus;
    }

    LinkedList<bool> getRecipe() {
      return recipe;
    }

    void setRecipe(LinkedList<bool> downlink) {
      // Creates custom recipe for the given device
      // Overwrites any old recipe
      recipe = downlink;
      return;
    }
    
  private:
    uint8_t addressMBus; // Address of the M-Bus device
    LinkedList<bool> recipe; // Each member shows if the corresponding byte is should be included in the customPackage

    LinkedList<byte> createPackage(LinkedList<bool> content) {
      // Makes M-Bus request and filters the desired content based on recipe
      // Returns the resulting package
      LinkedList<byte> rawPackage;
      LinkedList<byte> package;
      package = mbusRequest(addressMBus);
      for (uint8_t i; i < rawPackage.size(); i++) {
        if (recipe.get(i) = true) {
          package.add(rawPackage.get(i));
        }
      }
      return package;
    }
};

void loraOTAAJoin(String appKey, String appEui) {
  // Use deviceEUI and fluctuating analog value to generate random seed for random join wait if reset to prevent network congestion.
  String deviceEUI = modem.deviceEUI();
  uint64_t deviceSeed = 1; 
  for(uint8_t i = 0; i < deviceEUI.length(); i++) {
    char thisChar = deviceEUI.charAt(i);

    for(int j = 0; j < 8; j++) {
      byte bit = bitRead(thisChar, j);
      if (bit == true) {
        deviceSeed = deviceSeed*2;
      }
    }
  }
  Serial.println("deviceSeed is:");
  Serial.println(deviceSeed);
  randomSeed(deviceSeed + analogRead(0));
  uint32_t randomInt = random(120);
  for (uint32_t i = 0; i < randomInt; i++) {
    delay(1000);
    Serial.print(".");
  }

  // Attempt OTAA connection
  bool connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (randomInt < 3600) {
      randomInt = randomInt*2;
      for (uint32_t i = 0; i < randomInt; i++) {
        delay(1000);
        Serial.print(".");
      }
      connected = modem.joinOTAA(appEui, appKey);
    }
  }
  return;
}

void parseDownLink(byte downLinkMessage[64]) {
  // Parses downlink messages and makes relevant calls 
  uint8_t messageType = downLinkMessage[0];
  
  switch (messageType)
  {
    case 1: {
      // reset the device
      resetArduino();
      break;
    }
    case 2: {
      // Set uplink interval and return the uplink interval as uplink
      uint32_t uplinkInterval = downLinkMessage[1] + downLinkMessage[2] + downLinkMessage[3] + downLinkMessage[4];
      uint32_t setInterval = setUplinkInterval(uplinkInterval);
      break;
    }
    case 3: {
      // Get M-bus device addresses and return as list
      getMBusDevices();
      break;
    }
    case 4: {
      // Get full M-Bus package from specific device
      break;
    }
    case 5: {
      // Create or overwrite new uplink recipe
      break;
    }
    case 6: {
      // Delete uplink recipe
      break;
    }
    case 7: {
      // Get list of uplink class members
      break;
    }
    default: {

      break;
    }  
  }
};

void mbusRequest(byte address) { // TODO Needs to change to a returning function. Return the package as a linkedlist of bytes. 
  // Requests an mbus package on the given address

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

void getMBusDevices(bool scan) {
  // Scan for M-Bus devices if scan is true
  // Return list of M-Bus devices
  if (scan == true) {
    mbus_scan();
  }
  
}

void createHeartbeat() {
  //Create a heartbeat message, should be the periodical confirmed Uplink. Downlink should contain Unix time to sync
  LinkedList<byte> heartbeatPackage;

}

uint32_t setUplinkInterval(uint32_t interval) {
  // Allows setting a minimum uplink interval
  // Checks if the chosen value lies within the allowed airtime if not the interval defaults to the lowest allowed value
  uint8_t dataRate = modem.getDataRate();
  uint32_t bandWidth = getBandWidth(dataRate);
  uint8_t spreadingFactor = getSpreadingFactor(dataRate);
  float minPackageTime = getPackageTime(spreadingFactor, bandWidth, 1);
  if (interval < minPackageTime) {
    interval = minPackageTime;
    Serial.println("Requested interval lower than shortest allowed, setting as shortest allowed");
  }
  Serial.println(interval);
  return interval;
}

uint8_t getSpreadingFactor(uint8_t dataRate) {
  // Gets the spreadingfactor value of the given datarate
  uint8_t spreadingFactor;
  
  if (dataRate == 6) {
    spreadingFactor = 7;
  }
  else {
    uint8_t spreadingFactor = 12 - dataRate;
  }
  Serial.println(spreadingFactor);
  return spreadingFactor;
}

uint32_t getBandWidth(uint8_t dataRate) {
  // Gets the bandwidth of the given datarate
  uint32_t bandWidth;
  if (dataRate == 6) {
    bandWidth = 250000;
  }
  else {
    bandWidth = 125000;
  }
  return bandWidth;
}

uint8_t getPayloadMaxLength(uint8_t spreadingFactor) {
  // Returns the maximum payload length in bytes for each spreading factor. 
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
  float timeSymb = pow(2, spreadingFactor) / (bandwidth/1000);
  float packageAirTime = packageSymb * timeSymb;
  float packageTime = packageAirTime * 100;
  Serial.println(packageTime);
  return packageTime;
}

void scheduleNextAction() {
  // Todo
  // Schedules the next function using Arduino-Timer
  //
}

/* uint32_t canSend() {
  // Returns time in seconds before device is allowed to send next package. If 0 sending is allowed
  // Todo, needs testing
  int dataRate = modem.getDataRate();
  uint16_t waitTime = getPackageTime(getSpreadingFactor(dataRate), getBandWidth(dataRate), lastPayloadLength) / 1000; // Get the time the previous package needs for duty cycle restrictions in seconds
  time_t time = now();
  time_t waitedTime = time - lastPackageTime;

  uint16_t waitedTimeSeconds= waitedTime.hour()/3600 + waitedTime.minute()/60 + waitedTime.second();
  
  int remainingWaitTime = waitTime - waitedTimeSeconds;
  if (remainingWaitTime > 0) {
    return remainingWaitTime;
  } 
  else {
    return 0;
  }
} */

LinkedList<UpLink> getUplinkClassMembers() {
  //returns a list addresses for devices with custom uplink recipes
  // What returntype to use???
}

void resetArduino() {
  // Reset the device using the reset pin
  pinMode(resetPin, OUTPUT);
  delay(200);
  digitalWrite(resetPin, HIGH);
  Serial.println("Resetting device");
  delay(2000);
  digitalWrite(resetPin, LOW);
}

void rejoin() {
  // Rejoin the TTS instance. Can be requested from the TTS to give an quick 
  // Todo
}

void parseCayenne() {
  // Todo
}

void loop() {



  /*
  // Old looping function, naively collects data from M-Bus devices and sends it via 64 byte payloads as uplinks

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
  */
}