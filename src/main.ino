// todo cleanup datalists after binary conversion
// Todo Linked list as packet list?
// Todo heartbeat retry logic
// Todo make LoRa specific functions a child class of LoRaModem to simplify program
#include <MKRWAN.h>
#include <HardwareSerial.h>
#include <LinkedList.h>
#include <math.h>
#include <secrets.h>
#include <arduino-timer.h>
using namespace std;

HardwareSerial *customSerial;
char packet[64];

class MBus {
  public:
    LinkedList<MBusDevice> devices = LinkedList<MBusDevice>(); // Linked list for all the connected M-Bus-devices 

    void setBaud(uint8_t baudRate) { // Set the baudRate of the serial connection with the M-Bus-devices 
      /*
      List of possible baudrates:
      0 = 300 baud
      1 = 600 baud
      2 = 1200 baud
      3 = 2400 baud
      4 = 4800 baud
      5 = 9600 baud
      6 = 19200 baud
      7 = 38400 baud
      */

      switch (baudRate)
      {
      case 0:
        MBUS_BAUD_RATE = 300;
        break;
      
      case 1:
        MBUS_BAUD_RATE = 600;
        break;
      
      case 2:
        MBUS_BAUD_RATE = 1200;
        break;
      
      case 3:
        MBUS_BAUD_RATE = 2400;
        break;
      
      case 4:
        MBUS_BAUD_RATE = 4800;
        break;
      
      case 5:
        MBUS_BAUD_RATE = 9600;
        break;
      
      case 6:
        MBUS_BAUD_RATE = 19200;
        break;
      
      case 7:
        MBUS_BAUD_RATE = 38400;
        break;
      
      default:
        break;
      }
      return;
    }

    void setTimeout(uint16_t timeOut) { // Set the timeout on the serial connection with the -Bus-devices. Suggested value is 1000 ms
      MBUS_TIMEOUT = timeOut;
      return;
    }

    void setDataSize(uint8_t size) { // Set the maximum datasize of the M-Bus-payloads. Usually set to 256 bytes
      MBUS_DATA_SIZE = size;
      return;
    }

    uint32_t getBaudRate() { // Get the baudRate of the serial connection with the M-Bus-devices
      return MBUS_BAUD_RATE;
    }

    uint16_t getTimeOut() { // Get the timeout of the serial connection with the M-Bus-devices
      return MBUS_TIMEOUT;
    }

    uint8_t getDataSize() { // Get the maximum datasize of the M-Bus-payloads
      return MBUS_TIMEOUT;
    }

    LinkedList<uint8_t> getAddressList() { // Return list of known M-Bus-devices
      return addressList;
    }

    void mbusScan() { // Scan for M-bus-devices and put found addresses into addressList
      unsigned long timer_start = 0;
      for (uint8_t address = 1; address <= 250; address++) {
        for (uint8_t retry = 0; retry <= 1; retry++) {  
          Serial.print("Scanning address: ");
          Serial.println(address);
          mbus_application_reset(address);
          timer_start = millis();
          while (millis()-timer_start<256) {
            if (customSerial->available()) {
              byte val = customSerial->read();
              addressList.add(address);
              devices.add(address);
            }
            // Print the connected addresses
            Serial.println("The following addresses have connected M-Bus devices:");
            for (uint8_t i = 0; i < sizeof(devices); i++) {
                Serial.println(devices.get(i).getAddress());
            }
          }
        }
      }
    return;
    }

  private:
    bool DEBUG = true;
    uint32_t MBUS_BAUD_RATE = 2400;
    uint16_t MBUS_TIMEOUT = 1000; // milliseconds
    uint8_t MBUS_DATA_SIZE = 255;

    LinkedList<uint8_t> addressList = LinkedList<uint8_t>(); // Linked list for all the connected M-Bus-devices 
};

class MBusDevice { // Objects of this class are tied to specific M-Bus addresses. Each object can create a custom payload from the M-Bus-device based on a given recipe. Recipes can be given via downlinks
  public:
    MBusDevice(uint8_t address) { // Constructor
      // Sets the M-Bus-address for the device
      deviceAddress = address;
      return;
    }

    LinkedList<byte> getRawPackage() { // Get a full, raw M-Bus-package
      // Todo
      mbusRequest(deviceAddress);
      LinkedList<byte> package = rawPackage;
      return package;
    }

    LinkedList<byte> getPackage() { // Make request and then return customPackage based on the recipe
      createPackage(deviceAddress, recipe, rawPackage);
      return customPackage;
    }

    uint8_t getAddress() { // Gets the address of the device
      return deviceAddress;
    }

    LinkedList<bool> getRecipe() { // Get a the recipe for custom packages
      // if no recipe is present return errorList with only single false member
      if (hasRecipe == false) {
        LinkedList<bool> errorList = LinkedList<bool>();
        errorList.add(false);
        return errorList;
      }
      else {
        return recipe;
      }
    }

    void setRecipe(LinkedList<bool> downlinkRecipe) { // Set a custom recipe for the device
      // Overwrites any old recipe
      recipe = downlinkRecipe;
      hasRecipe = true;
      return;
    }
    
    void removeRecipe() { // Clears the custom recipe for the device
      recipe.clear();
      bool hasRecipe = false;
      return;
    }

    void setAddress(uint8_t downLinkAddress) { // Overwrites the known device address, use for debugging
      deviceAddress = downLinkAddress;
    }

  private:
    uint8_t deviceAddress; // Address of the M-Bus device
    bool hasRecipe = false; // True if a custom recipe has been created
    LinkedList<bool> recipe; // Each member is true if the corresponding byte is should be included in the customPackage
    LinkedList<byte> rawPackage = LinkedList<byte>(); // Last recieved raw M-Bus package
    LinkedList<byte> customPackage = LinkedList<byte>(); // customPackage of last rawPackage. Uses recipe to create from the rawPackage

    void createPackage(uint8_t address, LinkedList<bool> content, LinkedList<byte> Mbuspackage) { // Makes M-Bus request and filters the desired content based on recipe
      // Make a request for the raw package
      mbusRequest(address);

      // Filter the data of the raw package to create the custom package. Uses the given recipe
      for(uint8_t i = 0; i < recipe.size(); i++) {
        if (recipe.get(i) == true) {
          customPackage.add(rawPackage.get(i));
        }
      }
      return;
    }

    void mbusRequest(uint8_t address) { // Request a M-Bus-telegram for this device. Saves the content of the request in rawPackage. 
      bool frameResult;
      byte mbusData[mbus.getDataSize()] = {0};
      
      mbus_request_data(address);
      frameResult = mbus_get_response(mbusData, sizeof(mbusData));
      
      if (frameResult) {
        Serial.println(F("mbus: good frame: "));
      }
      else {
        Serial.println(F("mbus: bad frame: "));
      }

      for (int i = 0;i < dataList2.size(); i++) {
        Serial.print(dataList2.get(i));
      }
      return;
    }
};

MBus mbus;

// LoRaWAN constant initialization
LoRaModem modem;
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;
String devAddr;
String nwkSKey;
String appSKey;
int maxPacketSize = 64;
#define HEARTBEAT 1
#define MBUS_DEVICE_LIST 2
#define SET_PACKAGE_TIME 3
#define UPLINK_CLASS_MEMBERS 4
#define CUSTOM_PAYLOAD 5
#define FULL_PAYLOAD 6

int resetPin = 0;
int err = 0;
int usedBytes = 0;
int addressNumber = 0;

time_t unixTime; 
time_t lastPackageTime;
uint32_t heartbeatInterval;
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
  customSerial->begin(mbus.getBaudRate(), SERIAL_8E1); // mbus uses 8E1 encoding
  delay(5000); // Delay to startup. Let the serial initialize, or we get a bad first frame
  mbus_scan(); // First scan to get addresses
  Serial.println("Scan finished, sending list of devices");

  //Send list of M-Bus devices
  for(uint8_t i = 0; i < mbus.devices.size(); i++) {
    packet[i] = mbus.devices[i].getAddress();
  }
  modem.beginPacket();
  modem.write(packet, sizeof(packet));
  err = modem.endPacket(false);
  Serial.println(err); 
}

void loraOTAAJoin(String appKey, String appEui) { // Use deviceEUI and fluctuating analog value to generate random seed for random join wait if reset to prevent network congestion
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

void parseDownLink(byte downLinkMessage[64]) { // Parses downlink messages and makes relevant calls 
  uint8_t messageType = downLinkMessage[0];
  
  /* 
  Content of downlink message
  Byte 0 - request type
  Byte 1 - M-Bus address if relevant
  */
  switch (messageType)
  {
    case 1: {
      // reset the device
      resetArduino();
      break;
    }
    case 2: {
      // Set uplink interval and return the uplink interval as uplink
      uint32_t uplinkInterval = downLinkMessage[1] + downLinkMessage[2] + downLinkMessage[3] + downLinkMessage[4]; // TODO
      uint32_t setInterval = setUplinkInterval(uplinkInterval);
      break;
    }
    case 3: {
      // Get M-bus device addresses and return as list
      break;
    }
    case 4: {
      // Get full M-Bus package from specific device
      break;
    }
    case 5: {
      // Create or overwrite new uplink recipe
      // Content: uint8_t address, bool recipeList[]

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

void mbusRequest(byte address) { // TODO: Return the package as a linkedlist of bytes. Test how responses are returned 
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

void setHeartbeatInterval(uint32_t interval) { // Set heartbeat interval. Value in seconds. Minimum value is at 900 seconds
  if (interval > 900) {
    heartbeatInterval = interval;
  }
  else {
    heartbeatInterval = 900;
  }
  return;
}

LinkedList<byte> createHeartbeat() { // TODO: test. Create a heartbeat message, should be the periodical confirmed Uplink. Resulting downlink should echo the heartbeat number and contain Unix time to sync
  LinkedList<byte> payload;
  payload.add(HEARTBEAT);
  char binaryInterval[4];
  memcpy(binaryInterval, &heartbeatInterval, sizeof(heartbeatInterval));
  for (uint8_t i = 0; i > sizeof(binaryInterval); i++) {
    payload.add(binaryInterval[i]);
  }

  return payload;
}

uint32_t setUplinkInterval(uint32_t interval) { // Allows setting a minimum uplink interval. Checks if the chosen value lies within the allowed airtime if not the interval defaults to the lowest allowed value
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

uint8_t getSpreadingFactor(uint8_t dataRate) { // Gets the spreadingfactor value of the given datarate
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

uint32_t getBandWidth(uint8_t dataRate) { // Gets the bandwidth of the given datarate
  uint32_t bandWidth;
  if (dataRate == 6) {
    bandWidth = 250000;
  }
  else {
    bandWidth = 125000;
  }
  return bandWidth;
}

uint8_t getPayloadMaxLength(uint8_t spreadingFactor) { // Returns the maximum payload length in bytes for each spreading factor. 
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

float getPackageTime(uint8_t spreadingFactor, uint32_t bandwidth, uint8_t payloadLength) { // Returns the minimum interval between packages of the given specification in ms
  // See this page for formulas https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
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

void scheduleNextAction() { // Schedules the next function using Arduino-Timer
  // Todo
  timer_create_default();
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

LinkedList<uint8_t> getUplinkClassMembers() { // TODO: Returns a list with addresses for devices with custom uplink recipe
  LinkedList<uint8_t> list = LinkedList<uint8_t>();
  for (uint8_t i = 0; i < deviceList.size(); i++) {
    list.add(deviceList.get(i).getAddress()); 
  }
  return list; 
}

void resetArduino() { // Reset the device using the reset pin
  pinMode(resetPin, OUTPUT);
  delay(200);
  digitalWrite(resetPin, HIGH);
  Serial.println("Resetting device");
  delay(2000);
  digitalWrite(resetPin, LOW);
}

void reJoin() { // Rejoin the TTS instance. Can be requested from the TTS to ensure the device is reachable without waiting for a heartbeat
  loraOTAAJoin(appKey, appEui);
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