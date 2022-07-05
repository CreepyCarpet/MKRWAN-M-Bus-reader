// Todo heartbeat retry logic
// Todo Enable variable downlink length
// Todo Event scheduler with timed priority
// Todo ADR handling, check downlinks for ADR req and comply with the req

#include <MKRWAN.h>
#include <HardwareSerial.h>
#include <LinkedList.h>
#include <math.h>
#include <secrets.h>
#include <arduino-timer.h>
#include <TimeLib.h>
using namespace std;

HardwareSerial *customSerial;
time_t timeNow = now();
time_t lastSync;

class MBus { // Class for the M-Bus connection
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

    void setTimeout(uint16_t timeOut) { // Set the timeout on the serial connection with the M-Bus-devices. Suggested value is 1000 ms
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

    void scan() { // Scan for M-bus-devices and put found addresses into addressList. Clears addresslist and known devices INCLUDING SAVED RECIPIES 
      addressList.clear(); // clears known address list
      devices.clear(); //clears known device list
      unsigned long timer_start = 0;
      // Scans all M-Bus addresses for connected M-Bus-devices
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
            Serial.println("The following addresses are in use by M-Bus devices:");
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
    bool hasRecipe; // True if a custom recipe has been created
    
    MBusDevice(uint8_t address) { // Constructor
      // Sets the M-Bus-address for the device
      deviceAddress = address;
      hasRecipe = false;
      return;
    }

    LinkedList<byte> getRawPackage() { // Get a full, raw M-Bus-package
      mbusRequest(deviceAddress);
      return rawPackage;
    }

    LinkedList<byte> getPackage() { // Make request and then return customPackage based on the recipe
      createPackage(deviceAddress, recipe);
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

    void setSendCustomUplinks(bool setting) { // If set to false device will send no custom uplinks
      if (hasRecipe == true) {
        sendCustomUplinks = setting;
      }
      else {
        Serial.println("Device has no recipe, cannot create custom uplinks");
      }
      return;
    }

    void setAddress(uint8_t downLinkAddress) { // Overwrites the known device address, use for debugging
      deviceAddress = downLinkAddress;
    }

  private:
    uint8_t deviceAddress; // Address of the M-Bus device
    bool sendCustomUplinks; // Sends custom uplinks at given interval if true
    LinkedList<bool> recipe; // Each member is true if the corresponding byte is should be included in the customPackage
    LinkedList<byte> rawPackage = LinkedList<byte>(); // Last recieved raw M-Bus package
    LinkedList<byte> customPackage = LinkedList<byte>(); // customPackage of last rawPackage. Uses recipe to create from the rawPackage

    void createPackage(uint8_t address, LinkedList<bool> content) { // Makes M-Bus request and filters the desired content based on recipe
      // Make a request for a fresh mbus telegram
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
      byte mbusData[mbus.getDataSize()] = {0}; // Todo fix
      
      mbus_request_data(address);
      frameResult = mbus_get_response(mbusData, sizeof(mbusData));
      
      if (frameResult) {
        Serial.println(F("mbus: good frame: "));
      }
      else {
        Serial.println(F("mbus: bad frame: "));
      }

      // Copy bytes to local LinkedList using get_bytes function 
      rawPackage = get_bytes(mbusData, sizeof(mbusData));
      for (uint8_t i = 0; i < rawPackage.size(); i++) {
        Serial.print(rawPackage.get(i));
      }
      return;
    }
};

class LoRaWAN: public LoRaModem { // Child class of LoRaModem with additional functions. Everything LoRaWAN linked belongs in this class
  public:
    void randomOTAAJoin(String appKey, String appEui) { // Use deviceEUI and fluctuating analog value to generate random seed for random join wait if reset to prevent network congestion
      String deviceEUI = modem.deviceEUI();
      
      // Make deviceseed from deviceEUI
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

      // Combine analogRead with deviceseed to get a pseudo random seed 
      randomSeed(deviceSeed + analogRead(0));
      
      // Generate random wait time seeded with random seed
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

    uint8_t getSpreadingFactor() { // Returns the spreadingfactor value of the given datarate
      uint8_t dataRate = getDataRate();
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

    uint32_t getBandWidth() { // Returns the bandwidth of the given datarate
      uint8_t dataRate = getDataRate();
      uint32_t bandWidth;
      if (dataRate == 6) {
        bandWidth = 250000;
      }
      else {
        bandWidth = 125000;
      }
      return bandWidth;
    }

    uint8_t getPayloadMaxLength() { // Returns the maximum payload length in bytes for each spreading factor. 
      uint8_t spreadingFactor = getSpreadingFactor();
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

    uint32_t getPackageTime(uint8_t payloadLength) { // Returns the minimum interval between packages of the given specification in ms
      // See this page for formulas https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
      // Use this value for setting a minimum delay after transmission
      // Duty cycle restrictions are 1% in EU868 band, hence packageTime is packageAirTime * 100
      uint8_t spreadingFactor = getSpreadingFactor();
      uint32_t bandwidth = getBandWidth();

      uint8_t codingRate = 1;
      uint8_t preableSymb = 8;
      uint16_t payloadSymb = 8 + ((8 * payloadLength - 4 * spreadingFactor + 28 + 16) / (4 * spreadingFactor)) * (codingRate + 4);
      uint16_t packageSymb = preableSymb + payloadSymb;
      float timeSymb = pow(2, spreadingFactor) / (bandwidth/1000);
      float packageAirTime = packageSymb * timeSymb;
      float packageTime = packageAirTime * 100;
      // Cast float to uint32_t to signify package time in ms
      uint32_t packageTimeInt = int(packageTime) + 1;
      Serial.println(packageTime);
      return packageTimeInt;
    }
    
    void setNextPayload(LinkedList<byte> payload, uint8_t port) { // Set the content and port of the next payload.
      nextPayload = payload;
      nextPort = port;
      return;
    }

    uint8_t canSend() { // Todo, check if works. Checks the time until next allowed uplink, returns 0 if uplink is allowed
      uint8_t timeLeft; 
      uint32_t timeDiff = timeNow - lastUplink;
      if (timeDiff > lastPackageTime) {
        timeLeft = 0;
      }
      else {
        timeLeft = lastPackageTime - timeDiff;
      }
      return timeLeft;
    }

    void orderUplink(LinkedList<byte> payload, uint8_t port) { // Make uplink ASAP, checks the next available timeslot, prepares necessary variables and then waits if needed
      nextPort = port;
      nextPayload = payload;
      uint8_t waitTime = canSend();
      sleep(waitTime*1000);
      sendPayload();
    }

    uint8_t getDownlinkQueueLength() { // Returns the length of the downlink queue
      return downlinkQueue.size();
    }

    LinkedList<byte> getDownlink() { // Return oldest downlink from queue
      return downlinkQueue.shift();
    } 

  private:
    time_t lastUplink;
    uint32_t lastPackageTime; // Duty cycle time used by last payload in ms;
    LinkedList<byte> nextPayload = LinkedList<byte>(); // Content of the next scheduled payload
    uint8_t nextPort; // Port of the next scheduled package
    uint8_t offsetPort = 10; // Offset to add to port number to signify that packet is a part of a multipacket payload
    LinkedList<LinkedList<byte>> downlinkQueue = LinkedList<LinkedList<byte>>(); // Downlinks that are yet to be processed

    void sendPayload() { // Sends the scheduled payload. Calls recursively if payload is longer than the max length for a single packet. Saves downlinks in downlinkQueue
      beginPacket();
      // Add offset to port to keep track of package number
      nextPort = nextPort + offsetPort;
      // If the payload is larger than the maximum allowed for the given spreading factor split the packet up and send as multiple. 
      if (nextPayload.size() > getPayloadMaxLength()) {
        // Fill actual package to maximum length and delete the moved members
        for (uint8_t i = 0; i < getPayloadMaxLength(); i++) {
          write(nextPayload.shift());
        }
        setPort(nextPort);
        // Add airtime to lastPackageTime
        lastPackageTime = lastPackageTime + getPackageTime(getPayloadMaxLength());
        // Send packet
        endPacket();
        // Wait for potential downlink
        sleep(1000);
        // Check if there is a downlink
        if (available()) {
          // Parse the payload and add it to downlinkQueue
          downlinkQueue.add(readDownlink());
        }
        // Calls recursively to finish the remaining parts of the payload
        sendPayload();
      }

      // Last/single packet with payload
      else {
        // Fill packet with payload
        for (uint8_t i = 0; i < nextPayload.size(); i++) {
          write(nextPayload.shift());
        }
        // Set the port to show that packet is last/single packet
        setPort(nextPort);
        // Add airtime to lastPackageTime
        lastPackageTime = lastPackageTime + getPackageTime(nextPayload.size());
        // Send packet
        endPacket();
        // Take timestamp for lastUplink
        lastUplink = timeNow;
        // Wait for potential downlink
        sleep(1000);
        // Check if there is a downlink
        if (available()) {
          // Parse the payload and add it to downlinkQueue
          downlinkQueue.add(readDownlink());
        }
        return;
      }
    }
    
    LinkedList<byte> readDownlink() { // Reads content of a downlink and returns it as a LinkedList<byte>
      LinkedList<byte> downlink = LinkedList<byte>();
      parsePacket();
      while(available()) {
        downlink.add(read()); 
      }
      return downlink;
    }

};

class Scheduler { // Handles event scheduling.
  /* 
  Event scheduling hierachy:
  1.  Downlink responses
  2.  Heartbeats
  3.  Custom uplinks
  */
  public:
    MBus mbus;
    LoRaWAN lora;

    uint16_t setDeviceUplinkInterval(uint8_t device, uint16_t interval) { // Sets the uplink interval for a specific device, checks are in place to only allow valid interval settings. If no interval is configured for a device a new configuration will be saved. 
      uint16_t minimumInterval = 0;
      uint16_t returnValue = 0; // The highest of either interval or minimum interval
      // Check known devices for address
      for (uint8_t i =0; i < mbus.devices.size(); i++) {
        if (mbus.devices.get(i).getAddress() == device) {
          if (mbus.devices.get(i).hasRecipe == true) { // Check if the device has a custom recipe
            uint8_t payloadLength = mbus.devices.get(i).getRecipe().size();
            // Calculate total package time based on recipe given. 
            if (payloadLength > lora.getPayloadMaxLength()) {
              // recipe longer than single packet, calculating combined interval time
              while (payloadLength > lora.getPayloadMaxLength()) {
                minimumInterval = minimumInterval + lora.getPackageTime(lora.getPayloadMaxLength()); 
                payloadLength = payloadLength - lora.getPayloadMaxLength();
              }
              minimumInterval = minimumInterval + lora.getPackageTime(payloadLength);
            } 
            else {
              minimumInterval = lora.getPackageTime(payloadLength);
            }
            for (uint8_t i = 0; i < deviceSchedules.size(); i++) {
              // Check if device already has schedule. Delete and recreate if it does
              if (deviceSchedules.get(i).address == device) {
                deviceSchedules.remove(i);
              }
            }
            // Creating new schedule for device
            deviceSchedule newSched;
            newSched.address = device;
            if (minimumInterval > interval) {
                  newSched.interval = minimumInterval;
                  newSched.requestedInterval = interval;
                  returnValue = minimumInterval;
                }
                else {
                  newSched.interval = interval;
                  newSched.requestedInterval = interval;
                  returnValue = interval;
                }
            deviceSchedules.add(newSched);
            return returnValue;
          }
        }
        return returnValue;
      } 
    }

  private:
    struct deviceSchedule { // M-Bus-device schedule struct 
      uint8_t address;
      uint16_t requestedInterval;
      uint16_t interval;
      time_t lastUplink = timeNow;
    };
    LinkedList<deviceSchedule> deviceSchedules = LinkedList<deviceSchedule>(); // List of M-Bus-devices with scheduled custom uplinks
    struct heartBeat {
      uint32_t interval;
      time_t lastBeat;
      uint16_t beatNumber;
    };

    void parseDownLink() { // Parse downlink message
      LinkedList<byte> downlink = lora.getDownlink();
      uint8_t messageType = downlink.get(0);

      switch (messageType) {
        case 1: { // Reset device
          pinMode(resetPin, OUTPUT);
          delay(200);
          digitalWrite(resetPin, HIGH);
          Serial.println("Resetting device");
          delay(2000);
          digitalWrite(resetPin, LOW);
          break;
        }

        case 2: { // Force rejoin
          lora.randomOTAAJoin(appKey, appEui);
          break;
        }

        case 3: { // Get M-Bus device list
          lora.orderUplink(mbus.getAddressList(), 3);
          break;
        }

        case 4: { // Get raw M-Bus payload
          // Get M-Bus-address from the downlink request
          uint8_t address = downlink.get(1);

          for (uint8_t i = 0; i < mbus.getAddressList().size(); i++) {
            if (address == mbus.devices.get(i).getAddress()) {
              lora.orderUplink(mbus.devices.get(i).getRawPackage(), 4);
            }
          }
          break;
        }

        case 5: { // Get custom M-Bus payload
          uint8_t address = downlink.get(1);

          for (uint8_t i = 0; i < mbus.getAddressList().size(); i++) {
            if (address == mbus.devices.get(i).getAddress()) {
              lora.orderUplink(mbus.devices.get(i).getPackage(), 5);
            }
          }
          break;
        }

        case 6: { // Create custom M-Bus recipe
          uint8_t address = downlink.get(1);
          
          // Todo breakdown the downlink package into recipe
          LinkedList<bool> recipe;
          for (uint8_t i = 0; i < mbus.getAddressList().size(); i++) {
            if (address == mbus.devices.get(i).getAddress()) {
              mbus.devices.get(i).setRecipe(recipe);
              lora.orderUplink(mbus.devices.get(i).getPackage(), 5);
            }
          }
          break;
        }

        case 7: { // Remove custom M-Bus recipe
          uint8_t address = downlink.get(1);

          for (uint8_t i = 0; i < mbus.getAddressList().size(); i++) {
            if (address == mbus.devices.get(i).getAddress()) {
              mbus.devices.get(i).removeRecipe();
              // Todo, create ack
            }
          }
          break;
        }

        case 8: { // Set custom M-Bus payload uplink interval 
          uint8_t address = downlink.get(1);
          // Todo combine bytes to valid interval value. Check if this works
          uint16_t interval = 256*downlink.get(2) + downlink.get(3);
          uint16_t setInterval = setDeviceUplinkInterval(address, interval);
          if (setInterval != 0) { // If setting interval was successful then return the set value
            LinkedList<byte> payload = LinkedList<byte>();
            // Todo split up the interval correctly for the payload
            payload.add(setInterval);
            lora.orderUplink(payload, 8);
          } 
          break;
        }

        case 9: { // Heartbeat ACK
          // Todo extract time from heartbeat ACK to ackTime and set the time
          time_t ackTime;
          setTime(ackTime);
          lastSync = ackTime;
          break;
        }
        
        default: {

          break;
        }  
      }
      return;
    };

    uint32_t checkHeartbeat() { // Check the time until next heartbeat
      uint32_t time;

      return time; 
    }

  };

Scheduler schedule;

// LoRaWAN constant initialization
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;
String devAddr;
String nwkSKey;
String appSKey;

int resetPin = 0;
int err = 0;
int usedBytes = 0;
int addressNumber = 0;


void setup() {
  //Lora OTAA connection
  Serial.begin(115200);

  if (!schedule.lora.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };

  Serial.print("Your device EUI is: ");
  Serial.println(schedule.lora.deviceEUI());
  
  //Test area
  delay(5000);

  // Wait random time, then try to join
  schedule.lora.randomOTAAJoin(appKey, appEui);

  schedule.lora.setADR(true);

  schedule.lora.beginPacket();
  int err;
  // Print into data packet
  schedule.lora.print("Hi");

  // Send data packet
  err = schedule.lora.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
  } 

  //M-Bus serial initialization
  customSerial = &Serial1;
  customSerial->begin(schedule.mbus.getBaudRate(), SERIAL_8E1); // mbus uses 8E1 encoding
  delay(5000); // Delay to startup. Let the serial initialize, or we get a bad first frame
  schedule.mbus.scan(); // First scan to get addresses
  Serial.println("Scan finished, sending list of devices");

  
}

void loop() {

}