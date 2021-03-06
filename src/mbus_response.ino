#include <HardwareSerial.h>
 
bool mbus_get_response(byte *pdata, unsigned char len_pdata) {
  byte bid = 0;             // current byte of response frame
  byte bid_end = 255;       // last byte of frame calculated from length byte sent
  byte bid_checksum = 255;  // checksum byte of frame (next to last)
  byte len = 0;
  byte checksum = 0;
  bool long_frame_found = false;
  bool complete_frame  = false;
  bool frame_error = false;
 
  unsigned long timer_start = millis();
  while (!frame_error && !complete_frame && (millis()-timer_start) < MBUS_TIMEOUT)
  {
    while (customSerial->available()) {
      byte received_byte = (byte) customSerial->read();
      //Serial.println(received_byte);  //FR
      // Try to skip noise
      if (bid == 0 && received_byte != 0xE5 && received_byte != 0x68) {
        if (DEBUG) Serial.print(F(">"));
        continue;
      }
      
      if (bid > len_pdata) {
        Serial.print("mbus: error: frame length exceeded variable size of ");
        Serial.println(len_pdata);
        return MBUS_BAD_FRAME;
      }
      pdata[bid] = received_byte;

      // Single Character (ACK)
      if (bid == 0 && received_byte == 0xE5) {
        if (DEBUG) Serial.println(F("mbus: single character (ack)"));
        return MBUS_GOOD_FRAME;
      }
      
      // Long frame start
      if (bid == 0 && received_byte == 0x68) {
        if (DEBUG) Serial.println(F("mbus: start long frame"));
        long_frame_found = true;
      }

      if (long_frame_found) {
        // 2nd byte is the frame length
        if (bid == 1) {
          len = received_byte;
          bid_end = len+4+2-1;
          bid_checksum = bid_end-1;
        }
            
        if (bid == 2 && received_byte != len) {                          // 3rd byte is also length, check that its the same as 2nd byte
          if (DEBUG) Serial.println(F("mbus: frame length byte mismatch"));
          frame_error = true;
        }
        if (bid == 3 && received_byte != 0x68) {        ;                // 4th byte is the start byte again
          if (DEBUG) Serial.println(F("mbus: missing second start byte in long frame"));
          frame_error = true;
        }
        if (bid > 3 && bid < bid_checksum) checksum += received_byte;    // Increment checksum during data portion of frame
        
        if (bid == bid_checksum && received_byte != checksum) {          // Validate checksum
          if (DEBUG) Serial.println(F("mbus: frame failed checksum"));
          frame_error = true;
        }
        if (bid == bid_end && received_byte == 0x16) {                   // Parse frame if still valid
          complete_frame  = true;
        }
      }
      bid++;
    }
  }

  if (len > 0) {   //FR   //complete_frame && !frame_error
    return MBUS_GOOD_FRAME;
  } else {
    return MBUS_BAD_FRAME;
  }
}

// Spire 280T-S returns decimal values as hex, right to left for multi-byte values
// start_byte is first data byte / most significant data byte
long get_spire_value(byte *pdata, unsigned int start_byte, unsigned char num_bytes) {
  String val;
  for (uint8_t i = start_byte - 1 + num_bytes; i >= start_byte; i--) {
    // Serial.println(String(pdata[i], HEX));
    char hex[4];
    sprintf(hex, "%.2X", pdata[i]); // ensure leading zeros are included
    val = val + hex;
  }
  return val.toInt();
}

// Prints a whole response as a string for debugging
int print_bytes(byte *bytes, unsigned char len_bytes) {
  dataList.clear();
  dataList2.clear();
  int size = bytes[1];
  for (int i = 0; i < len_bytes; i++) {
    Serial.print(String(bytes[i], HEX));
    Serial.print(F(" "));

    // Pad data if needed
    String tempString = String(bytes[i], HEX);
    if (tempString.length() < 2) {
      tempString = "0" + tempString;
    }
    dataList.add(tempString); //FR
    dataList2.add(bytes[i]);
  } 
  Serial.println();
  return size;
}