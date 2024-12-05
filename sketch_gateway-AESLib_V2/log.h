



// 
int MIN_LOG_LEVEL = 5;        // 4 = INFO

// Mapping
/**
2 ERROR
3 WARN
4 INFO
5 DEBUG
**/
String LOG_LEVEL_NAMES[] = {"OFF", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL"};

// debug   
const uint8_t debugPin = 39; 
bool debugMode = LOW;



// Message at once
void debugOutput(String text, int logLevel) {
  if (MIN_LOG_LEVEL >= logLevel) {
    Serial.println("["+LOG_LEVEL_NAMES[logLevel]+ "] " + text);
  }
}


// Message in two parts
// part 1, without new line
void debugOutputP1(String text, int logLevel) {
  if (MIN_LOG_LEVEL >= logLevel) {
    Serial.print("["+LOG_LEVEL_NAMES[logLevel]+ "] " + text);
  }
}
// port2, with new line
void debugOutputP2(String text, int logLevel) {
  if (MIN_LOG_LEVEL >= logLevel) {
    Serial.println(" "+ text);
  }
}


// is logger enabled
void isDebugEnabled() {
  pinMode(debugPin, INPUT_PULLDOWN);
  debugMode = digitalRead(debugPin);
  if( digitalRead(debugPin) == LOW) {
    //MIN_LOG_LEVEL = 0;

    MIN_LOG_LEVEL = 5;
  
  }
  debugOutput("Debug mode: " +String(debugMode)+ " - Level: " +String(MIN_LOG_LEVEL), 4);
}

// It's just a simple class inherting from print
class debugPrint : public Print {
    uint8_t _debugLevel = 1;
  public:
    void setDebugLevel(byte newLevel) {
      _debugLevel = newLevel;
    }
    size_t write (uint8_t value)
    {
      if (_debugLevel)
      {
        Serial.write(value);
      }
      return 1;
    }
} Debug;
