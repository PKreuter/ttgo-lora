

/**
2 ERROR
3 WARN
4 INFO
5 DEBUG
**/

// 
int MIN_LOG_LEVEL = 4;        // 4 = INFO


// debug   
const uint8_t debugPin = 36; 
bool debugMode = LOW;


String LOG_LEVEL_NAMES[] = {"OFF", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL"};

// is logger enabled
void isDebugEnabled() {
  pinMode(debugPin, INPUT);
  if( digitalRead(debugPin) == LOW) {
    MIN_LOG_LEVEL = 0;
  }
}


//
void debugOutput(String text, int logLevel) {
  if (MIN_LOG_LEVEL >= logLevel) {
    Serial.println("["+LOG_LEVEL_NAMES[logLevel]+ "] " + text);
  }
}

