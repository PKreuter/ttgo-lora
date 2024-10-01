

#include "AESLib.h"
AESLib aesLib;

#define BUFFER_LIMIT 300
char cleartext[BUFFER_LIMIT] = {0};    // THIS IS BUFFER (FOR TEXT)
char ciphertext[2*BUFFER_LIMIT] = {0}; // THIS IS BUFFER (FOR BASE64-ENCODED ENCRYPTED DATA)


unsigned long loopcount = 0;



// KEY: NIjSqCYwgXxdnXBA, https://onlinetools.com/utf8/convert-utf8-to-bytes

// AES Encryption Key
byte aes_key[] = { 0x4e, 0x49, 0x6a, 0x53, 0x71, 0x43, 0x59, 0x77, 0x67, 0x58, 0x78, 0x64, 0x6e, 0x58, 0x42, 0x41 };

// General initialization vector (you must use your own IV's in production for full security!!!)
byte aes_iv[N_BLOCK] = { 0x79, 0x31, 0x44, 0x56, 0x63, 0x49, 0x72, 0x33, 0x6a, 0x65, 0x70, 0x71, 0x59, 0x42, 0x33, 0x39 };




/* non-blocking wait function */
void wait(unsigned long milliseconds) {
  unsigned long timeout = millis() + milliseconds;
  while (millis() < timeout) {
    yield();
  }
}


String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}


String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

// Generate IV (once)
void aes_init() {
  Serial.println("gen_iv()");
  aesLib.gen_iv(aes_iv);
  Serial.println("encrypt_impl()");
}

