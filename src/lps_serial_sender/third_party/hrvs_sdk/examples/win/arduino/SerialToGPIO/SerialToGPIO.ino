const byte SYNC_WORD = 0xFF;

const byte MSG_SIZE = 2;
byte receivedBytes[MSG_SIZE];

// set pin numbers 5-GPIO_SYNC_IN
const int GPIO_SYNC_IN = 5;

/**
 * @brief Starts serial communication. Initializes used pins and interrupts.
 * 
 */
void setup() 
{
  Serial.begin(115200, SERIAL_8N1);
  
  // Initialize Input pins.
  pinMode(GPIO_SYNC_IN, OUTPUT);

  // Set default value to 0
  writeInputGpio(0, GPIO_SYNC_IN);
}

/**
 * @brief Program main loop.
 * 
 */
void loop()
{
  recvBytes();
}
/**
 * @brief Reads incomming serial communication. Parses the message and call writeInputGpio method.
 * 
 */
void recvBytes()
{
  byte recv;
  static byte numBytesReceived = 0;

  while(Serial.available() > 0)
  {
    recv = Serial.read();
    receivedBytes[numBytesReceived] = recv;
    numBytesReceived++;

    if(numBytesReceived == MSG_SIZE)
    {
      if(receivedBytes[0] == SYNC_WORD && (receivedBytes[1] == 0 || receivedBytes[1] == 1))
      { 
        writeInputGpio(receivedBytes[1], GPIO_SYNC_IN);
        byte replyMsg[] = {0xFF, digitalRead(GPIO_SYNC_IN)};
        Serial.write(replyMsg, sizeof(replyMsg));
      }


      clearBuffer();
      numBytesReceived = 0;
    }
  }
}

/**
 * @brief Turns on/off GPIO.
 * 
 * @param state State to write, either 0x00 or 0x01.
 * @param pin Affected pin.
 */
void writeInputGpio(byte state, const int pin)
{
  if(state == 0x01)
  {
    digitalWrite(pin, HIGH);
  }
  else if(state == 0x00)
  {
    digitalWrite(pin, LOW);
  }
}

/**
 * @brief Cleans serial read buffer.
 * 
 */
void clearBuffer()
{
  for (byte n = 0; n < MSG_SIZE; n++) {
    receivedBytes[n] = 0;
  }
}