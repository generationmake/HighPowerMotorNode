/*
 * UAVCAN node with HighPowerMotorFeatherWing, CANFeatherWing and Adafruit Feather M0
 *
 * Hardware:
 *   - HighPowerMotorFeatherWing
 *   - CANFeatherWing
 *   - Adafruit Feather M0
 *
 * Used Subject-IDs
 * Heartbeat - pub
 * 2001 - pub - Bit    - Emergency Switch Status
 * 2002 - pub - Real32 - InputVoltage (0.0V..52.8V)
 * 2101 - sub - Real32 - Speed A (-1.0..+1.0)
 * 2102 - sub - Real32 - Speed B (-1.0..+1.0)
 * 2011 - pub - Real32 - Speed feedback A (-1.0..+1.0)
 * 2012 - pub - Real32 - Speed feedback B (-1.0..+1.0)
 * 2021 - pub - Real32 - Current A (-10.0A..+10.0A)
 * 2022 - pub - Real32 - Current A (-10.0A..+10.0A)
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = A2;
static int const MKRCAN_MCP2515_INT_PIN = A3;
static CanardPortID const BIT_PORT_ID   = 1622U;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    spi_select        ();
void    spi_deselect       ();
uint8_t spi_transfer       (uint8_t const);
void    onExternalEvent    ();
bool    transmitCanFrame   (CanardFrame const &);
void    onReceiveBufferFull(CanardFrame const &);
void    onBit_1_0_Received (CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);

ArduinoUAVCAN uc(13, transmitCanFrame);

Heartbeat_1_0 hb;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
//  while(!Serial) { }

  /* Setup LED and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0::Health::NOMINAL;
  hb = Heartbeat_1_0::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the reception of Bit message. */
  uc.subscribe<Bit_1_0<BIT_PORT_ID>>(onBit_1_0_Received);
}

void loop()
{
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
    uc.publish(hb);
    prev = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc.transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

bool transmitCanFrame(CanardFrame const & frame)
{
  return mcp2515.transmit(frame);
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  uc.onCanFrameReceived(frame);
}

void onBit_1_0_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uc */)
{
  Bit_1_0<BIT_PORT_ID> const uavcan_led = Bit_1_0<BIT_PORT_ID>::deserialize(transfer);

  if(uavcan_led.data.value)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Received Bit: true");
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Received Bit: false");
  }
}
