
/*
  Copyright (C) Duncan Greenwood 2023 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming
      ACAN2515 - for MCP2515 CAN controller
*/

// 3rd party libraries
#include <Streaming.h>

// MLCB library header files
#include <MLCB2515.h>               // CAN controller and MLCB class
#include <MLCBSwitch.h>             // pushbutton switch
#include <MLCBLED.h>                // MLCB LEDs
#include <MLCBConfig.h>             // module configuration
#include <MLCBParams.h>             // MLCB parameters

// constants
const byte MAJOR_VERSION = 1;       // code major version
const char MINOR_VERSION = 0;       // code minor version
const byte BUILD_VERSION = 0;       // code build
const byte MODULE_ID = 99;          // MLCB module type

const byte LED_GRN = 4;             // MLCB green SLiM LED pin
const byte LED_YLW = 5;             // MLCB yellow FLiM LED pin
const byte SWITCH0 = 6;             // MLCB push button switch pin

// MLCB objects
MLCBConfig module_config;           // configuration object
MLCB2515 MLCB(&module_config);      // MLCB object, specific to the CAN hardware in use
MLCBLED ledGrn, ledYlw;             // two LED objects
MLCBSwitch pb_switch;               // switch object

// module name, must be 7 characters, space padded.
unsigned char mname[7] = { 'E', 'M', 'P', 'T', 'Y', ' ', ' ' };

// forward function declarations
void eventhandler(byte, CANFrame *);
void framehandler(CANFrame *);
void processSerialInput(void);
void printConfig(void);

//
/// setup MLCB - runs once at power on from setup()
//
void setupMLCB() {

  // set config layout parameters
  module_config.EE_NVS_START = 10;
  module_config.EE_NUM_NVS = 10;
  module_config.EE_EVENTS_START = 50;
  module_config.EE_MAX_EVENTS = 32;
  module_config.EE_NUM_EVS = 1;
  module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

  // initialise and load configuration
  module_config.setEEPROMtype(EEPROM_INTERNAL);
  module_config.begin();

  Serial << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
  Serial << F(", NN = ") << module_config.nodeNum << endl;

  // set module parameters
  MLCBParams params(module_config);
  params.setVersion(MAJOR_VERSION, MINOR_VERSION, BUILD_VERSION);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to MLCB
  MLCB.setParams(params.getParams());
  MLCB.setName(mname);

  // show code version and copyright notice
  printConfig();

  // set MLCB LED pins and assign to MLCB
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);
  MLCB.setLEDs(ledGrn, ledYlw);

  // initialise MLCB switch and assign to MLCB
  pb_switch.setPin(SWITCH0, LOW);
  pb_switch.run();
  MLCB.setSwitch(pb_switch);

  // power-on self-test
  if (pb_switch.isPressed()) {
    // do any self-tests here
  }

  // register our MLCB event handler, to receive event messages of learned events
  MLCB.setEventHandler(eventhandler);

  // register our CAN frame handler, to receive *every* CAN frame
  MLCB.setFrameHandler(framehandler);

  // set MLCB LEDs to indicate the current mode
  MLCB.indicateMode(module_config.FLiM);

  // configure and start CAN bus and MLCB message processing
  MLCB.setNumBuffers(2, 1);      // more buffers = more memory used, fewer = less
  MLCB.setOscFreq(16000000UL);   // select the crystal frequency of the CAN module
  MLCB.setPins(10, 2);           // select pins for CAN bus CE and interrupt connections

  if (!MLCB.begin()) {
    Serial << F("> error starting MLCB") << endl;
  }
}

//
/// setup - runs once at power on
//

void setup() {

  Serial.begin (115200);
  Serial << endl << endl << F("> ** MLCB Arduino basic example module ** ") << __FILE__ << endl;

  setupMLCB();

  // end of setup
  Serial << F("> ready") << endl << endl;
}

//
/// loop - runs forever
//

void loop() {

  //
  /// do MLCB message, switch and LED processing
  //

  MLCB.process();

  //
  /// process console commands
  //

  processSerialInput();

  //
  /// bottom of loop()
  //
}

//
/// user-defined event processing function
/// called from the MLCB library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(byte index, CANFrame *msg) {

  // as an example, display the opcode and the first EV of this event
  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;
  Serial << F("> EV1 = ") << module_config.getEventEVval(index, 1) << endl;
  return;
}

//
/// user-defined frame processing function
/// called from the MLCB library for *every* CAN frame received
/// it receives a pointer to the received CAN frame
//

void framehandler(CANFrame *msg) {

  // as an example, format and display the received frame
  Serial << "[ " << (msg->id & 0x7f) << "] [" << msg->len << "] [";

  for (byte d = 0; d < msg->len; d++) {
    Serial << " 0x" << _HEX(msg->data[d]);
  }

  Serial << " ]" << endl;
  return;
}

//
/// print code version config details and copyright notice
//

void printConfig(void) {

  // code version
  Serial << F("> code version = ") << MAJOR_VERSION << "." << MINOR_VERSION << "." << BUILD_VERSION << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2023") << endl;
  return;
}

//
/// command interpreter for serial console input
//

void processSerialInput(void) {

  byte uev = 0;
  char msgstr[32], dstr[32];

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        printConfig();

        // node identity
        Serial << F("> MLCB node configuration") << endl;
        Serial << F("> mode = ") << (module_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID << F(", node number = ") << module_config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << module_config.EE_MAX_EVENTS << F(" EVs per event = ") << module_config.EE_NUM_EVS << F(" bytes per event = ") << module_config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (module_config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * module_config.EE_BYTES_PER_EVENT) << F(" of ") << (module_config.EE_MAX_EVENTS * module_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (module_config.EE_NUM_EVS); j++) {
          sprintf(dstr, "EV%03d | ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {

          if (module_config.getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d  | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (module_config.EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", module_config.readEEPROM(module_config.EE_EVENTS_START + (j * module_config.EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, "%4d |", module_config.getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= module_config.EE_NUM_NVS; j++) {
          byte v = module_config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':

        MLCB.printStatus();
        break;

      case 'h':
        // event hash table
        module_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and MLCB message processing
        MLCB.reset();
        break;

      case '*':
        // reboot
        module_config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << module_config.freeSRAM() << F(" bytes") << endl;
        break;

      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
