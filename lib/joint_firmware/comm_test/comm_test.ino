/**
 * @file comm_test.ino
 * @author sbstorz
 * @brief joint firmware for communication test
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains a copy of the joint firmware used for communication tests.
 *
 */


/**
 * @brief Define either joint that is to be flashed
 * 
 * Define either J1, J2, J3 or J4 and subsequently include configuration.h 
 */
#define J1
#include "../joint/configuration.h"

#include <UstepperS32.h>
#include <Wire.h>
#include "../joint/joint.h"
#include "../joint/filters.h"
#include "../joint/stall.h"

UstepperS32 stepper;
static Lowpass lp(1, 0.01, 0.1);

static uint8_t driveCurrent, holdCurrent;
static uint8_t notHomed = 1;
static uint8_t isStalled = 0;
static uint8_t isBusy = 0;
static uint8_t notEnabled = 1;
static uint8_t isStallguardEnabled = 0;
static int stallguardThreshold = 0;
static float q_set = 0.0, q = 0.0, qd_set = 0.0, qd = 0.0;
static float maxAccel = MAXACCEL;
static float maxVel = MAXVEL;
static float homingOffset = 0;

uint8_t reg = 0, blk_reg = 0;

uint8_t rx_buf[MAX_BUFFER] = { 0 };
uint8_t tx_buf[MAX_BUFFER + RFLAGS_SIZE] = { 0 };
bool rx_data_ready = 0;

size_t tx_length = 0;
size_t rx_length = 0;

static uint32_t deadman = 0;
static String reg_txt;



void blocking_handler(uint8_t reg);
void non_blocking_handler(uint8_t reg);
void set_flags_for_blocking_handler(uint8_t reg);

/**
 * @brief I2C receive event Handler.
 *
 * Reads the content of the received message. Saves the register so it can be used in the main loop. 
 * If the master invokes the read() function the message contains only the register byte and no payload.
 * If the master invokes the write() the message has a payload of appropriate size for the command.
 * Every I2C transaction starts with a receive event when the command is sent and is immediatly followed by a request 
 * since at minimum the flags need to be transmitted back. This means that the receive handler and request handler are always 
 * executed sequentially. The main loop is not executed since both handlers are ISRs.
 * For a read request the message looks like this: \n 
 * \< [REG] \n 
 * \> [TXBUFn]...[TXBUF2][TXBUF1][TXBUF0][FLAGS] \n 
 * For a command the message looks like this: \n 
 * \< [REG][RXBUFn]...[RXBUF2][RXBUF1][RXBUF0] \n 
 * \> [FLAGS] \n 
 * The payload is read into the rx_buf, rx_length is set to the payload length.
 * @param n the number of bytes read from the controller device: MAX_BUFFER
 */
void receiveEvent(int n) {
  Serial.print("Receive Event, reading...\n");
  reg = Wire.read();
  Serial.printf("Read Register: 0x%02x\n", reg);
  int i = 0;
  while (Wire.available()) {
    rx_buf[i] = Wire.read();
    i++;
  }
  rx_length = i;
  tx_length = 0;

  if (i) {
    Serial.print("rx_buf: ");
    DUMP_BUFFER(rx_buf, rx_length);
  }
}

/**
 * @brief I2C request event Handler.
 *
 * Sends the response data to the master. Every transaction begins with a receive event. The request event is always triggered since at a minimum the status flags are returned
 * to the master.
 * Hence this function is only invoked after the receiveEvent() handler has been called. The function calls the non_blocking_handler() which is non-blocking.
 * Since most Ustepper functions are non-blocking as they just read/write registers to the stepper driver/encoder they can be handled directly in the ISR.
 * The non_blocking_handler() populates the tx_buf with relevant data, the current state flags are appended to the tx_buf and then it is send to the master.
 */
void requestEvent() {
  Serial.print("Request Event...\n");
  // Serial.print("Register: ");
  // Serial.println(reg);

  non_blocking_handler(reg);
  uint8_t state = 0x00;
  state |= (isStalled << 0);
  state |= (isBusy << 1);
  state |= (notHomed << 2);
  state |= (notEnabled << 3);
  // Serial.print("state: \t");
  // Serial.print(state, HEX);
  // Serial.print("\n");
  tx_buf[tx_length++] = state;
  // Serial.print("tx_buf: \t");
  // DUMP_BUFFER(tx_buf, tx_length);
  Wire.write(tx_buf, tx_length);
  deadman = millis();
}

/**
 * @brief Handles commands received via I2C.
 * @warning This is a blocking function which may take some time to execute. This function must not be called from an ISR or callback! 
 * Call from main loop instead.

 * The registers handled in this handler are those whose implementation can take time and can thereby not be called directly from the request handler.
 * @param reg command that should be executed.
 */
void blocking_handler(uint8_t reg) {
  // Serial.print("Receive Handler: \t Register: ");
  // Serial.println(reg);
  switch (reg) {
    case CHECKORIENTATION:
      {
        Serial.print("Executing CHECKORIENTATION\n");
        reg_txt = "CHECKORIENTATION";
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.checkOrientation(v);
        break;
      }

    case HOME:
      {
        Serial.print("Executing HOME\n");
        reg_txt = "HOME";
        uint8_t dir;
        uint8_t speed;
        uint8_t sensitivity;
        uint8_t current;
        memcpy(&dir, rx_buf, 1);
        memcpy(&speed, rx_buf + 1, 1);
        memcpy(&sensitivity, rx_buf + 2, 1);
        memcpy(&current, rx_buf + 3, 1);

        Serial.print("[HOME] Stopping Stepper\n");
        stepper.stop(HARD);
        delay(500);
        Serial.print("[HOME] Stopped Stepper\n");



        Serial.print("[HOME] Set Speed and Current \n");
        stepper.setRPM(dir ? speed : -speed);
        stepper.setCurrent(current);

        while (isBusy) {
          float err = stepper.getPidError();
          // Serial.print(abs(err));
          // Serial.printf("\t %u\n", sensitivity);
          if (abs(err) > sensitivity) {
            break;
          }
          delay(1);
        }

        /* Homing has been cancled from ISR (f.x. STOP) */
        if (!isBusy) {
          Serial.print("[HOME] aborted \n");
          notHomed = 1;
        } else {
          stepper.encoder.setHome();
          stepper.driver.setHome();
          notHomed = 0;
          isStalled = 0;
          Serial.print("[HOME] stalled \n");
        }

        Serial.print("[HOME] stopping stepper \n");
        stepper.stop();  // Stop motor !
        Serial.print("[HOME] Stopped Stepper\n");

        Serial.printf("[HOME] reset driveCurrent %u \n", driveCurrent);
        stepper.setCurrent(driveCurrent);

        Serial.println("[HOME] Done Homing");
        break;
      }

    default:
      Serial.println("UNKOWN REGISTER");
      break;
  }
}

/**
 * @brief Handles read request received via I2C.

 * Can be invoked from the I2C ISR since reads from the stepper are non-blocking. 
 * Also Handling reads and the subsequent wire.write(), did not work from the main loop.
 * @param reg command to execute/register to read.
 */

void non_blocking_handler(uint8_t reg) {
  rx_data_ready = 0;
  switch (reg) {
    case PING:
      {
        Serial.print("Executing PING\n");
        reg_txt = "PING";
        writeValue<char>(ACK, tx_buf, tx_length);
        // static uint32_t delay_ms = 0;
        // Serial.printf("Delay: %ld\n",delay_ms);
        // delay(delay_ms++);
        break;
      }

    case SETUP:
      {
        Serial.print("Executing SETUP\n");
        reg_txt = "SETUP";
        memcpy(&driveCurrent, rx_buf, 1);
        memcpy(&holdCurrent, rx_buf + 1, 1);

        stepper.setCurrent(driveCurrent);
        stepper.setHoldCurrent(holdCurrent);
        stepper.moveSteps(0);

        isStallguardEnabled = 0;
        notEnabled = 0;
        isStalled = 0;
        qd_set = 0;  //reset here so that no matter how long it takes after the enable call for the next command to arrive, we dont trigger the watchdog
        break;
      }

    case GETDRIVERRPM:
      // Serial.print("Executing GETDRIVERRPM\n");
      reg_txt = "GETDRIVERRPM";
      break;


    case ANGLEMOVED:
      {
        // Serial.print("Executing ANGLEMOVED\n");
        reg_txt = "ANGLEMOVED";
        q = stepper.angleMoved();
        writeValue<float>(q, tx_buf, tx_length);
        break;
      }

    case GETENCODERRPM:
      {
        // Serial.print("Executing GETENCODERRPM\n");
        reg_txt = "GETENCODERRPM";
        qd = stepper.encoder.getRPM();
        writeValue<float>(qd, tx_buf, tx_length);
        break;
      }

      /* Below are write commands that are non-blocking and can be executed from the request ISR */

    case SETRPM:
      {
        // Serial.print("Executing SETRPM\n");
        reg_txt = "SETRPM";
        readValue<float>(qd_set, rx_buf, rx_length);
        if (!isStalled) {
          stepper.setRPM(qd_set);
          // Serial.println(qd_set,4);
        }
        break;
      }

    case MOVESTEPS:
      {
        Serial.print("Executing MOVESTEPS\n");
        reg_txt = "MOVESTEPS";
        int32_t v;
        readValue<int32_t>(v, rx_buf, rx_length);
        stepper.moveSteps(v);
        break;
      }

    case MOVETOANGLE:
      {
        Serial.print("Executing MOVETOANGLE\n");
        reg_txt = "MOVETOANGLE";
        readValue<float>(q_set, rx_buf, rx_length);
        if (!isStalled) {
          stepper.moveToAngle(q_set);
          // Serial.println(q_set,4);
        }
        break;
      }


    case SETCURRENT:
      {
        Serial.print("Executing SETCURRENT\n");
        reg_txt = "SETCURRENT";
        readValue<uint8_t>(driveCurrent, rx_buf, rx_length);
        stepper.setCurrent(driveCurrent);
        if (driveCurrent == 0) {
          notEnabled = 1;
        }
        break;
      }

    case SETHOLDCURRENT:
      {
        Serial.print("Executing SETHOLDCURRENT\n");
        reg_txt = "SETHOLDCURRENT";
        readValue<uint8_t>(holdCurrent, rx_buf, rx_length);
        stepper.setHoldCurrent(holdCurrent);
        if (holdCurrent == 0) {
          notEnabled = 1;
        }
        break;
      }

    case SETMAXACCELERATION:
      {
        Serial.print("Executing SETMAXACCELERATION\n");
        reg_txt = "SETMAXACCELERATION";
        readValue<float>(maxAccel, rx_buf, rx_length);
        maxAccel *= 200 / 360.0;  // conversion from degrees/s^2 to steps/s^2
        stepper.setMaxAcceleration(maxAccel);
        stepper.setMaxDeceleration(maxAccel);
        break;
      }

      // case SETMAXDECELERATION:
      // Serial.print("Executing SETMAXDECELERATION\n");
      reg_txt = "SETMAXDECELERATION";
      //   break;

    case SETMAXVELOCITY:
      {
        Serial.print("Executing SETMAXVELOCITY\n");
        reg_txt = "SETMAXVELOCITY";
        readValue<float>(maxVel, rx_buf, rx_length);
        maxVel *= 200 / 360.0;  // conversion from degrees/s to steps/s
        stepper.setMaxVelocity(maxVel);
        break;
      }

    case ENABLESTALLGUARD:
      {
        Serial.print("Executing ENABLESTALLGUARD\n");
        reg_txt = "ENABLESTALLGUARD";

        // Very simple workaround for stall detection, since the built-in encoder stall-detection is tricky to work with in particular in combination with homing since it can not be reset.
        uint8_t sensitivity;
        readValue<uint8_t>(sensitivity, rx_buf, rx_length);
        stallguardThreshold = sensitivity * 10;
        // Serial.println(sensitivity*1.0/10);
        // stepper.encoder.encoderStallDetectSensitivity = sensitivity * 1.0/10 ;
        // stepper.encoder.encoderStallDetectEnable = 1;
        // stepper.encoder.encoderStallDetect = 0;
        isStallguardEnabled = 1;
        isStalled = 0;

        break;
      }

      // case DISABLESTALLGUARD:
      // Serial.print("Executing DISABLESTALLGUARD\n");
      // reg_txt = "DISABLESTALLGUARD";
      //   break;

      // case CLEARSTALL:
      // Serial.print("Executing CLEARSTALL\n");
      // reg_txt = "CLEARSTALL";
      //   break;

    case SETBRAKEMODE:
      {
        Serial.print("Executing SETBRAKEMODE\n");
        reg_txt = "SETBRAKEMODE";
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setBrakeMode(v);
        break;
      }

      // case ENABLEPID:
      // Serial.print("Executing ENABLEPID\n");
      // reg_txt = "ENABLEPID";
      //   break;

      // case DISABLEPID:
      // Serial.print("Executing DISABLEPID\n");
      // reg_txt = "DISABLEPID";
      //   break;


    case DISABLECLOSEDLOOP:
      {
        Serial.print("Executing DISABLECLOSEDLOOP\n");
        reg_txt = "DISABLECLOSEDLOOP";
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.disableClosedLoop();
        notEnabled = 1;
        break;
      }

    case STOP:
      {
        Serial.print("Executing STOP\n");
        reg_txt = "STOP";
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setRPM(0);

        // Set new position
        // stepper.driver.setPosition(stepper.driver.getPosition());

        // reset isBusy flag to signal to blocking functions to stop when they can
        isBusy = 0;
        break;
      }

    case HOMEOFFSET:
      {
        Serial.print("Executing HOMEOFFSET\n");
        reg_txt = "HOMEOFFSET";
        if (rx_length) {
          readValue<float>(homingOffset, rx_buf, rx_length);
        } else {
          writeValue<float>(homingOffset, tx_buf, tx_length);
        }
        break;
      }

    case HOME:
      {
        /* Immediatly set the notHomed and isBusy flag.
        This is neccessary since if homing command is received but the blocking_handler has not handled the command yet
        a following read request might read isBusy and notHomed to be 0 leading to the false assumption homing has completed. */
        notHomed = 1;
        set_flags_for_blocking_handler(reg);

        break;
      }

    default:
      set_flags_for_blocking_handler(reg);
      break;
  }
}

/**
 * @brief prepare flags to initiate the blocking handling of the received comman.
 *
 * Sets the blk_reg to reg, this makes sure that even if a new command is received the blocking hanlder access the latest blocking command.
 * Sets the isBusy flag, to indicate immediatly that the blocking has not been finished. This is if a status request follows immidiatly the command,
 * before the context back to the main loop has happened.
 * Sets the rx_data_ready flag to indicate to the main loop that there is data to read in the blocking handler.
 * Sets the tx_length to 0 because blocking commands can not return a payload.
 */
void set_flags_for_blocking_handler(uint8_t reg) {
  blk_reg = reg;
  isBusy = 1;
  rx_data_ready = 1;
  tx_length = 0;
}


/**
 * @brief Setup Peripherals
 *
 * Setup I2C with the address ADR, and begin Serial for debugging with baudrate 9600.
 * Setup the stepper, perform orientation check to check wiring and disable again.
 */
void setup(void) {
  // Join I2C bus as follower
  Wire.begin(ADR);
  Serial.begin(9600);

  stepper.setup(CLOSEDLOOP, 200);
  stepper.enableClosedLoop();  // necessary to be able to use PID error
  stepper.setMaxAcceleration(maxAccel);
  stepper.setMaxDeceleration(maxAccel);
  stepper.setMaxVelocity(maxVel);
  stepper.setControlThreshold(15);
  stepper.checkOrientation(1);
  stepper.stop();
  stepper.setCurrent(0);      // Not technically necessary, freewheeling also without
  stepper.setHoldCurrent(0);  // Not technically necessary, freewheeling also without
  stepper.setBrakeMode(0);
  notHomed = 1;

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}


uint32_t last = millis();

/**
 * @brief Main loop

 * Executes the following: \n 
 * 1. if isStallguardEnabled: compares stepper.getPidError() with stallguardThreshold and sets isStalled flag. \n 
 * 2. if rx_data_ready: set isBusy flag to indicate device is busy. Invoke blocking_handler. 
 * Clear isBusy flag to indicate device is no longer busy \n 
 */
void loop(void) {

  if (rx_data_ready) {
    rx_data_ready = 0;
    isBusy = 1;  // set is busy flag
    blocking_handler(blk_reg);
    isBusy = 0;  // reset is busy flag
  }

  uint32_t now = millis();
  uint32_t dt = now -last;
  if(dt > 1){
     Serial.printf("time: %ld\n", dt);
  }
 
  last = now;
  delay(1);
}
