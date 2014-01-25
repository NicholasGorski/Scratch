/*
 * Nintendo 64 verifier adapter for Arduino (16MHz ATmega328).
 * Copyright 2014 Nicholas Gorski.
 *
 * For notes and instructions on use, see NOTES.md.
 *
 * If you use this on anything other than an Arduino Uno r3,
 * you're on your own for modifying the code and circuit. :)
 */

#define MSG_DEBUG 127
#define DEBUG_PRINT(...) ((void)(Serial.write(MSG_DEBUG),Serial.print(__VA_ARGS__), Serial.write(0)))
#define DEBUG_PRINTLN(...) ((void)(Serial.write(MSG_DEBUG),Serial.println(__VA_ARGS__), Serial.write(0)))

//#define VERBOSE
#ifdef VERBOSE
  #define VERBOSE_PRINT(...) ((void)(Serial.write(MSG_DEBUG),Serial.print(__VA_ARGS__), Serial.write(0)))
  #define VERBOSE_PRINTLN(...) ((void)(Serial.write(MSG_DEBUG),Serial.println(__VA_ARGS__), Serial.write(0)))
#else
  #define VERBOSE_PRINT(...) ((void)0)
  #define VERBOSE_PRINTLN(...) ((void)0)
#endif

// Bits per second to transfer over Serial communication.
#define SERIAL_BITS_PER_SECOND 1000000
#define SERIAL_TIMEOUT 15000

#define SERIAL_WAIT_FOR_BYTES(bytesAvailable)         \
        while (Serial.available() < bytesAvailable) { \
          continue;                                   \
        }

#define SERIAL_WAIT_FOR_BYTES_TIMEOUT(bytesAvailable, timeout, timeoutVar)  \
        for (unsigned long endTime = micros() + timeout; ;) {               \
          timeoutVar = false;                                               \
          while (!timeoutVar && Serial.available() < bytesAvailable) {      \
            timeoutVar = ((long) (micros() - endTime)) >= 0;                \
          }                                                                 \
                                                                            \
          break;                                                            \
        }
          

// Buffer sizes for communication.
#define RX_BUFFER_SIZE 36

#define TX_ID_POS  0
#define TX_ID_SIZE 3
#define TX_STATUS_POS  (TX_ID_POS + TX_ID_SIZE)
#define TX_STATUS_SIZE 4
 
#define TX_BUFFER_SIZE (TX_STATUS_POS + TX_STATUS_SIZE)

// These constants define the DDR port, data pin, pin number,
// and interrupt number/flag the console's data line is using.
#define N64_CONSOLE_DATA_DDR      DDRD
#define N64_CONSOLE_DATA_PIN      PIND
#define N64_CONSOLE_DATA_PIN_NO   PIND3
#define N64_CONSOLE_DATA_INT_NUM  1
#define N64_CONSOLE_DATA_INT_FLAG INTF1
 
// These constants define the DDR port, data pin, pin number,
// and interrupt number/flag the controller's data line is using.
#define N64_CONTROLLER_DATA_DDR      DDRD
#define N64_CONTROLLER_DATA_PIN      PIND
#define N64_CONTROLLER_DATA_PIN_NO   PIND2
#define N64_CONTROLLER_DATA_INT_NUM  0
#define N64_CONTROLLER_DATA_INT_FLAG INTF0

// The pin that program control waits on for recording and playback.
#define WAIT_PIN    PIND
#define WAIT_PIN_NO PIND7

// Pin that the acive LED is connected to.
#define ACTIVE_PIN 6

// Pin that the panic LED is connected to.
#define PANIC_PIN A5

// These are the possible commands to send to the controller.
// TODO(ngorski): The rest of them.
#define N64_COMMAND_IDENTITY 0x00
#define N64_COMMAND_RESET 0xff
#define N64_COMMAND_STATUS 0x01

enum n64ControllerIdentity {
  IDENTITY_NO_EXPANSION_PACK = 0x0,
  IDENTITY_EXPANSION_PACK = 0x01,
  IDENTITY_EXPANSION_PACK_REMOVED = 0x02,
  IDENTITY_EXPANSION_PACK_INSERTED = 0x03,
  IDENTITY_CONTROLLER_ERROR = 0x04,
};

// This is the 32-bit response the controller
// gives when commanded for its status.
struct n64ControllerStatus {
  uint8_t right : 1;
  uint8_t left : 1;
  uint8_t down : 1;
  uint8_t up : 1;
  uint8_t start : 1;
  uint8_t z : 1;
  uint8_t b : 1;
  uint8_t a : 1;
  
  uint8_t cRight : 1;
  uint8_t cLeft : 1;
  uint8_t cDown : 1;
  uint8_t cUp : 1;
  uint8_t r : 1;
  uint8_t l : 1;
  uint8_t unused : 1;
  uint8_t reset : 1;
  
  int8_t x;
  int8_t y;
};

// To prevent the Arduino preprocessor from trying to generate these prototypes itself, we
// provide them manually. It must match exactly (whitespace included) for the preprocessor
// to see it as a duplicate. (Generating them has to be prevented because the prototypes will
// try to use custom data types before they are defined, an error in the preprocessor logic.)
boolean handleN64Command(uint8_t rxdata[RX_BUFFER_SIZE], const uint8_t txdata[TX_BUFFER_SIZE], boolean wait);
boolean commandN64Controller(const uint8_t txdata[], const uint8_t txlen, uint8_t rxdata[], const uint8_t rxlen);
boolean convertN64RawControllerIdentity(const uint8_t rawResult[3], n64ControllerIdentity& result);
boolean getN64ControllerIdentity(n64ControllerIdentity& result);
boolean getN64ControllerResetIdentity(n64ControllerIdentity& result);
boolean getN64ControllerStatus(n64ControllerStatus& result);

// General implementation note:
// Some bits of assembly code are repeated, exactly or nearly verbatim, across different functions.
// While removing this duplicate is technically possible, the result makes the program difficult to
// read and reason about. The assembly blocks err on the side of being straight-forward and easy to
// read, so the repetition is seen as justified. The maintanence of the blocks is hoped to be minimal.
//
// Much of the work here is for timing purposes.
//
// The number of cycles taken per-operation is noted
// in the comments. X/Y means X cycles if the branch
// wasn't taken, and Y if it was. For the sbic and
// sbis instructions, X/Y means X if no skip is done
// and Y if a skip was done, *where Y has already
// taken into account the cycles needed to skip the
// next instruction* (i.e., either 2 or 3).
//
// For a 16MHz processor, these timings are relevant:
//     1 cycle takes 62.5ns.
//     16 cycles takes 1us.
//     32 cycles takes 2us.
//     48 cycles takes 3us.
//     64 cycles takes 4us.
//
// See NOTES.md for how N64 communication works.

/**
 * Receive a command from the N64 console.
 *
 * The size of the commands vary, but the largest known is 35 bytes (1 byte + 34 data bytes).
 * Ensure the receive buffer is large enough to accomodate the entire command, or data loss
 * will occur.
 *
 * This receives bytes MSB to LSB. For example, the buffer:
 *     10010101 10101101
 * is received in this order:
 *     ABCDEFGH IJKLMNOP
 * Make sure the receive buffer is prepared for this order.
 *
 * @param rxdata The buffer for receiving data. Must be RX_BUFFER_SIZE bytes.
 * @param txdata The buffer for transmitting data. Must be TX_BUFFER_SIZE bytes.
 * @param wait If true, wait for the wait pin to go HIGH before continuing.
 * @return True if the response bytes were read without timeout, false otherwise.
 */
boolean handleN64Command(uint8_t rxdata[RX_BUFFER_SIZE], const uint8_t txdata[TX_BUFFER_SIZE], boolean wait) {
  uint8_t num = 0;
  uint8_t debug0 = 0;
  uint8_t debug1 = 0;
  uint8_t debug2 = 0;
  uint8_t debug3 = 0;

  // Disable interrupts, and store old interrupt status for later.
  const uint8_t oldSREG = SREG; 
  noInterrupts();
  
  asm volatile("\n"
      "/* N64 Console Rx/Tx assembly block. */"                                                          "\n"      
      // We'll want to refer to the first byte of the request buffer (the command)
      // directly later on, so alias x into y because x is going to change.
      "  movw r28, r26             " /* Copy the X address to Y.                          1 cycle.    */ "\n"
      
      // We'll do some transmission preparation during receiving, and this register
      // will hold a result of the preparation, clear it so we have a test on it.
      "  clr r19                   "  /* Clear the transmission length counter.           1 cycle.    */ "\n"
      
      // Now receiving. Prepare the byte buffer and bit mask and jump to start.
      "  clr r18                   " /* Clear the byte buffer.                            1 cycle.    */ "\n"      
      "  ldi r17, 0x80             " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n"
      "  cpi %[wait], 0x00         " /* Compare wait with zero.                           1 cycle.    */ "\n"
      "  breq waitFirstFall%=      " /* Wait is false, so go directly to receiving.       1/2 cycles. */ "\n"
      "  rjmp waitForButton%=      " /* Wait is true, so wait for the button.             2 cycles.   */ "\n"
      
      // We need to put this jump target near the top for the waitFall loop, because for
      // speed purposes the target must be in a brne instruction which can only jump
      // somewhere within a 7-bit relative address. It's been 11 cycles since the stop bit
      // was finished, and this jump will make it 13.
      "timeoutDone%=:              "                                                                     "\n"
      "  rjmp transmit%=           " /* Got the stop bit, not begin transmitting.         2 cycles.   */ "\n"
      
      // If this is the first communication point, wait for the wait button to be pressed,
      // then wait for the line to go HIGH (console is ready for communication).
      "waitForButton%=:            "                                                                     "\n"
      "  sbis %[Wpin], %[WpinNo]   " /* Check pin; if HIGH, skip the next instruction.    1/2 cycles. */ "\n"
      "  rjmp waitForButton%=      " /* Pin was still LOW, continue waiting.              2 cycles.   */ "\n"
      "waitFirstRise%=:            "                                                                     "\n"
      "  sbis %[pin], %[pinNo]     " /* Check pin; if HIGH, skip the next instruction.    1/2 cycles. */ "\n"
      "  rjmp waitFirstRise%=      " /* Pin was still LOW, continue waiting.              2 cycles.   */ "\n"
      
      // Other times we wait for a fall cycle we have a timeout that is setup
      // to catch the stop bit as quickly as possible. But the initial wait
      // needs to be arbitrarily large, as we're waiting on the console.
      "waitFirstFall%=:            "                                                                     "\n"
      "  sbic %[pin], %[pinNo]     " /* Check pin; if LOW, skip the next instruction.     1/2 cycles. */ "\n"
      "  rjmp waitFirstFall%=      " /* Pin was still HIGH, continue waiting.             2 cycles.   */ "\n"
      
      // The start sub-bit was received; wait as close to 2us after that, then sample.
      // Since 2 cycles pass between checks, on average the line will have been LOW
      // for 1 cycle before we detected it. With the 2 cycles it took to skip the
      // loop jump, we are about 3 cycles into the signal. After jumping to the
      // sample loop (2 cycles), we still need to wait 27 more cycles before we
      // sample to hit the 2us mark on average.
      "  ldi r16, 9                " /* Counter for wait loop (26 cycles of loop total).  1 cycle.    */ "\n"
      "  rjmp samplelp%=           " /* Jump into the sample loop.                        2 cycles.   */ "\n"
      
      // Setup for a wait fall cycle. The timeout lets us know the last bit was the stop
      // bit. We need to give the console time to follow the 1us HIGH protocol before
      // expecting the fall. Since we came from waitRise%=, we now in the stop sub-bit
      // best case (pin was HIGH the entire time after the start sub-bit of the previous
      // bit was detected, so was detected right away) that it took 4 cycles to get here,
      // so we have 12 more cycles to wait before assuming timeout. From testing, an
      // additional 4 cycles are needed to account for clock drift, giving 16 cycles to wait
      // until timeout (20 total since the line was read HIGH). Setting up the timeout
      // value leaves 15 cycles. Each loop takes 5 cycles, with 2 cycles of timeout handling
      // before actually sampling. By adding 4 wait cycles before beginning the loop
      // (leaving 12 more to go), we can align the timeout cycle with the instruction just
      // prior to the sample. Should we timeout (timeoutDone%= is taken), we'll have waited
      // (on average, account for clock drift) a total of 11 cycles past the full stop bit.
      "setupWaitFall%=:            "                                                                     "\n"
      "  ldi r16, 4                " /* Timeout before assuming we read the stop bit.     1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      // Wait for the line to fall to the start sub-bit of the response.
      "waitFall%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the timeout counter.                    1 cycle.    */ "\n"
      "  breq timeoutDone%=        " /* If zero, jump to timeout handler.                 1/2 cycles. */ "\n"
      "  sbic %[pin], %[pinNo]     " /* Check pin; if LOW, skip the next instruction.     1/2 cycles. */ "\n"
      "  rjmp waitFall%=           " /* Pin was still HIGH, continue waiting.             2 cycles.   */ "\n"
       
      // The start sub-bit was received; wait as close to 2us after that, then sample.
      // From testing, the waitFall%= loop will execute multiple times before we see
      // the line go LOW (that is, the first sbic is definitely reached and the test
      // does not result in a skip). Since 4 cycles pass between checks, on average
      // the line will have been LOW for 2 cycles before we detected it. With the 2
      // cycles it took to skip the loop jump, we are about 4 cycles into the signal.
      // We need to wait 28 more cycles before we sample to hit the 2us mark on average.
      //
      // The way we'll use this time is to prepare the transmission data if the
      // command is ready, so when it comes time to transmit this portion has already
      // been completed, giving us more time to do other things. First we check if
      // this processing has been completed (r19 will be non-zero), and if not then
      // we check of the processing can be completed (num is non-zero). If these
      // pass, we can read the command from the Y register we copied at the start,
      // and adjust the Z register (txdata) to the correct point and store the length
      // in r19 according to the command. Every path from this computation tree will
      // end at 28 cycles.
      "  cpi r19, 0x00             " /* Check if the process has already been done.       1 cycle.    */ "\n"
      "  brne txIsReady%=          " /* If so, skip over the preparation.                 1/2 cycles. */ "\n"
      "  cpi %[num], 0x00          " /* Check if the process can even be done.            1 cycle.    */ "\n"
      "  breq txCantReady%=        " /* If not, skip over the preparation.                1/2 cycles. */ "\n"
      "  ld r16, y                 " /* Load the command into r16.                        2 cycles.   */ "\n"
      // It's taken 6 cycles to get here, 22 to go.
      "  cpi r16, 0xff             " /* Check if it's the reset command.                  1 cycle.    */ "\n"
      "  breq cmdReset%=           " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x00             " /* Check if it's the identity command.               1 cycle.    */ "\n"
      "  breq cmdIdentity%=        " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x01             " /* Check if it's the status command.                 1 cycle.    */ "\n"
      "  breq cmdStatus%=          " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x02             " /* Check if it's the read pak command.               1 cycle.    */ "\n"
      "  breq cmdReadPak%=         " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x03             " /* Check if it's the write pak command.              1 cycle.    */ "\n"
      "  breq cmdWritePak%=        " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x04             " /* Check if it's the read ROM command.               1 cycle.    */ "\n"
      "  breq cmdReadROM%=         " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  cpi r16, 0x05             " /* Check if it's the write ROM command.              1 cycle.    */ "\n"
      "  breq cmdWriteROM%=        " /* If so, jump to the command preparation code.      1/2 cycles. */ "\n"
      "  rjmp cmdUnknown%=         " /* If none of the above, this is an unknown command. 2 cycles.   */ "\n"
      // Reset and identity are identical to us, sync timing with the next case.
      "cmdReset%=:                 "                                                                     "\n"
      "  nop                       " /* Waiting, cycle 1 of 2.                            1 cycle.    */ "\n"
      "  nop                       " /* Waiting, cycle 2 of 2.                            1 cycle.    */ "\n"
      // Point to the identity portion of the buffer, then wait the remaining cycles.
      // 17 more to go from here.
      "cmdIdentity%=:              "                                                                     "\n"
      "  adiw r30, %[idPos]        " /* Offset Z by the position of the data.             2 cycles.   */ "\n"
      "  ldi r19, %[idSize]        " /* Store the length of the data in r19.              1 cycle.    */ "\n"
      "  ldi r16, 4                " /* Counter for wait loop (11 cycles of loop total).  1 cycle.    */ "\n"
      "cmdIdentitylp%=:            "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne cmdIdentitylp%=      " /* If not zero, jump to cmdIdentitylp%=.             1/2 cycles. */ "\n"
      "  rjmp sample%=             " /* Now sample.                                       2 cycles.   */ "\n"
      // Point to the status portion of the buffer, then wait the remaining cycles.
      // 15 more to go from here.
      "cmdStatus%=:                "                                                                     "\n"
      "  adiw r30, %[statusPos]    " /* Offset Z by the position of the data.             2 cycles.   */ "\n"
      "  ldi r19, %[statusSize]    " /* Store the length of the data in r19.              1 cycle.    */ "\n"
      "  ldi r16, 3                " /* Counter for wait loop (8 cycles of loop total).   1 cycle.    */ "\n"
      "cmdStatuslp%=:              "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne cmdStatuslp%=        " /* If not zero, jump to cmdStatuslp%=.               1/2 cycles. */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  rjmp sample%=             " /* Now sample.                                       2 cycles.   */ "\n"
      // None of the following commands are supported, just error out:
      "cmdReadPak%=:               "                                                                     "\n"
      "  rjmp commandBad%=         " /* Cannot handle this command.                       2 cycles.   */ "\n"
      "cmdWritePak%=:              "                                                                     "\n"
      "  rjmp commandBad%=         " /* Cannot handle this command.                       2 cycles.   */ "\n"
      "cmdReadROM%=:               "                                                                     "\n"
      "  rjmp commandBad%=         " /* Cannot handle this command.                       2 cycles.   */ "\n"
      "cmdWriteROM%=:              "                                                                     "\n"
      "  rjmp commandBad%=         " /* Cannot handle this command.                       2 cycles.   */ "\n"
      "cmdUnknown%=:               "                                                                     "\n"
      "  rjmp commandBad%=         " /* Cannot handle this command.                       2 cycles.   */ "\n"
      // Preparation has already been done, sync timing with the next case.
      "txIsReady%=:                "                                                                     "\n"
      "  nop                       " /* Waiting, cycle 1 of 2.                            1 cycle.    */ "\n"
      "  nop                       " /* Waiting, cycle 2 of 2.                            1 cycle.    */ "\n"
      // Prepartion cannot yet be done, just wait to sample. 23 to go from here.
      "txCantReady%=:              "                                                                     "\n"
      "  nop                       " /* Waiting, cycle 1 of 2.                            1 cycle.    */ "\n" 
      "  nop                       " /* Waiting, cycle 2 of 2.                            1 cycle.    */ "\n"            
      "  ldi r16, 7                " /* Counter for wait loop (20 cycles of loop total).  1 cycle.    */ "\n"
      
      // Sample waiting loop.
      "samplelp%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne samplelp%=           " /* If not zero, jump to samplelp%=.                  1/2 cycles. */ "\n"

      // Now perform the sample. The work between here and waitHigh needs to be
      // quick enough (< 1us) that we don't miss the data line going HIGH. However, we
      // need to wait 1us before sampling for the stop sub-bit, otherwise if we just
      // received a 1 and sample the stop-bit as soon as possible, we'll have skipped
      // an entire 1us of signal. This will throw off the stop bit timeout at waitFall%=.
      // To counter-act this, we assume we got here as fast as possible (31 cycles since
      // detecting the start sub-bit), and that we need to wait 48 - 31 = 17 cycles more
      // before sampling. The sample loop has 3 cycles preamble, so we need to ensure all
      // paths from the following computation tree end at 14 cycles. Now should we sample
      // the stop sub-bit immediately because a 1 was sent, it's just as if we sampled it
      // from a perfectly-timed 0 instead.      
      "sample%=:                   "                                                                     "\n"
      "  sbic %[pin], %[pinNo]     " /* Check pin; if LOW, skip the next instruction.     1/2 cycles. */ "\n"
      "  or r18, r17               " /* The pin was HIGH, enable this bit in our byte.    1 cycle.    */ "\n"
      "  lsr r17                   " /* Move on to the next bit in our byte.              1 cycle.    */ "\n"
      "  brne sampleMoreBits%=     " /* If more bits to read, jump to sampleMoreBits%=.   1/2 cycles. */ "\n"
      
      // Finished reading that byte, store it in rxdata.
      "  cpi %[num], %[rxlen]      " /* Compare rxlen with num.                           1 cycle.    */ "\n"
      "  breq oomRx%=              " /* If equal, ran out of receive memory.              1/2 cycles. */ "\n"
      "  st x+, r18                " /* Store value in r18 to address x, incrementing x.  2 cycles.   */ "\n"
      "  inc %[num]                " /* Increment the number of received bytes.           1 cycle.    */ "\n"
      "  clr r18                   " /* Clear the byte buffer.                            1 cycle.    */ "\n"
      "  ldi r17, 0x80             " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n" 
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  rjmp setupWaitRise%=      " /* Enough cycles have passed, jump to the sample.    2 cycles.   */ "\n" 
      
      // Had more bits to go and skipped the byte storage section.
      "sampleMoreBits%=:           "                                                                     "\n"
      "  ldi r16, 3                " /* Counter for wait loop (8 cycles of loop total).  1 cycle.    */ "\n"
      
      // The wait loop shared by both above paths.
      "samplePostDelay%=:          "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne samplePostDelay%=    " /* If not zero, jump to samplePostDelay%=.           1/2 cycles. */ "\n"     
      
      // To move on to the next bit, first we wait for the stop sub-bit. The entire
      // process should fit within 4us, so we need to wait at least that long since
      // we first saw the start sub-bit. This timeout would be an unexpected error,
      // though, so as long as we don't wait too long and mistakingly read a rise
      // from a subsequent request as one for this request, we're fine. Since the
      // console will wait ~200us before bugging us again, a 0xff timeout is fine.
      "setupWaitRise%=:            "                                                                     "\n"
      "  ldi r16, 0xff             " /* Timeout before assuming no more data is coming.   1 cycle.    */ "\n"
      // Wait for the line to rise to the stop sub-bit of the response.
      "waitRise%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the timeout counter.                    1 cycle.    */ "\n"
      "  breq timeoutBad%=         " /* If zero, jump to timeout handler.                 1/2 cycles. */ "\n"
      "  sbis %[pin], %[pinNo]     " /* Check pin; if HIGH, skip the next instruction.    1/2 cycles. */ "\n"
      "  rjmp waitRise%=           " /* Pin was still LOW, continue waiting.              2 cycles.   */ "\n"
      "  rjmp setupWaitFall%=      " /* Pin has gone HIGH, move on to next bit.           2 cycles.   */ "\n"

      // An error occurred. Indicate this by setting num to max. We also may have 3 slots
      // to store debug information, though the reuse of registers makes this difficult.

      // OOM receiving: 0xff (255), loaded command length, and transmission buffer position.
      "oomRx%=:                    "                                                                     "\n"
      "  mov %[debug0], r30        " /* Show transmission buffer position (lower byte).   1 cycle.    */ "\n"
      "  mov %[debug1], r19        " /* Show the loaded command length.                   1 cycle.    */ "\n"
      "  ldi %[num], 0xff          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
      
      // Bad command: 0xfe (254), loaded command.
      "commandBad%=:               "                                                                     "\n"
      "  mov %[debug0], r16        " /* Show the loaded command.                          1 cycle.    */ "\n"
      "  ldi %[num], 0xfe          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
      
      // Timeout on stop sub-bit: 0xfd (253), current byte, current bit, number of bytes.
      "timeoutBad%=:               "                                                                     "\n" 
      "  mov %[debug0], r18       " /* Show the current byte.                             1 cycle.    */ "\n"
      "  mov %[debug1], r17       " /* Show the current bit.                              1 cycle.    */ "\n"
      "  mov %[debug2], %[num]    " /* Show number of received bytes.                     1 cycle.    */ "\n"
      "  ldi %[num], 0xfd          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
      
      // Bad transmission setup: 0xfc (252), current byte, current bit, number of bytes.
      "transmissionBad%=:          "                                                                     "\n"
      "  mov %[debug0], r18        " /* Show the current byte.                            1 cycle.    */ "\n"
      "  mov %[debug1], r17        " /* Show the current bit.                             1 cycle.    */ "\n"
      "  mov %[debug2], %[num]     " /* Show number of received bytes.                    1 cycle.    */ "\n"
      "  ldi %[num], 0xfc          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
            
      // Now we are transmitting. The stop bit has been completed for 13 cycles already.
      // The console will accept a delay of up to 61.375us (982 cycles) after the stop
      // bit. However, controllers regularily respond within 1.75us (28 cycles) after
      // the stop bit. What effect waiting longer has is unknown, so we opt to limit
      // ourselves to the controller-like response times. With 28 cycles max, then,
      // we have 15 remaining cycles to respond. Because we did the preparation while
      // waiting to sample the 9th bit, the data is already setup at txdata (Z), with
      // txlen (r19) bytes to transmit. The transmission preamble takes 10 cycles before
      // sending the first bit, so we have 5 cycles to spare.
      "transmit%=:                 "                                                                     "\n"
      "  cpi r19, 0x00             " /* Check that we are prepared for transmission.      1 cycle.    */ "\n"
      "  breq transmissionBad%=    " /* If not, we shouldn't be here.                     1/2 cycles. */ "\n"

      // TODO(ngorski): Expected command.

      // Prepares the next byte to send.
      // Takes 6 cycles if not done sending, 5 if done sending.
      "nextByte%=:            "                                                                     "\n"
      "  cpi r19, 0x00        " /* Compare txlen with zero.                          1 cycle.    */ "\n"
      "  breq doneSending%=   " /* If equal, jump to doneSending.                    1/2 cycles. */ "\n"
      "  dec r19              " /* Decrement txlen by one.                           1 cycle.    */ "\n"
      "  ld r16, z+           " /* Load value at address z into r16, incrementing z. 2 cycles.   */ "\n"
      "  ldi r17, 0x80        " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n"

      // Checks the next bit to send by testing if it's on or off.
      // Both branches (send 1 or send 0) take 4 cycles before the start sub-bit.
      "nextBit%=:             "                                                                     "\n"
      "  mov r18, r16         " /* Copy r16 into r18.                                1 cycle.    */ "\n"
      "  and r18, r17         " /* r18 equals bitwise-and on r18 nd r17.             1 cycle.    */ "\n"
      "  breq send0%=         " /* If zero, bit was off; send 0. Otherwise send 1.   1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"

      // The bit to send is 1. Pull LOW for 1us for the start sub-bit, then go HIGH
      // for 3us (keeping in mind the jump target's time at the end of the block).
      "send1%=:               "                                                                     "\n"
      // Going LOW for 1us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 1us LOW. With the 3 cycles since the sbi instruction, at
      // the termination the loop the total LOW wait time so far will be 14
      // cycles. Then two more cycles are used with a nop, giving 16 cycles total.
      "  ldi r18, 4           " /* Counter for wait loop (11 cycles of loop total).  1 cycle.    */ "\n"
      "send1lp0%=:            "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send1lp0%=      " /* If not zero, jump to send1lp0%=.                  1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (15 of 16 cycles).                1 cycle.    */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"
      // Going HIGH for 3us.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      // Wait loop for 3us HIGH. With the 3 cycles since the cbi instruction, at
      // the termination of the loop the total HIGH wait time so far will be 35
      // cycles. Then one more cycle is used shifting the bit mask. 12 cycles need
      // to be used before the next start sub-bit is sent. If the end jump is to
      // nextByte%=, 2 cycles are used making the jump and 10 cycles are used in
      // nextByte%= and nextBit%=, giving 12 cycles total. Otherwise, the jump is to
      // nextBit%=; to make the breq a consistent 2 cycles a nop is inserted after it,
      // then to make up for not going through nextByte%= 4 nops are used. Then 2 are
      // used for the rjmp, giving 12 cycles total.
      "  ldi r18, 11          " /* Counter for wait loop (32 cycles of loop total).  1 cycle.    */ "\n"
      "send1lp1%=:                 "                                                                "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send1lp1%=      " /* If not zero, jump to send1lp1%=.                  1/2 cycles. */ "\n"
      "  lsr r17              " /* Shift the bit in the mask to the right.           1 cycle.    */ "\n"
      "  breq nextByte%=      " /* If no more bits to check, jump to nextByte%=.     1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (1 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (2 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (3 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (4 of 4). 1 cycle.    */ "\n"
      "  rjmp nextBit%=       " /* Send next bit, and last 2 cycles of nextByte%=.   2 cycles.   */ "\n"

      // The bit to send is 0. Pull LOW for 3us, then go HIGH for 1us for the stop
      // sub-bit (keeping in mind the jump target's time at the end of the block).
      "send0%=:               "                                                                     "\n"
      // Going LOW for 3us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 3us LOW. With the 3 cycles since the sbi instruction, at
      // the termination of the loop the total LOW wait time so far will be 47
      // cycles. Then one more cycle is used with a nop, giving 48 cycles total.
      "  ldi r18, 15          " /* Counter for wait loop (44 cycles of loop total).  1 cycle.    */ "\n"
      "send0lp0%=:            "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send0lp0%=      " /* If not zero, jump to send0lp0%=.                  1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 3us (48 of 48 cycles).                1 cycle.    */ "\n"
      // Going HIGH for 1us.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      // There is no wait loop for 1us HIGH because of the jump target's cycles.
      // The cbi and nop instructions take 3 cycles, then one more cycle is used
      // shifting the bit mask. As was the case for send1lp1%=, 12 cycles need
      // to be used before the next start sub-bit is sent. If the end jump is to
      // nextByte%=, 2 cycles are used making the jump and 10 cycles are used in
      // nextByte%= and nextBit%=, giving 12 cycles total. Otherwise, the jump is to
      // nextBit%=; to make the breq a consistent 2 cycles a nop is inserted after it,
      // then to make up for not going through nextByte%= 4 nops are used. Then 2 are
      // used for the rjmp, giving 12 cycles total.
      "  nop                  " /* Waiting for 1us (3 of 16 cycles).                 1 cycle.    */ "\n"
      "  lsr r17              " /* Shift the bit in the mask to the right.           1 cycle.    */ "\n"
      "  breq nextByte%=      " /* If no more bits to check, jump to nextByte%=.     1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (1 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (2 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (3 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (4 of 4). 1 cycle.    */ "\n"
      "  rjmp nextBit%=       " /* Send next bit, and last 2 cycles of nextByte%=.   2 cycles.   */ "\n"

      // Finished sending. If we just sent out a stop sub-bit, it needs to be there
      // for 1us. Compared to sending more data, we left in the middle of running
      // through nextByte%= and nextBit%= which would have taken 10 cycles before
      // finishing the stop sub-bit. 3 cycles were used before arriving here, so we
      // wait for 7 cycles to complete the 1us stop sub-bit waiting time.
      "doneSending%=:         "                                                                     "\n"
      // Wait loop for 1us HIGH. With 1 cycle above, at the termination of the loop
      // the total HIGH wait time so far will be 6 cycles. One more cycle is used
      // afterward with a nop, giving 7 cycles total.
      "  ldi r18, 2           " /* Counter for wait loop (5 cycles of loop total).   1 cycle.    */ "\n"
      "doneSendinglp%=:       "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne doneSendinglp%= " /* If not zero, jump to doneSendinglp%=.             1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"

      // Now send the data stop bit. Like a regular 1 bit, the line will be brought
      // LOW for 1us then HIGH for 3us. Same timing as send1%= without the end jump.
      // Going LOW for 1us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 1us LOW. With the 3 cycles since the sbi instruction, at
      // the termination the loop the total LOW wait time so far will be 14
      // cycles. Then two more cycles are used with a nop, giving 16 cycles total.
      "  ldi r18, 4           " /* Counter for wait loop (11 cycles of loop total).  1 cycle.    */ "\n"
      "stoplp%=:              "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne stoplp%=        " /* If not zero, jump to stoplp%=.                    1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (15 of 16 cycles).                1 cycle.    */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"
      // Going HIGH for the last time. We don't need to ensure this hits 3us before
      // moving on because until the next request it'll be left HIGH, as is default.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      
      // Finished. 
      "end%=:                 "                                                                     "\n"
      : [num] "=a"(num),  // %[num] refers to num.
        [debug0] "=r"(debug0),  // %[debug0] refers to debug0.
        [debug1] "=r"(debug1),  // %[debug1] refers to debug1.
        [debug2] "=r"(debug2),  // %[debug2] refers to debug2.
        [debug3] "=r"(debug3)   // %[debug3] refers to debug3.
      : [ddr] "I"(_SFR_IO_ADDR(N64_CONSOLE_DATA_DDR)),  // %[ddr] refers to the DDR port IO register.
        [pin] "I"(_SFR_IO_ADDR(N64_CONSOLE_DATA_PIN)),  // %[pin] refers to the data pin IO register.
        [pinNo] "I"(N64_CONSOLE_DATA_PIN_NO),  // %[pinNo] refers to the pin bit number within %[pin].
        [Wpin] "I"(_SFR_IO_ADDR(WAIT_PIN)),  // %[Wpin] refers to the wait pin IO register.
        [WpinNo] "I"(WAIT_PIN_NO),  // %[WpinNo] refers to the pin bit number within %[Wpin].
        [wait] "a"(wait),  // %[wait] refers to wait.
        [rxdata] "x"(rxdata),  // Register X refers to the same address as rxdata.
        [rxlen] "I"(RX_BUFFER_SIZE),  // %[rxlen] refers to rxlen, in bits.
        [txdata] "z"(txdata),  // Register Z refers to the same address as txdata.
        [idPos] "I"(TX_ID_POS),    // %[idPos] refers to the offset of the identity data in the snapshot.
        [idSize] "I"(TX_ID_SIZE),  // %[idSize] refers to the size of the identity data in the snapshot.
        [statusPos] "I"(TX_STATUS_POS),    // %[statusPos] refers to the offset of the status data in the snapshot.
        [statusSize] "I"(TX_STATUS_SIZE),  // %[statusSize] refers to the size of the status data in the snapshot.
        "0"(num),  // The same register as %[num] above is used for num here.
        "1"(debug0),  // The same register as %[debug0] above is used for debug0 here.
        "2"(debug1),  // The same register as %[debug1] above is used for debug1 here.
        "3"(debug2),  // The same register as %[debug2] above is used for debug2 here.
        "4"(debug3)   // The same register as %[debug3] above is used for debug3 here.
      : "r16", "r17", "r18", "r19", "r28", "r29"  // These registers are clobbered.
      );

  if (num == 254) {
    // Bad command. Wait long enough so that whatever may have come after the command
    // has passed before we re-enable interrupts.
    unsigned int bitsLeft = (RX_BUFFER_SIZE - 1) * 8;  // We received one byte already.
    unsigned int usLeft = (bitsLeft + 1) * 4; //  4us per bit plus the stop bit.
    delayMicroseconds(usLeft);
  }

  // Restore interrupts to whatever they were before we disabled them.
  // Before doing so, clear the interrupt flag so the panic handler
  // doesn't run; this signal was correctly handled.
  EIFR |= (1 << N64_CONSOLE_DATA_INT_FLAG);
  SREG = oldSREG;
 
  if (num > RX_BUFFER_SIZE) {
    DEBUG_PRINT(F("Error on console, "));
    
    if (num == 255) {
      // OOM, fix up debug1 as an offset.
      debug0 -= lowByte((uint16_t) txdata);
      
      DEBUG_PRINT(F("OOM."));
      DEBUG_PRINT(F(" Tx Position: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", Tx Size: "));
      DEBUG_PRINT(debug1);
      DEBUG_PRINT(F(", debug2: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else if (num == 254) {
      DEBUG_PRINT(F("bad command."));
      DEBUG_PRINT(F(" Command: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", debug1: "));
      DEBUG_PRINT(debug1);
      DEBUG_PRINT(F(", debug2: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else if (num == 253) {
      DEBUG_PRINT(F("bad timeout."));
      DEBUG_PRINT(F(" Curent Byte: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", Current Bit: "));
      DEBUG_PRINT(debug1, BIN);
      DEBUG_PRINT(F(", Rx Bytes: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else if (num == 252) {
      DEBUG_PRINT(F("bad transmission setup."));
      DEBUG_PRINT(F(" Curent Byte: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", Current Bit: "));
      DEBUG_PRINT(debug1, BIN);
      DEBUG_PRINT(F(", Rx Bytes: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else {
      DEBUG_PRINT(F("unknown reason."));
      DEBUG_PRINT(F(" num: "));
      DEBUG_PRINT(num);
      DEBUG_PRINT(F(", debug0: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", debug1: "));
      DEBUG_PRINT(debug1);
      DEBUG_PRINT(F(", debug2: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    }
  }
  
  return num <= RX_BUFFER_SIZE;
}


// NOTE(ngorski): This next function may be slightly bit-rotten, as
// handleN64Command is more important to maintain for playback and
// that is my primary focus; this can be seen as handleN64Command
// but with the receive-send portions swapped.

/**
 * Send an N64 command to the controller.
 *
 * This sends and receives bytes MSB to LSB. For example, the buffer:
 *     10010101 10101101
 * is sent and received in this order:
 *     ABCDEFGH IJKLMNOP
 * Make sure the send and receive buffers are prepared for this order.
 *
 * @param txdata The buffer of data to transmit.
 * @param txlen  The size of the txdata buffer.
 * @param rxdata The buffer for receiving data.
 * @param rxlen  The size of the rxdata buffer. Cannot be 255.
 * @return True if all rxlen bytes of response were read without timeout, false otherwise.
 */
boolean commandN64Controller(const uint8_t txdata[], const uint8_t txlen, uint8_t rxdata[], const uint8_t rxlen) {
  uint8_t num = 0;
  uint8_t debug0 = 0;
  uint8_t debug1 = 0;
  uint8_t debug2 = 0;
  uint8_t debug3 = 0;
  
  // Disable interrupts, and store old interrupt status for later.
  const uint8_t oldSREG = SREG; 
  noInterrupts();

  asm volatile("\n"
      "/* N64 Controller Tx/Rx assembly block. */"                                                  "\n"
      // Prepares the next byte to send.
      // Takes 6 cycles if not done sending, 5 if done sending.
      "nextByte%=:            "                                                                     "\n"
      "  cpi %[txlen], 0x00   " /* Compare txlen with zero.                          1 cycle.    */ "\n"
      "  breq doneSending%=   " /* If equal, jump to doneSending.                    1/2 cycles. */ "\n"
      "  dec %[txlen]         " /* Decrement txlen by one.                           1 cycle.    */ "\n"
      "  ld r16, z+           " /* Load value at address z into r16, incrementing z. 2 cycles.   */ "\n"
      "  ldi r17, 0x80        " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n"

      // Checks the next bit to send by testing if it's on or off.
      // Both branches (send 1 or send 0) take 4 cycles before the start sub-bit.
      "nextBit%=:             "                                                                     "\n"
      "  mov r18, r16         " /* Copy r16 into r18.                                1 cycle.    */ "\n"
      "  and r18, r17         " /* r18 equals bitwise-and on r18 nd r17.             1 cycle.    */ "\n"
      "  breq send0%=         " /* If zero, bit was off; send 0. Otherwise send 1.   1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"

      // The bit to send is 1. Pull LOW for 1us for the start sub-bit, then go HIGH
      // for 3us (keeping in mind the jump target's time at the end of the block).
      "send1%=:               "                                                                     "\n"
      // Going LOW for 1us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 1us LOW. With the 3 cycles since the sbi instruction, at
      // the termination the loop the total LOW wait time so far will be 14
      // cycles. Then two more cycles are used with a nop, giving 16 cycles total.
      "  ldi r18, 4           " /* Counter for wait loop (11 cycles of loop total).  1 cycle.    */ "\n"
      "send1lp0%=:            "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send1lp0%=      " /* If not zero, jump to send1lp0%=.                  1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (15 of 16 cycles).                1 cycle.    */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"
      // Going HIGH for 3us.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      // Wait loop for 3us HIGH. With the 3 cycles since the cbi instruction, at
      // the termination of the loop the total HIGH wait time so far will be 35
      // cycles. Then one more cycle is used shifting the bit mask. 12 cycles need
      // to be used before the next start sub-bit is sent. If the end jump is to
      // nextByte%=, 2 cycles are used making the jump and 10 cycles are used in
      // nextByte%= and nextBit%=, giving 12 cycles total. Otherwise, the jump is to
      // nextBit%=; to make the breq a consistent 2 cycles a nop is inserted after it,
      // then to make up for not going through nextByte%= 4 nops are used. Then 2 are
      // used for the rjmp, giving 12 cycles total.
      "  ldi r18, 11          " /* Counter for wait loop (32 cycles of loop total).  1 cycle.    */ "\n"
      "send1lp1%=:                 "                                                                "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send1lp1%=      " /* If not zero, jump to send1lp1%=.                  1/2 cycles. */ "\n"
      "  lsr r17              " /* Shift the bit in the mask to the right.           1 cycle.    */ "\n"
      "  breq nextByte%=      " /* If no more bits to check, jump to nextByte%=.     1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (1 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (2 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (3 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (4 of 4). 1 cycle.    */ "\n"
      "  rjmp nextBit%=       " /* Send next bit, and last 2 cycles of nextByte%=.   2 cycles.   */ "\n"

      // The bit to send is 0. Pull LOW for 3us, then go HIGH for 1us for the stop
      // sub-bit (keeping in mind the jump target's time at the end of the block).
      "send0%=:               "                                                                     "\n"
      // Going LOW for 3us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 3us LOW. With the 3 cycles since the sbi instruction, at
      // the termination of the loop the total LOW wait time so far will be 47
      // cycles. Then one more cycle is used with a nop, giving 48 cycles total.
      "  ldi r18, 15          " /* Counter for wait loop (44 cycles of loop total).  1 cycle.    */ "\n"
      "send0lp0%=:            "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne send0lp0%=      " /* If not zero, jump to send0lp0%=.                  1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 3us (48 of 48 cycles).                1 cycle.    */ "\n"
      // Going HIGH for 1us.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      // There is no wait loop for 1us HIGH because of the jump target's cycles.
      // The cbi and nop instructions take 3 cycles, then one more cycle is used
      // shifting the bit mask. As was the case for send1lp1%=, 12 cycles need
      // to be used before the next start sub-bit is sent. If the end jump is to
      // nextByte%=, 2 cycles are used making the jump and 10 cycles are used in
      // nextByte%= and nextBit%=, giving 12 cycles total. Otherwise, the jump is to
      // nextBit%=; to make the breq a consistent 2 cycles a nop is inserted after it,
      // then to make up for not going through nextByte%= 4 nops are used. Then 2 are
      // used for the rjmp, giving 12 cycles total.
      "  nop                  " /* Waiting for 1us (3 of 16 cycles).                 1 cycle.    */ "\n"
      "  lsr r17              " /* Shift the bit in the mask to the right.           1 cycle.    */ "\n"
      "  breq nextByte%=      " /* If no more bits to check, jump to nextByte%=.     1/2 cycles. */ "\n"
      "  nop                  " /* Use the extra cycle given for not branching.      1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (1 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (2 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (3 of 4). 1 cycle.    */ "\n"
      "  nop                  " /* Using cycle that would be in nextByte%= (4 of 4). 1 cycle.    */ "\n"
      "  rjmp nextBit%=       " /* Send next bit, and last 2 cycles of nextByte%=.   2 cycles.   */ "\n"

      // Finished sending. If we just sent out a stop sub-bit, it needs to be there
      // for 1us. Compared to sending more data, we left in the middle of running
      // through nextByte%= and nextBit%= which would have taken 10 cycles before
      // finishing the stop sub-bit. 3 cycles were used before arriving here, so we
      // wait for 7 cycles to complete the 1us stop sub-bit waiting time.
      "doneSending%=:         "                                                                     "\n"
      // Wait loop for 1us HIGH. With 1 cycle above, at the termination of the loop
      // the total HIGH wait time so far will be 6 cycles. One more cycle is used
      // afterward with a nop, giving 7 cycles total.
      "  ldi r18, 2           " /* Counter for wait loop (5 cycles of loop total).   1 cycle.    */ "\n"
      "doneSendinglp%=:       "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne doneSendinglp%= " /* If not zero, jump to doneSendinglp%=.             1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"

      // Now send the data stop bit. Like a regular 1 bit, the line will be brought
      // LOW for 1us then HIGH for 3us. Same timing as send1%= without the end jump.
      // Going LOW for 1us.
      "  sbi %[ddr], %[pinNo] " /* Set the pin to be OUTPUT (data LOW to GRND).      2 cycles.   */ "\n"
      // Wait loop for 1us LOW. With the 3 cycles since the sbi instruction, at
      // the termination the loop the total LOW wait time so far will be 14
      // cycles. Then two more cycles are used with a nop, giving 16 cycles total.
      "  ldi r18, 4           " /* Counter for wait loop (11 cycles of loop total).  1 cycle.    */ "\n"
      "stoplp%=:              "                                                                     "\n"
      "  dec r18              " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne stoplp%=        " /* If not zero, jump to stoplp%=.                    1/2 cycles. */ "\n"
      "  nop                  " /* Waiting for 1us (15 of 16 cycles).                1 cycle.    */ "\n"
      "  nop                  " /* Waiting for 1us (16 of 16 cycles).                1 cycle.    */ "\n"
      // Going HIGH for the last time. We don't need to ensure this hits 3us before
      // moving on because until the next request it'll be left HIGH, as is default.
      "  cbi %[ddr], %[pinNo] " /* Set the pin to be INPUT (data HIGH to +3.3v).     2 cycles.   */ "\n"
      
      // If there's no room for receiving, stop.
      "  cpi %[rxlen], 0           " /* Compare rxlen with zero.                          1 cycle.    */ "\n"
      "  breq end%=                " /* If equal, jump to end.                            1/2 cycles. */ "\n"

      // Now receiving. Prepare the byte buffer and bit mask.
      "  clr r18                   " /* Clear the byte buffer.                            1 cycle.    */ "\n"
      "  ldi r17, 0x80             " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n"

      // For the first bit, we set the timeout to max. The controller responds in
      // 1.75us at the latest, and this timeout lasts much longer.
      "  ldi r16, 0xff             " /* Timeout before assuming we read the stop bit.     1 cycle.    */ "\n"
      "  rjmp waitFall%=           " /* Pin was still HIGH, continue waiting.             2 cycles.   */ "\n"
      
      // Setup for a wait fall cycle. The timeout lets us know the last bit was the stop
      // bit. We need to give the console time to follow the 1us HIGH protocol before
      // expecting the fall. Since we came from waitRise%=, we now in the stop sub-bit
      // best case (pin was HIGH the entire time after the start sub-bit of the previous
      // bit was detected, so was detected right away) that it took 4 cycles to get here,
      // so we have 12 more cycles to wait before assuming timeout. From testing, an
      // additional 4 cycles are needed to account for clock drift, giving 16 cycles to wait
      // until timeout (20 total since the line was read HIGH). Setting up the timeout
      // value leaves 15 cycles. Each loop takes 5 cycles, with 2 cycles of timeout handling
      // before actually sampling. By adding 4 wait cycles before beginning the loop
      // (leaving 12 more to go), we can align the timeout cycle with the instruction just
      // prior to the sample. Should we timeout (timeoutDone%= is taken), we'll have waited
      // (on average, account for clock drift) a total of 11 cycles past the full stop bit.
      "setupWaitFall%=:            "                                                                     "\n"
      "  ldi r16, 4                " /* Timeout before assuming we read the stop bit.     1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      // Wait for the line to fall to the start sub-bit of the response.
      "waitFall%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the timeout counter.                    1 cycle.    */ "\n"
      "  breq end%=                " /* If zero, jump to end; stop bit was reached.       1/2 cycles. */ "\n"
      "  sbic %[pin], %[pinNo]     " /* Check pin; if LOW, skip the next instruction.     1/2 cycles. */ "\n"
      "  rjmp waitFall%=           " /* Pin was still HIGH, continue waiting.             2 cycles.   */ "\n"
       
      // The start sub-bit was received; wait as close to 2us after that, then sample.
      // From testing, the waitFall%= loop will execute multiple times before we see
      // the line go LOW (that is, the first sbic is definitely reached and the test
      // does not result in a skip). Since 4 cycles pass between checks, on average
      // the line will have been LOW for 2 cycles before we detected it. With the 2
      // cycles it took to skip the loop jump, we are about 4 cycles into the signal.
      // We need to wait 28 more cycles before we sample to hit the 2us mark on average.
      "  ldi r16, 9                " /* Counter for wait loop (27 cycles of loop total).  1 cycle.    */ "\n"

      // Sample waiting loop.
      "samplelp%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne samplelp%=           " /* If not zero, jump to samplelp%=.                  1/2 cycles. */ "\n"

      // Now perform the sample. The work between here and waitHigh needs to be
      // quick enough (< 1us) that we don't miss the data line going HIGH. However, we
      // need to wait 1us before sampling for the stop sub-bit, otherwise if we just
      // received a 1 and sample the stop-bit as soon as possible, we'll have skipped
      // an entire 1us of signal. This will throw off the stop bit timeout at waitFall%=.
      // To counter-act this, we assume we got here as fast as possible (31 cycles since
      // detecting the start sub-bit), and that we need to wait 48 - 31 = 17 cycles more
      // before sampling. The sample loop has 3 cycles preamble, so we need to ensure all
      // paths from the following computation tree end at 14 cycles. Now should we sample
      // the stop sub-bit immediately because a 1 was sent, it's just as if we sampled it
      // from a perfectly-timed 0 instead.      
      "sample%=:                   "                                                                     "\n"
      "  sbic %[pin], %[pinNo]     " /* Check pin; if LOW, skip the next instruction.     1/2 cycles. */ "\n"
      "  or r18, r17               " /* The pin was HIGH, enable this bit in our byte.    1 cycle.    */ "\n"
      "  lsr r17                   " /* Move on to the next bit in our byte.              1 cycle.    */ "\n"
      "  brne sampleMoreBits%=     " /* If more bits to read, jump to sampleMoreBits%=.   1/2 cycles. */ "\n"
      
      // Finished reading that byte, store it in rxdata.
      "  cp %[num], %[rxlen]       " /* Compare rxlen with num.                           1 cycle.    */ "\n"
      "  breq oomRx%=              " /* If equal, ran out of receive memory.              1/2 cycles. */ "\n"
      "  st x+, r18                " /* Store value in r18 to address x, incrementing x.  2 cycles.   */ "\n"
      "  inc %[num]                " /* Increment the number of received bytes.           1 cycle.    */ "\n"
      "  clr r18                   " /* Clear the byte buffer.                            1 cycle.    */ "\n"
      "  ldi r17, 0x80             " /* Load 0x80 into r17 (leftmost bit on).             1 cycle.    */ "\n" 
      "  nop                       " /* Waiting.                                          1 cycle.    */ "\n"
      "  rjmp setupWaitRise%=      " /* Enough cycles have passed, jump to the sample.    2 cycles.   */ "\n" 
      
      // Had more bits to go and skipped the byte storage section.
      "sampleMoreBits%=:           "                                                                     "\n"
      "  ldi r16, 3                " /* Counter for wait loop (8 cycles of loop total).  1 cycle.    */ "\n"
      
      // The wait loop shared by both above paths.
      "samplePostDelay%=:          "                                                                     "\n"
      "  dec r16                   " /* Decrement the wait counter.                       1 cycle.    */ "\n"
      "  brne samplePostDelay%=    " /* If not zero, jump to samplePostDelay%=.           1/2 cycles. */ "\n"     
      
      // To move on to the next bit, first we wait for the stop sub-bit. The entire
      // process should fit within 4us, so we need to wait at least that long since
      // we first saw the start sub-bit. This timeout would be an unexpected error,
      // though, so as long as we don't wait too long and mistakingly read a rise
      // from a subsequent request as one for this request, we're fine. Since the
      // console will wait ~200us before bugging us again, a 0xff timeout is fine.
      "setupWaitRise%=:            "                                                                     "\n"
      "  ldi r16, 0xff             " /* Timeout before assuming no more data is coming.   1 cycle.    */ "\n"
      // Wait for the line to rise to the stop sub-bit of the response.
      "waitRise%=:                 "                                                                     "\n"
      "  dec r16                   " /* Decrement the timeout counter.                    1 cycle.    */ "\n"
      "  breq timeoutBad%=         " /* If zero, jump to timeout handler.                 1/2 cycles. */ "\n"
      "  sbis %[pin], %[pinNo]     " /* Check pin; if HIGH, skip the next instruction.    1/2 cycles. */ "\n"
      "  rjmp waitRise%=           " /* Pin was still LOW, continue waiting.              2 cycles.   */ "\n"
      "  rjmp setupWaitFall%=      " /* Pin has gone HIGH, move on to next bit.           2 cycles.   */ "\n"

      // An error occurred. Indicate this by setting num to max. We also may have 3 slots
      // to store debug information, though the reuse of registers makes this difficult.

      // OOM receiving: 0xff (255), loaded command length, and transmission buffer position.
      "oomRx%=:                    "                                                                     "\n"
      "  ldi %[num], 0xff          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
      
      // Timeout on stop sub-bit: 0xfd (253), current byte, current bit, number of bytes.
      "timeoutBad%=:               "                                                                     "\n" 
      "  mov %[debug0], r18       " /* Show the current byte.                             1 cycle.    */ "\n"
      "  mov %[debug1], r17       " /* Show the current bit.                              1 cycle.    */ "\n"
      "  mov %[debug2], %[num]    " /* Show number of received bytes.                     1 cycle.    */ "\n"
      "  ldi %[num], 0xfd          " /* Set num to the magic error value.                 1 cycle.    */ "\n"
      "  rjmp end%=                " /* Finished attempting to receive.                   2 cycles.   */ "\n"
      // Finished.     
      "end%=:                 "                                                                     "\n"
      : [num] "=a"(num),  // %[num] refers to num.
        [debug0] "=r"(debug0),  // %[debug0] refers to debug0.
        [debug1] "=r"(debug1),  // %[debug1] refers to debug1.
        [debug2] "=r"(debug2),  // %[debug2] refers to debug2.
        [debug3] "=r"(debug3)   // %[debug3] refers to debug3.
      : [ddr] "I"(_SFR_IO_ADDR(N64_CONTROLLER_DATA_DDR)),  // %[ddr] refers to the DDR port IO register.
        [pin] "I"(_SFR_IO_ADDR(N64_CONTROLLER_DATA_PIN)),  // %[pin] refers to the data pin IO register.
        [pinNo] "I"(N64_CONTROLLER_DATA_PIN_NO),  // %[pinNo] refers to the pin bit number within %[pin].
        [txdata] "z"(txdata),  // Register Z refers to the same address as txdata.
        [txlen] "a"(txlen),    // %[txlen] refers to txlen.
        [rxdata] "x"(rxdata),  // Register X refers to the same address as rxdata.
        [rxlen] "a"(rxlen),    // %[rxlen] refers to rxlen.
        "0"(num),  // The same register as %[num] above is used for num here.
        "1"(debug0),  // The same register as %[debug0] above is used for debug0 here.
        "2"(debug1),  // The same register as %[debug1] above is used for debug1 here.
        "3"(debug2),  // The same register as %[debug2] above is used for debug2 here.
        "4"(debug3)   // The same register as %[debug3] above is used for debug3 here.
      : "r16", "r17", "r18"  // These registers are clobbered.
      );

  // Restore interrupts to whatever they were before we disabled them.
  // Before doing so, clear the interrupt flag so the panic handler
  // doesn't run; this signal was correctly handled.
  EIFR |= (1 << N64_CONTROLLER_DATA_INT_FLAG);
  SREG = oldSREG;
  
  if (num != rxlen) {
    DEBUG_PRINT(F("Error on controller, "));
    
    if (num == 255) {
      // OOM, fix up debug1 as an offset.
      debug0 -= lowByte((uint16_t) txdata);
      
      DEBUG_PRINT(F("OOM."));
      DEBUG_PRINT(F(", debug0: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", debug1: "));
      DEBUG_PRINT(debug1);
      DEBUG_PRINT(F(", debug2: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else if (num == 253) {
      DEBUG_PRINT(F("bad timeout."));
      DEBUG_PRINT(F(" Curent Byte: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", Current Bit: "));
      DEBUG_PRINT(debug1, BIN);
      DEBUG_PRINT(F(", Rx Bytes: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    } else {
      DEBUG_PRINT(F("unknown reason."));
      DEBUG_PRINT(F(" num: "));
      DEBUG_PRINT(num);
      DEBUG_PRINT(F(", debug0: "));
      DEBUG_PRINT(debug0);
      DEBUG_PRINT(F(", debug1: "));
      DEBUG_PRINT(debug1);
      DEBUG_PRINT(F(", debug2: "));
      DEBUG_PRINT(debug2);
      DEBUG_PRINT(F(", debug3: "));
      DEBUG_PRINT(debug3);
      DEBUG_PRINTLN();
    }
  }

  return num == rxlen;
}

/**
 * Convert the raw identity response from the controller
 * into a specific n64ControllerIdentity enumeration value.
 *
 * @param rawResult The raw response from the controller.
 * @param result Variable to store the result into.
 * @return True if the identity was successfully read, false otherwise.
 *         If false, the value of result is undefined.
 */
boolean convertN64RawControllerIdentity(const uint8_t rawResult[3], n64ControllerIdentity& result) {
  // Magic numbers from the controller.
  if (rawResult[0] != 0x05 || rawResult[1] != 0x00) {
    return false;
  }

  switch (rawResult[2]) {
  case 0x00: result = IDENTITY_NO_EXPANSION_PACK;       return true;
  case 0x01: result = IDENTITY_EXPANSION_PACK;          return true;
  case 0x02: result = IDENTITY_EXPANSION_PACK_REMOVED;  return true;
  case 0x03: result = IDENTITY_EXPANSION_PACK_INSERTED; return true;
  case 0x04: result = IDENTITY_CONTROLLER_ERROR;        return true;
  default: return false;  // Unexpected.
  }
}

/**
 * Get the identity of the N64 controller.
 *
 * @param result Variable to store the result into.
 * @return True if the identity was successfully read, false otherwise.
 *         If false, the value of result is undefined.
 */
boolean getN64ControllerIdentity(n64ControllerIdentity& result) {
  const uint8_t request[] = { N64_COMMAND_IDENTITY };
  
  uint8_t rawResult[3] = {};
  
  if (!commandN64Controller(request, sizeof(request), rawResult, sizeof(rawResult))) {
    return false;
  }

  return convertN64RawControllerIdentity(rawResult, result);
}

/**
 * Reset and get the identity of the N64 controller.
 *
 * @param result Variable to store the result into.
 * @return True if the identity was successfully read, false otherwise.
 *         If false, the value of result is undefined.
 */
boolean getN64ControllerResetIdentity(n64ControllerIdentity& result) {
  const uint8_t request[] = { N64_COMMAND_RESET };
  
  uint8_t rawResult[3] = {};
  
  if (!commandN64Controller(request, sizeof(request), rawResult, sizeof(rawResult))) {
    return false;
  }

  return convertN64RawControllerIdentity(rawResult, result);
}

/**
 * Get the status of the N64 controller.
 *
 * @param result Variable to store the result into.
 * @return True if the status was successfully read, false otherwise.
 *         If false, the value of result is undefined.
 */
boolean getN64ControllerStatus(n64ControllerStatus& result) {
  const uint8_t request[] = { N64_COMMAND_STATUS };
  
  return commandN64Controller(request, sizeof(request), (uint8_t*) &result, sizeof(result));
}

uint8_t rxdata[RX_BUFFER_SIZE] = {};
uint8_t txdata[TX_BUFFER_SIZE] = { 0x05, 0x00, 0x00, };

void updateId() {
  n64ControllerIdentity id;
  if (!getN64ControllerIdentity(id)) {
    Serial.println(F("Controller ID Error!"));    
    return;
  }
 
  txdata[2] = id;
}

void updateStatus() {
  // fetch new button status
  n64ControllerStatus& btn = *(n64ControllerStatus*) (txdata + TX_STATUS_POS);
  if (!getN64ControllerStatus(btn)) {
    Serial.println(F("Controller Status Error!"));
    return;
  }
}

void setup() {
  Serial.begin(SERIAL_BITS_PER_SECOND);
  
  pinMode(ACTIVE_PIN, OUTPUT);
  pinMode(PANIC_PIN, OUTPUT);
}

// Core state machine and setup.
enum State {
  STATE_ECHO,
  STATE_PANIC,
};

State state = STATE_ECHO;

void loop() {
  if (state == STATE_ECHO) {
    // Left as an exercise for the reader. Try something like this:

    for (boolean wait = true; ; wait = false) {
      digitalWrite(ACTIVE_PIN, false);

      // We may be printing this, so clear it from last time
      // to ensure no misinformation occurs. Magic number.
      rxdata[0] = 0xaa;
      
      while (!handleN64Command(rxdata, txdata, wait)) {
        DEBUG_PRINTLN("Bad console communication.");
        
        state = STATE_PANIC;
        return;
      }
      
      digitalWrite(ACTIVE_PIN, true);
      
      // We just echo the actual controller data to the console.
      // This delay *may* be too slow for the console to be happy.
      updateStatus();
    }
  } else {
    // Panic.
    DEBUG_PRINTLN("Panicking.");
    
    DEBUG_PRINT(F("Rx Data Contents: "));
    
    for (uint8_t index = 0; index != RX_BUFFER_SIZE; ++index) {
      DEBUG_PRINT(rxdata[index]);
      DEBUG_PRINT(F(" "));
    }
    
    DEBUG_PRINTLN();
    
    DEBUG_PRINT(F("Tx Data Contents: "));
    
    for (uint8_t index = 0; index != TX_BUFFER_SIZE; ++index) {
      DEBUG_PRINT(txdata[index]);
      DEBUG_PRINT(F(" "));
    }
    
    DEBUG_PRINTLN();
    
    digitalWrite(ACTIVE_PIN, false);
    
    for (;;) {
      digitalWrite(PANIC_PIN, !digitalRead(PANIC_PIN));
      delay(125);
    }
  }
}

