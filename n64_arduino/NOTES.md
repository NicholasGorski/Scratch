# N64 Verifier Notes

## Communication

### Hardware

The N64 controller has three connections. Looking at the *controller* plug, flat-side down:

<pre>
    _____
  / _____ \
 // A B C \\
||  ◎ ◎ ◎  ||
 \⎺⎺⎺⎺⎺⎺⎺⎺⎺/
  \       /
    ⎺⎺⎺⎺⎺
</pre>

A is ground, B is data, and C is +3.3v. (The N64 console actually provides +3.6v, but +3.3v works fine.)

The resistance through the controller between A and C is about 540Ω.

## Protocol

The data line is kept HIGH with a pull-up resistor to +3.3v. Communication of a single bit takes place in this fashion:

1. Start sub-bit is sent: line brought LOW for 1us.
2. First data sub-bit sent for 1us. (Line brought LOW if 0, HIGH if 1.)
3. Second data sub-bit sent for 1us. (Repeat of step 2.)
4. Stop sub-bit is sent: line brought high for 1us (where it remains until further transmission).

Total time per bit is 4us. After all the bits have been sent a stop bit is sent, which is just a regular 1 bit (i.e., brought LOW for 1us, then HIGH for 3us)

For the receiver, after detecting the start sub-bit it waits 2us (which will align it between steps 2 and 3 of the sender), then samples to determine the bit status.

The data bus is never in contention because the controller only responds to console commands. The overall cycle is then:

1. Idle, controller listening.
2. Console commands controller.
3. Controller responds, console listening.
4. Controller has ~200us cool-down returning to idle.

From testing, the controller consistently responds between 1.4375us and 1.75us after the entire stop bit is sent. However, the delay can be as large as somewhere between 61.37500us to 61.75us before the console will timeout on the response. What effect this has on gameplay is unknown.

The commands sent from the console are:

**0x00: Identity:**

This gets the identity of the controller (what configuration it is in), and receives (in this order):

1. 0x05 (magic number #1): 8 bits
2. 0x00 (magic number #2): 8 bits
3. 0x0X: 8 bits

Where X can be 0, 1, 2, 3, or 4. Meanings:

| Final Byte | Meaning                                |
|:-----------|:---------------------------------------|
| 0x00       | There is no expansion pack attached.   |
| 0x01       | There is an expansion pack attached.   |
| 0x02       | An expansion pack was just removed.    |
| 0x03       | An expansion pack was just attached.   |
| 0x04       | Previous command resulted in an error. |

When a controller is plugged in with no expansion pack, 0x00 is returned.

When an expansion pack is inserted, the first query afterwards will return 0x03 indicating that a pack was just attached. The subsequent query will return 0x01 instead, indicating simply that the pack is attached.

However, when the expansion pack is then unplugged, 0x02 will be the only returned response, even after multiple queries; 0x00 is never reached again. The only time 0x00 is returned is when the controller is first plugged in without an expansion pack. This is asymmetric compared to the behavior of 0x01/0x03.

If an expansion pack is already attached when the controller is plugged in, the response will be 0x01 directly.

**0xff: Reset:**

This is the same as the Identity command (0x00), except that it causes the control to reset and recalibrate. (As if L+R+Start were pressed.)

**0x01: Status:**

This gets the status of the controller, and receives (in this order):

1. A: 1 bit
2. B: 1 bit
3. Z: 1 bit
4. Start: 1 bit
5. D-Up: 1 bit
6. D-Down: 1 bit
7. D-Left: 1 bit
8. D-Right: 1 bit
9. Reset (L+R+Start is pressed): 1 bit
10. Unused (always zero): 1 bit
11. L: 1 bit
12. R: 1 bit
13. C-Up: 1 bit
14. C-Down: 1 bit
15. C-Left: 1 bit
16. C-Right: 1 bit
17. Signed X position: 8 bits
18. Signed Y position: 8 bits

For the X and Y position, from centered the joystick itself is physically limited. On a new controller, the X-axis reaches from -90 to 81, and the Y-axis reaches from -83 to 73. Across controllers this changes depending on the joystick's condition and force applied.

The entire range can be reached by recalibrating the joystick off-center (e.g., pull full-right, press L+R+Start/send Reset command to recalibrate this as center, then pull full-left to reach -128).

## Setup

**Required:**

1. Arduino Uno (ATmega328, 16MHz)
2. Breadboard and components
3. N64 console with an N64 controller
4. Computer to connect the Arduino to, with AVR GCC

*Using the verifier with a different setup is possible, but is outside the realm of these notes.*

**Recommended:**

Acquire an N64 controller extension cable. Cut it in half and strip the ends of the wire at the cut; these wires can be used to connect the console and controller with the Arduino circuit (pay attention to which wire is which, and use a multimeter to check which plug port goes to which wire color; the canonical wire colorings may not be respected by the manufacturer). This makes it easy to connect the controller to the console (just use the extension ports) while still having wires to plug into Arduino.

Otherwise, you can place wires directly into the console and controller; just beware the connection may be difficult to maintain.

### The software

**Warning:**

**Upload the program to Arduino before starting the circuit**. You could damage the N64 controller should it be connected to any voltage source above 3.6v (such as an output pin driven HIGH); the program will not do this, of course, so building the circuit will be safe after the program is running.

#### Arduino program

The program was written specifically for an Arduino Uno running ATmega328 at 16MHz, and contains inline assembly for time-sensitive sections. It is located in this directory, as n64_arduino.ino. Upload this to Arduino, and keep the serial connection connected.

Like all verifier programs, the rest of the process is controlled from the alice-verifier program. Run it and select "N64".

### The circuit

See the images in this folder. Not that the SD card portion has been greyed out.

TODO(ngorski): Diagram.

#### Controller

Connect the +3.3v pin from the Arduino to the breadboard's supply bus strip. Connect the ground pin from the Arduino to breadboard's ground bus strip.

Connect pin 2 to a strip in the breadboard, and connect controller line B to this strip. Place a 1kΩ pull-up resistor between +3.3v and this strip. With pin 2 set to INPUT (without internal pull-up, the default), this will keep the line 3.3v HIGH by default. By setting pin 2 to OUTPUT (and writing LOW, the default), the line will go LOW.

**Never drive pin 2 5v HIGH.** This can damage the controller.

#### Console

**Before you start:**

Make sure never to short the +3.6v line and ground line from the console. Doing so will kill the console (likely the AC adapter). If you're using an extension cable make sure the non-data wires are secured away from each other, by putting them in separate disconnected breadboard holes.

Connect pin 3 to a strip in the breadboard separate from the controller, and connect the console line B to this strip. (With the +3.6 and ground lines connected to nothing; they only supple voltage to the controller and are not used by the console.) Place a 1kΩ pull-up resistor between +3.3v and this strip. With pin 3 set to INPUT (without internal pull-up, the default), this will keep the line 3.3v HIGH by default. By setting pin 3 to OUTPUT (and writing LOW, the default), the line will go LOW.

**Never drive pin 9 5v HIGH.** This can damage the console.

You can now run the verifier.
