//===------_ Hacking STEM – MeasuringToolsCode.X.X.X.ino – Microbit _-----===//
// For use with the "MEASURING TOOLS Using the Pythagorean Theorem to Explore
// Topography in 2D/3D Space" lesson plan available from Microsoft Education
// Workshop at https://www.microsoft.com/en-us/education/education-workshop/measuring-tools.aspx
// http://aka.ms/hackingSTEM
//
// Overview:
// This project uses Rotary Encoders to find angles.
//
// Output to Serial reports angle, count, and revolutions of two encoders, in
// format: "angleOne,countOne,revolutionsOne,angleTwo,countTwo,revolutionsTwo"
// example:
// "353,-1087,-4,45,45,0,"
//
// For encoder one:
// AngleOne: Angle, in degrees. Can be positive or negative.
// CountOne: Count from the rotary encoder signal, can be positive or negative.
// RevolutionsOne: Count of full rotations, can be positive or negative.
//
// For encoder two:
// AngleTwo: Angle, in degrees. Can be positive or negative.
// CountTwo: Count from the rotary encoder signal, can be positive or negative.
// RevolutionsOne: Count of full rotations, can be positive or negative.
//
// How rotary encoders work:
// Rotary encoders interface through two digital pins on the arduino. State
// changes on these pins will progress through four combinations per step
//
// So to transition from step 0 to step 1, there will be four state changes:
//
// Step ZERO:           LOW LOW
// Step QUARTER:        HIGH LOW
// Step HALF:           HIGH HIGH
// Step THREE-QUARTER:  LOW HIGH
// Step ZERO:           LOW LOW
//
//   Direction:
//   To go the other direction, from 1 to 0, the pins will go through the same
//   four state changes in reverse. The direction of change can be understood by
//   comparing the current state to the last state.
//
//   Step Count:
//   To keep track of the step count, we add 1 for every step encountered in
//   the forward rotation and subtract one for each step encountered in the
//   backward direction. For example, the step count of four steps forward and
//   one steps back is 3, or (1 + 1 + 1 + 1 - 1). The step count of 3 steps
//   backward, two steps forward, and one step backward is negative 2, or
//   (0 - 1 - 1 - 1 + 1 + 1 -1)
//
//   Degree:
//   With an encoder that has 360 steps per revoltion, degree is simply the
//   remainder left over by dividing step count by 360. For example,
//   The remainder of 792 Steps / 360 Steps Per Revolution is 72, so the
//   current degree is 72.
//
//   Rotation Count:
//   To count rotations simply sum every positive and negative step and divide
//   by the steps per revolution of the encoder and round down to get a whole
//   number. For example: 792 Steps / 360 Steps Per Revolution is 2 steps
//   (that's 2.2 rounded down).
//
// With the 360 step rotary encoder degree translation is easy, the
// current step position will be
//
// This project uses a micro:bit controller board, information at:
// http://microbit.org/
//
// This project uses a 360 steps per revoltion rotary encoder, such as:
// lpd3806-360bm-g5-24c
//
// Note: Lancester micro:bit micropython an C libraries do not make available 
// API methods with sufficient speed to handle the rotary encoder pulses. 
// Therefore, this code is written against the lower level mbed library, and 
// more specifically InterruptIn, which allows us to update state on rise
// and fall of our rotary encoder pins.
//
// Comments, contributions, suggestions, bug reports, and feature requests
// are welcome! For source code and bug reports see:
// http://github.com/[TODO github path to Hacking STEM]
//
// Copyright [year], [your name] Microsoft EDU Workshop - HackingSTEM
// MIT License terms detailed in LICENSE.md
//===----------------------------------------------------------------------===//


#include "mbed.h"
#include <string>

//===----------------------------------------------------------------------===//
// Informational section. This is a list of some mbed pin names to micro:bit 
// pin numbers. I couldn't find this documented elsewhere.
//
//
// mbed		ubit
// name		number
// P0_16 	16
// P0_10 	9
// P0_18 	8
// P0_11 	7
// P0_12 	6
// P0_17 	5
// P0_5  	4
// P0_4  	3
// P0_1  	2
// P0_2  	1
// P0_3  	0
// 
// If porting to another microcontroller, simply change pins below
//===----------------------------------------------------------------------===//

InterruptIn encoderA0(P0_3);	// Rotary Encoder A, pin 0 interrupt call back
InterruptIn encoderA1(P0_2);	// Rotary Encoder B, pin 2 interrupt call back 
DigitalOut ledA6(P0_12);	// LED indicator for state of Encoder A pin 0
DigitalOut ledA7(P0_11);	// LED indicator for state of Encoder A pin 2

InterruptIn encoderB2(P0_1);	// Rotary Encoder B, pin 1 interrupt call back
InterruptIn encoderB3(P0_4);	// Rotary Encoder B, pin 4 interrupt call back
DigitalOut ledB8(P0_18);	// LED indicator for state of Encoder B pin 1
DigitalOut ledB9(P0_10);	// LED indicator for state of Encoder B pin 4

DigitalOut led16(P0_16);	// LED indicator for serial loop
Serial serial(USBTX, USBRX);	// Serial connection to computer

float encoderADegrees = 0;  // # of steps encoder A has turned. pos or neg
long skipCountA = 0;  // Count of times loop found itself in unknown state

float encoderBDegrees = 0;  // # of steps encoder A has turned. pos or neg
long skipCountB = 0;  // Count of times loop found itself in unknown state

//
// Step state for rotary encoder.
//
// In terms of rotary encoder pin state, enum corresponds to:
//
// Step ZERO:           LOW LOW
// Step QUARTER:        HIGH LOW
// Step HALF:           HIGH HIGH
// Step THREE-QUARTER:  LOW HIGH
// Step ZERO:           LOW LOW
enum StepState {
	ZERO, QUARTER, HALF, THREE_QUARTER
};

StepState stepStateA = ZERO;  // Current step state of rotary A
StepState stepStateB = ZERO;  // Current step state of rotary B

// Callback hooked to Encoder A pin 0 going high. 
// Using the state table it's possible to determine if encoder moves 
// positively to QUARTER step or negatively to HALF step.
// If state does not match expected state, increment skip_count and guess 
// new state (QUARTER), as there's a 50/50 chance of getting it right!
//
// Updates stepStateA
void onRiseA0() {
    ledA6 = 1;
	switch (stepStateA) {
		case ZERO: stepStateA = QUARTER; encoderADegrees += .25; break;
		case THREE_QUARTER: stepStateA = HALF; encoderADegrees -= .25; break;
		default: stepStateA = QUARTER; skipCountA++; //guess current state
	}
}

// Callback hooked to Encoder A pin 1 going high. 
// Using the state table it's possible to determine if encoder moves 
// positively to HALF step or negatively to ZERO step.
// If state does not match expected state, increment skip_count and guess 
// new state (HALF), as there's a 50/50 chance of getting it right!
//
// Updates stepStateA
void onRiseA1() {
    ledA7 = 1;
	switch (stepStateA) {
		case QUARTER: stepStateA = HALF; encoderADegrees += .25; break;
		case ZERO: stepStateA = THREE_QUARTER; encoderADegrees -= .25; break;
		default: stepStateA = HALF; skipCountA++; //guess current state
	}
}

// Callback hooked to Encoder A pin 0 going low. 
// Using the state table it's possible to determine if encoder moves 
// negatively to ZERO step or positively to THREE_QUARTER step.
// If state does not match expected state, increment skip_count and guess 
// new state (ZERO), as there's a 50/50 chance of getting it right!
//
// Updates stepStateA
void onFallA0() {
    ledA6 = 0;
	switch (stepStateA) {
		case QUARTER: stepStateA = ZERO; encoderADegrees -= .25; break;
		case HALF: stepStateA = THREE_QUARTER; encoderADegrees += .25; break;
		default: stepStateA = ZERO; skipCountA++; //guess current state
	}
}
 
// Callback hooked to Encoder A pin 1 going low. 
// Using the state table it's possible to determine if encoder moves 
// negatively to QUARTER step or positively to ZERO step.
// If state does not match expected state, increment skip_count and guess 
// new state (QUARTER), as there's a 50/50 chance of getting it right!
//
// Updates stepStateA
void onFallA1() {
    ledA7 = 0;
	switch (stepStateA) {
		case HALF: stepStateA = QUARTER; encoderADegrees -= .25; break;
		case THREE_QUARTER: stepStateA = ZERO; encoderADegrees += .25; break;
		default: stepStateA = QUARTER; skipCountA++; //guess current state
	}
}

// Callback hooked to Encoder B pin 2 going high. 
// Using the state table it's possible to determine if encoder moves 
// positively to QUARTER step or negatively to HALF step.
// If state does not match expected state, increment skip_count and guess 
// new state (QUARTER), as there's a 50/50 chance of getting it right!
//
// Updates stepStateB
void onRiseB0() {
    ledB8 = 1;
	switch (stepStateB) {
		case ZERO: stepStateB = QUARTER; encoderBDegrees += .25; break;
		case THREE_QUARTER: stepStateB = HALF; encoderBDegrees -= .25; break;
		default: stepStateB = QUARTER; skipCountB++; //guess current state
	}
}

// Callback hooked to Encoder B pin 3 going high. 
// Using the state table it's possible to determine if encoder moves 
// positively to HALF step or negatively to THREE-QUARTER step.
// If state does not match expected state, increment skip_count and guess 
// new state (HALF), as there's a 50/50 chance of getting it right!
//
// Updates stepStateB
void onRiseB1() {
    ledB9 = 1;
	switch (stepStateB) {
		case QUARTER: stepStateB = HALF; encoderBDegrees += .25; break;
		case ZERO: stepStateB = THREE_QUARTER; encoderBDegrees -= .25; break;
		default: stepStateB = HALF; skipCountB++; //guess current state
	}
}

// Callback hooked to Encoder B pin 2 going low. 
// Using the state table it's possible to determine if encoder moves 
// negatively to ZERO step or positively to THREE_QUARTER step.
// If state does not match expected state, increment skip_count and guess 
// new state (ZERO), as there's a 50/50 chance of getting it right!
//
// Updates stepStateB
void onFallB0() {
    ledB8 = 0;
	switch (stepStateB) {
		case QUARTER: stepStateB = ZERO; encoderBDegrees -= .25; break;
		case HALF: stepStateB = THREE_QUARTER; encoderBDegrees += .25; break;
		default: stepStateB = ZERO; skipCountB++; //guess current state
	}
}
 
// Callback hooked to Encoder B pin 3 going low. 
// Using the state table it's possible to determine if encoder moves 
// negatively to QUARTER step or positively to ZERO step.
// If state does not match expected state, increment skip_count and guess 
// new state (QUARTER), as there's a 50/50 chance of getting it right!
//
// Updates stepStateB
void onFallB1() {
    ledB9 = 0;
	switch (stepStateB) {
		case HALF: stepStateB = QUARTER; encoderBDegrees -= .25; break;
		case THREE_QUARTER: stepStateB = ZERO; encoderBDegrees += .25; break;
		default: stepStateB = ZERO; skipCountB++; //guess current state
	}
}

//
// Utility function to pull value from delimited string
//
// Gets value from mDataString using an index and separator
// Example:
// given 'alice,bob,dana' and separator ','
// index 0 returns 'alice'
// index 1 returns 'bob'
// index 2 returns 'dana'
//
// mDataString: String as read from Serial (mInputString)
// separator: Character used to separate values (a comma)
// index: where we want to look in the data 'array' for value
string getValue(string mDataString, char separator, int index) {
  int matchingIndex = 0;
  int strIndex[] = {0, -1};
  int maxIndex = mDataString.length()-1;
  // loop until end of array or until we find a match
  for(int i=0; i<=maxIndex && matchingIndex<=index; i++){
    if(mDataString.at(i)==separator || i==maxIndex){ // if we hit a comma
                                                         // OR end of the array
      matchingIndex++;   // increment to track where we have looked
      strIndex[0] = strIndex[1]+1;   // increment first substring index
      strIndex[1] = (i == maxIndex) ? i+1 : i;   // set second substring index
    }
  }
  // if match return substring or ""
  if (matchingIndex > index) {
    return mDataString.substr(strIndex[0], strIndex[1]);
  } else {
    return "";
  }
}

// main() runs on micro:bit start
int main() {
	// Register callbacks on encoder pin rise and fall
	encoderA0.rise(&onRiseA0);
	encoderA1.rise(&onRiseA1);
	encoderA0.fall(&onFallA0);
	encoderA1.fall(&onFallA1);

	encoderB2.rise(&onRiseB0);
	encoderB3.rise(&onRiseB1);
	encoderB2.fall(&onFallB0);
	encoderB3.fall(&onFallB1);

	// String used later to read characters in from serial
	string serial_in_string = ""; 

    while(1) {           // wait around, interrupts will interrupt this!
        led16 = !led16;  // toggle led
        wait(0.05);
		
		
		// Write all reading results to serial:
		// Output to Serial reports angle, count, and revolutions of two encoders, in
		// format: 
		// "angleOne, countOne, revolutionsOne, angleTwo, countTwo, revolutionsTwo, 
		//  skipCountA, skipCountB"
		// example:
		// "353,-1087,-4,45,45,0,"
	    serial.printf("%i,%i,%i,%i,%i,%i,%ld,%ld,\n", 
		int(encoderADegrees) % 360, 
		int(encoderADegrees), 
		int(encoderADegrees/360), 
		int(encoderBDegrees) % 360, 
		int(encoderBDegrees), 
		int(encoderBDegrees/360) 
		,skipCountA, skipCountB);

		// Read data from serial
		bool not_end_line = true;
		while (serial.readable() && not_end_line) {
			char inChar = (char) serial.getc();
			if (inChar == '\n') {
				not_end_line = false;
				break;
			} else {
				serial_in_string += inChar;
			}
		}

	    // If serial read data is complete, reset zero if indicated
		if (serial_in_string.length() > 0) {
			string reset_string = getValue(serial_in_string,',',0); 
			// string points_string = getValue(serial_in_string,',',1);
			if (reset_string.length() > 0 && reset_string.at(0) != '0') {
				encoderADegrees = 0;
				encoderBDegrees = 0;
			}
			// We do not support revolution step override, but second 
			// variable would provide that
			// if (points_string.length() > 0 && std::stoi(points_string) > 0) {
			// 	serial.printf("____________________________________________PPR:%d \n", std::stoi(points_string));
			// 	pointsPerRevolution = std::stoi(points_string);
			// }
			serial_in_string="";  // reset string for next time
		}
    }
}