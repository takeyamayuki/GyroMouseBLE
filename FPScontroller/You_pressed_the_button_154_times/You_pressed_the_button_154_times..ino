essed the button 23 times.
You pressed the button 24 times.
You pressed the button 25 times.
You pressed the button 26 times.
You pressed the button 27 times.
You pressed the button 28 times.
You pressed the button 29 times.
You pressed the button 30 times.
You pressed the button 31 times.
You pressed the button 32 times.
You pressed the button 33 times.
You pressed the button 34 times.
You pressed the button 35 times.
You pressed the button 36 times.
You pressed the button 37 times.
You pressed the button 38 times.
You pressed the button 39 times.
You pressed the button 40 times.
You pressed the button 41 times.
You pressed the button 42 times.
You pressed the button 43 times.
You pressed the button 44 times.
You pressed the button 45 times.
You pressed the button 46 times.
You pressed the button 47 times.
You pressed the button 48 times.
You pressed the button 49 times.
You pressed the button 50 times.
You pressed the button 51 times.
You pressed the button 52 times.
You pressed the button 53 times.
You pressed the button 54 times.
You pressed the button 55 times.
You pressed the button 56 times.
You pressed the button 57 times.
You pressed the button 58 times.
You pressed the button 59 times.
You pressed the button 60 times.
You pressed the button 61 times.
You pressed the button 62 times.
You pressed the button 63 times.
You pressed the button 64 times.
You pressed the button 65 times.
You pressed the button 66 times.
You pressed the button 67 times.
You pressed the button 68 times.
You pressed the button 69 times.
You pressed the button 70 times.
You pressed the button 71 times.
You pressed the button 72 times.
You pressed the button 73 times.
You pressed the button 74 times.
You pressed the button 75 times.
You pressed the button 76 times.
You pressed the button 77 times.
You pressed the button 78 times.
You pressed the button 79 times.
You pressed the button 80 times.
You pressed the button 81 times.
You pressed the button 82 times.
You pressed the button 83 times.
You pressed the button 84 times.
You pressed the button 85 times.
You pressed the button 86 times.
You pressed the button 87 times.
You pressed the button 88 times.
You pressed the button 89 times.
You pressed the button 90 times.
You pressed the button 91 times.
You pressed the button 92 times.
You pressed the button 93 times.
You pressed the button 94 times.
You pressed the button 95 times.
You pressed the button 96 times.
You pressed the button 97 times.
You pressed the button 98 times.
You pressed the button 99 times.
You pressed the button 100 times.
You pressed the button 101 times.
You pressed the button 102 times.
You pressed the button 103 times.
You pressed the button 104 times.
You pressed the button 105 times.
You pressed the button 106 times.
You pressed the button 107 times.
You pressed the button 108 times.
You pressed the button 109 times.
You pressed the button 110 times.
You pressed the button 111 times.
You pressed the button 112 times.
You pressed the button 113 times.
You pressed the button 114 times.
You pressed the button 115 times.
You pressed the button 116 times.
You pressed the button 117 times.
You pressed the button 118 times.
You pressed the button 119 times.
You pressed the button 120 times.
You pressed the button 121 times.
You pressed the button 122 times.
You pressed the button 123 times.
You pressed the button 124 times.
You pressed the button 125 times.
You pressed the button 126 times.
You pressed the button 127 times.
You pressed the button 128 times.
You pressed the button 129 times.
You pressed the button 130 times.
You pressed the button 131 times.
You pressed the button 132 times.
You pressed the button 133 times.
You pressed the button 134 times.
You pressed the button 135 times.
You pressed the button 136 times.
You pressed the button 137 times.
You pressed the button 138 times.
You pressed the button 139 times.
You pressed the button 140 times.
You pressed the button 141 times.
You pressed the button 142 times.
You pressed the button 143 times.
You pressed the button 144 times.
You pressed the button 145 times.
You pressed the button 146 times.
You pressed the button 147 times.
You pressed the button 148 times.
You pressed the button 149 times.
You pressed the button 150 times.
/*
  Keyboard Message test

  For the Arduino Leonardo and Micro.

  Sends a text string when a button is pressed.

  The circuit:
  - pushbutton attached from pin 4 to +5V
  - 10 kilohm resistor attached from pin 4 to ground

  created 24 Oct 2011
  modified 27 Mar 2012
  by Tom Igoe
  modified 11 Nov 2013
  by Scott Fitzgerald

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/KeyboardMessage
*/

#include "Keyboard.h"

const int buttonPin = 4;          // input pin for pushbutton
int previousButtonState = HIGH;   // for checking the state of a pushButton
int counter = 0;                  // button push counter

void setup() {
  // make the pushButton pin an input:
  pinMode(buttonPin, INPUT);
  // initialize control over the keyboard:
  Keyboard.begin();
}

void loop() {
  // read the pushbutton:
  int buttonState = digitalRead(buttonPin);
  // if the button state has changed,
  if ((buttonState != previousButtonState)
      // and it's currently pressed:
      && (buttonState == HIGH)) {
    // increment the button counter
    counter++;
    // type out a message
    Keyboard.print("You pressed the button ");
    Keyboard.print(counter);
    Keyboard.println(" times.");
  }
  // save the current button state for comparison next time:
  previousButtonState = buttonState;
}
