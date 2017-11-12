/*
 * Ultrasonic.cpp
 *
 * Library for Ultrasonic Ranging Module in a minimalist way
 *
 * created 3 Apr 2014
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 23 Jan 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 04 Mar 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 15 May 2017
 * by Eliot Lim    (github: @eliotlim)
 * 
 * adaptado por Fábio Ferreira em 12/11/2017
 *
 * Released into the MIT License.
 */

#include <Arduino.h>
#include "UltrasonicMult.h"

UltrasonicMult::UltrasonicMult(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut) {
  trig = trigPin;
  echo = echoPin;
  timeout = timeOut;
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

unsigned int UltrasonicMult::timing() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  return pulseIn(echo, HIGH, timeout); // duration
}

unsigned int UltrasonicMult::distanceRead(uint8_t und) {
  return timing() / und / 2;  //distance by divisor
}