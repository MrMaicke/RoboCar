/*
 * Ultrasonic.h
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

#ifndef UltrasonicMult_h
#define UltrasonicMult_h

/*
 * Values of divisors
 */
#define CM 28
#define INC 71

class UltrasonicMult {
  public:
    UltrasonicMult(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut = 20000UL);
    unsigned int distanceRead(uint8_t und);
    void setTimeout(unsigned long timeOut) {timeout = timeOut;}

  private:
    uint8_t trig;
    uint8_t echo;
    unsigned long timeout;
    unsigned int timing();
};

#endif // Ultrasonic_h
