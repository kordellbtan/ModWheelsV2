 * Program to play a simple melody
 *
 * Tones are created by quickly pulsing a speaker on and off 
 *   using PWM, to create signature frequencies.
 *
 * Each note has a frequency, created by varying the period of 
 *  vibration, measured in microseconds. We'll use pulse-width
 *  modulation (PWM) to create that vibration.

 * We calculate the pulse-width to be half the period; we pulse 
 *  the speaker HIGH for 'pulse-width' microseconds, then LOW 
 *  for 'pulse-width' microseconds.
 *  This pulsing creates a vibration of the desired frequency.
 *
 * (cleft) 2005 D. Cuartielles for K3
 * Refactoring and comments 2006 clay.shirky@nyu.edu
 * See NOTES in comments at end for possible improvements
 */
#define speakerOut      22
#define  c            3830    // 261 Hz 
#define  d            3400    // 294 Hz 
#define  e            3038    // 329 Hz 
#define  f            2864    // 349 Hz 
#define  g            2550    // 392 Hz 
#define  a            2272    // 440 Hz 
#define  b            2028    // 493 Hz 
#define  C            1912    // 523 Hz
#define  R               0
  int melody[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C };
  int beats[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 }; 
  int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping.
  
// Set overall tempo
  long tempo = 10000;
  
// Set length of pause between notes
  int pause = 1000;
  
// Loop variable to increase Rest length
  int rest_count = 100;
  int tone_ = 0;
  int beat = 0;
  long duration  = 0;

void song()
{
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {

      digitalWrite(speakerOut,HIGH);
      delayMicroseconds(tone_ / 2);

      // DOWN
      digitalWrite(speakerOut, LOW);
      delayMicroseconds(tone_ / 2);

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delayMicroseconds(duration);  
    }                                
  }
}

void play()
{
  for (int i=0; i<MAX_COUNT; i++) 
      {
          tone_ = melody[i];
          beat = beats[i];

          duration = beat * tempo; // Set up timing

          song(); 
          // A pause between notes...
          delayMicroseconds(pause);
      }
}