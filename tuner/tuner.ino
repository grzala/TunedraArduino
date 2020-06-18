/*
* This code uses Arduino Frequency Detection algorithm
* by Nicole Grimwood
*
* For more information please visit: 
* https://www.instructables.com/id/Arduino-Guitar-Tuner/
* 
*
* This code is in the public domain.
*/

// --------------------------------------------------------------- NOTES AND TUNING -------------------------------------------------------------------------
// constants for note detection
const double MIN_FREQ = 15.8927255d;
const double FIRST_OCT_MAX_FREQ = 31.7854510d;
const double MAX_FREQ = 508.565d;
const int NOTES_IN_OCTAVE = 12;

const double firstOctaveFreqs[] = {
  16.3515978d, 
  17.3239144d, 
  18.3540480d, 
  19.4554365d, 
  20.6017223d, 
  21.8267644d, 
  23.1246514d, 
  24.4997147d, 
  25.9565435d, 
  27.5000000d, 
  29.1352351d, 
  30.8677063d
 };
 
const char noteNames[] = {'C', 'C', 'D', 'D', 'E', 'F', 'F', 'G', 'G', 'A', 'A', 'B'};
const bool noteSharps[] = {false, true, false, true, false, false, true, false, true, false, true, false};

// Namespace due to compiling problems
namespace {
  
typedef struct {
  char note;
  bool sharp;
  double freq;
  double min_freq;
  double max_freq;
  bool valid = false;
} Note;

bool isFreqLegal(double freq) {
  return freq >= MIN_FREQ && freq < MAX_FREQ;
}

// Determine in which octave lies the frequency and get a multiplier required to work with that octave
double get_octave_multiplier(double freq) {
  double multiplier = 1.d;
  double last_octave_freq = 0;
  while (freq > FIRST_OCT_MAX_FREQ * multiplier) {
    multiplier *= 2.d;
    last_octave_freq = FIRST_OCT_MAX_FREQ * multiplier;
  }

  return multiplier;
}

void getNoteByFreq(Note* note, double freq) {
  if (!isFreqLegal(freq)) {
    note->valid = false;
    return;
  }
  
  double multiplier = get_octave_multiplier(freq);

  // Find closest note in O(n) time
  double min_distance = MAX_FREQ;
  int closest_i = -1;
  for (int note_i = 0; note_i < NOTES_IN_OCTAVE; note_i++) {
    double distance = abs(freq - (firstOctaveFreqs[note_i]*multiplier));
    if (distance < min_distance) {
      min_distance = distance;
      closest_i = note_i;
    } else {
      break;
    }
  }

  // Fill note struct
  note->note = noteNames[closest_i];
  note->sharp = noteSharps[closest_i];
  note->freq = firstOctaveFreqs[closest_i]*multiplier;
  note->valid = true;

  if (closest_i == NOTES_IN_OCTAVE-1) {
    note->max_freq = FIRST_OCT_MAX_FREQ*multiplier;
  } else {
    note->max_freq = note->freq + (((firstOctaveFreqs[closest_i+1]*multiplier) - note->freq)/2.d);
  }

  if (closest_i == 0) {
    note->min_freq = MIN_FREQ*multiplier;
  } else {
    note->min_freq = note->freq - ((note->freq - (firstOctaveFreqs[closest_i-1]*multiplier))/2.d);
  }
}

}

// --------------------------------------------------------------- END NOTES AND TUNING ---------------------------------------------------------------------

// --------------------------------------------------------------- DISPLAY ----------------------------------------------------------------------------------
// Map display segments to int
typedef enum {
  mid = 0,
  up = 1,
  upR = 2,
  downR = 3,
  down = 4,
  downL = 5,
  upL = 6,
} DisplBarName;
typedef DisplBarName DBN;

// Instructions for displaying letters
typedef enum {
  A = 0b1101111,
  B = 0b1111001,
  C = 0b1110010,
  D = 0b0111101,
  E = 0b1110011,
  F = 0b1100011,
  G = 0b1111010,
  H = 0b1101101,
  J = 0b0011100,
  I = 0b0001100,
  U = 0b1111100,
  S = 0b1011011,
  Z = 0b0110111, 
} DisplInstruction;
typedef DisplInstruction DI;

// Display class handles LED 7 segment display, LED indicating sharp note and a LED bar indicating deviation
class Display {
  public:
    static const int LEDFunctions_Len = 3; // 3 linear functions will help determine LED brightness
  
    Display(int midPin, int upPin, int upRPin, int downRPin,
                        int downPin, int downLPin, int UpLPin, int sharpPin,
                        int rLED0, int rLED1, int gLED, int rLED2, int rLED3); 
    void clean();
    void cleanIndicator();
    void do_sth1();
    void do_sth2();
    void light(DI instruction, bool sharp);
    void light(unsigned int instruction);
    void lightSharp(bool light);
    void lightIndicator(double currentFreq, const Note* note);
    void write(DBN pin);
    void write(unsigned int pin);
    void displayNote(const Note* note, double frequency);
    void resetIfTime();

  private:
    int pin_array[7];
    unsigned int currentlyDisplaying = 0;
    int sharpPin;
    bool currentSharpPinStatus = false;
    int indicatorBar[5];
    
    unsigned long time_at_last_display = 0;
    const unsigned int time_to_reset = 5000;

    
    // Adjust those to alter light. LEDs are lighted according to three linear functions intersecting
    const static double MAX_ANALOG = 200.d; // LEDS do not need to go any higher
    
    static const double xBoundFactors[LEDFunctions_Len] = { 0.035d, 0.15d, 0.35d };
    static const double yBoundFactors[LEDFunctions_Len] = { 0.4, 0.05, 0.d };
    static const double yBounds[LEDFunctions_Len] = { 
      yBoundFactors[0] * MAX_ANALOG,
      yBoundFactors[1] * MAX_ANALOG, 
      yBoundFactors[2] * MAX_ANALOG
    }; 
    
    typedef struct {
      double max_distance;
      double As[Display::LEDFunctions_Len]; 
      double Bs[Display::LEDFunctions_Len];
      double xBounds[Display::LEDFunctions_Len];
    } LEDFunctionCache;
    LEDFunctionCache ledFCache {-1};

    
    int getIndicatorValByDistance(double distance, double max_distance);
    void rebuildCache(double max_distance);
    void printCacheInfo();
};
// END CLASS DISPLAY DECLARATION

// IMPLEMENT CLASS DISPLAY
Display::Display(int midPin, int upPin, int upRPin, int downRPin,
                 int downPin, int downLPin, int UpLPin, int sharpPin,
                 int rLED0, int rLED1, int gLED, int rLED2, int rLED3) {
  pinMode(midPin, OUTPUT);
  pinMode(upPin, OUTPUT);
  pinMode(upRPin, OUTPUT);
  pinMode(downRPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(downLPin, OUTPUT);
  pinMode(UpLPin, OUTPUT);
  
  pinMode(sharpPin, OUTPUT);
  
  pinMode(rLED0, OUTPUT);
  pinMode(gLED, OUTPUT);
  pinMode(rLED1, OUTPUT);
  
  this->pin_array[0] = midPin;
  this->pin_array[1] = upPin;
  this->pin_array[2] = upRPin;
  this->pin_array[3] = downRPin;
  this->pin_array[4] = downPin;
  this->pin_array[5] = downLPin;
  this->pin_array[6] = UpLPin;
  
  this->sharpPin = sharpPin;

  this->indicatorBar[0] = rLED0;
  this->indicatorBar[1] = rLED1;
  this->indicatorBar[2] = gLED;
  this->indicatorBar[3] = rLED2;
  this->indicatorBar[4] = rLED3;

  this->clean();
  this->lightSharp(false);
  this->cleanIndicator();
}

void Display::clean() {
  digitalWrite(this->pin_array[0], LOW);
  digitalWrite(this->pin_array[1], LOW);
  digitalWrite(this->pin_array[2], LOW);
  digitalWrite(this->pin_array[3], LOW);
  digitalWrite(this->pin_array[4], LOW);
  digitalWrite(this->pin_array[5], LOW);
  digitalWrite(this->pin_array[6], LOW);
  this->currentlyDisplaying = 0;
}

void Display::cleanIndicator() {
  analogWrite(this->indicatorBar[0], 0);
  analogWrite(this->indicatorBar[1], 0);
  analogWrite(this->indicatorBar[2], 0);
  analogWrite(this->indicatorBar[3], 0);
  analogWrite(this->indicatorBar[4], 0);
}

void Display::do_sth1() {
  this->clean();
  
  digitalWrite(this->pin_array[DBN::mid], HIGH);
  digitalWrite(this->pin_array[DBN::up], HIGH);
  digitalWrite(this->pin_array[DBN::down], HIGH);
}

void Display::do_sth2() {
  this->clean();
  
  digitalWrite(this->pin_array[DBN::upR], HIGH);
  digitalWrite(this->pin_array[DBN::downR], HIGH);
  digitalWrite(this->pin_array[DBN::upL], HIGH);
  digitalWrite(this->pin_array[DBN::downL], HIGH);
}

void Display::write(DBN pin) {
  this->write(static_cast<unsigned int>(pin));
}

void Display::write(unsigned int pin) {
  digitalWrite(this->pin_array[pin], HIGH);
}

void Display::light(DI instruction, bool sharp) {
  this->light(static_cast<unsigned int>(instruction), sharp);
}

void Display::light(unsigned int instruction) {
  if (instruction == this->currentlyDisplaying) return;
  
  this->clean();

  if (instruction & (1 << DBN::mid)) {
    this->write(DBN::mid);
  }
  if (instruction & (1 << DBN::up)) {
    this->write(DBN::up);
  }
  if (instruction & (1 << DBN::upR)) {
    this->write(DBN::upR);
  }
  if (instruction & (1 << DBN::downR)) {
    this->write(DBN::downR);
  }
  if (instruction & (1 << DBN::down)) {
    this->write(DBN::down);
  }
  if (instruction & (1 << DBN::downL)) {
    this->write(DBN::downL);
  }
  if (instruction & (1 << DBN::upL)) {
    this->write(DBN::upL);
  }
  
  this->currentlyDisplaying = instruction;
}

void Display::lightSharp(bool light) {
  if (light) {
    digitalWrite (this->sharpPin, HIGH);
  } else {
    digitalWrite (this->sharpPin, LOW);
  }
  this->currentSharpPinStatus = light;
}


void Display::printCacheInfo() {
  Serial.print("f(x) = ");
  Serial.print(ledFCache.As[0]);
  Serial.print("x + ");
  Serial.print(ledFCache.Bs[0]);
  Serial.println();
  Serial.print("f(x) = ");
  Serial.print(ledFCache.As[1]);
  Serial.print("x + ");
  Serial.print(ledFCache.Bs[1]);
  Serial.println();
  Serial.print("f(x) = ");
  Serial.print(ledFCache.As[2]);
  Serial.print("x + ");
  Serial.print(ledFCache.Bs[2]);
  Serial.println();
}

// build a and b coefficients for linear functions
void Display::rebuildCache(double max_distance) {
  ledFCache.xBounds[0] = xBoundFactors[0] * max_distance;
  ledFCache.xBounds[1] = xBoundFactors[1] * max_distance;
  ledFCache.xBounds[2] = xBoundFactors[2] * max_distance;

  ledFCache.As[0] = (yBounds[0] - MAX_ANALOG) / (ledFCache.xBounds[0]);
  ledFCache.Bs[0] = MAX_ANALOG;
  
  ledFCache.As[1] = (yBounds[1] - yBounds[0]) / (ledFCache.xBounds[1] - ledFCache.xBounds[0]);
  ledFCache.Bs[1] = yBounds[1] - (ledFCache.As[1] * ledFCache.xBounds[1]);
  
  ledFCache.As[2] = (yBounds[2] - yBounds[1]) / (ledFCache.xBounds[2] - ledFCache.xBounds[1]);
  ledFCache.Bs[2] = yBounds[2] - (ledFCache.As[2] * ledFCache.xBounds[2]);
}

int Display::getIndicatorValByDistance(double distance, double max_distance) {
  this->rebuildCache(max_distance); // find new functions
  
  if (distance < 0) return MAX_ANALOG; // distance should never be negative - in case it is, return max
  if (distance > ledFCache.xBounds[2]) return 0; // always no light if distance too high

  // Find which function to use
  int i = 0;
  for(; i < LEDFunctions_Len; i++) 
    if (distance <= ledFCache.xBounds[i]) 
      break;
      
  int val = (int)((ledFCache.As[i] * distance) + ledFCache.Bs[i]);
  return val;
}

// currentFreq must be beterrn min and max freq of note
void Display::lightIndicator(double currentFreq, const Note* note) {
  if (currentFreq < note->min_freq || currentFreq > note->max_freq) {
    return;
  }

  double max_dist = (note->max_freq - note->min_freq);

  double bound_1 = note->freq - (note->freq  - note->min_freq)/2.d;
  double bound_3 = note->freq + (note->max_freq - note->freq)/2.d;
  
  double dists[] = {
    min(max_dist, currentFreq - note->min_freq),
    min(max_dist, abs(currentFreq - bound_1)),
    min(max_dist, abs(currentFreq - note->freq)),
    min(max_dist, abs(bound_3 - currentFreq)),
    min(max_dist, note->max_freq - currentFreq)
  };
  
  for (int i = 0; i < 5; i++) {
    int val = this->getIndicatorValByDistance(dists[i], max_dist);
    analogWrite(this->indicatorBar[i], val);
  }  
}

void Display::displayNote(const Note* note, double frequency) {
  DI di = DI::A;
  switch(note->note) {
    case 'A':
      di = DI::A;
      break;
    case 'B':
      di = DI::B;
      break;
    case 'C':
      di = DI::C;
      break;
    case 'D':
      di = DI::D;
      break;
    case 'E':
      di = DI::E;
      break;
    case 'F':
      di = DI::F;
      break;
    case 'G':
      di = DI::G;
      break;
  }
  
  this->light(di);
  this->lightSharp(note->sharp);
  this->lightIndicator(frequency, note);
  this->time_at_last_display = millis();
}

void Display::resetIfTime() {
  unsigned long currentTime = millis();
  if (currentTime - this->time_at_last_display > this->time_to_reset) {
    this->clean();
    this->lightSharp(false);
    this->cleanIndicator();
  }
}

// --------------------------------------------------------------- END DISPLAY  ------------------------------------------------------------------------------


// --------------------------------------------------------------- SETUP ---------------------------------------------------------------------------------


Note* currentNote = NULL;
Display* displ = NULL;
void setup(){
  Serial.begin(115200);
  
  currentNote = new Note;
  
  // (mid, up, upright, downright, down, leftdown, rightdown, sharp, red0, red1, green, red2, red3)
  displ = new Display(3, 19, 18, 17, 7, 4, 2, 16, 6, 9, 5, 10, 11);
  
  cli();//diable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  sei();//enable interrupts
}
// --------------------------------------------------------------- END SETUP ---------------------------------------------------------------------------------


// --------------------------------------------------------------- PHYSICS ---------------------------------------------------------------------------------
//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally
int timer[10];//storage for timing of events
int slope[10];//storage for slope of events
unsigned int totalTimer;//used to calculate period
unsigned int period;//storage for period of wave
byte index = 0;//current storage index
float frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for decided whether you have a match
byte noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 3;//slope tolerance- adjust this if you need
int timerTol = 10;//timer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 14;//raise if you have a very noisy signal

const int MID_POINT = 127; //2.5V


ISR(ADC_vect) {//when new ADC value ready
  //return; /////////////////////////////////// COMMENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0

//  Serial.print("oldData: ");
//  Serial.print(prevData);
//  Serial.print("newData: ");
//  Serial.println(newData);
  
  if (prevData < MID_POINT && newData >= MID_POINT){//if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope-maxSlope)<slopeTol){//if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){//new max slope just reset
        //Serial.println("I AM HER1 ");
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0]-timer[index])<timerTol && abs(slope[0]-newSlope)<slopeTol){//if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i=0;i<index;i++){
          totalTimer+=timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        noMatch = 0;
      }
      else{//crossing midpoint but not match
        index++;//increment index
        if (index > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope){//if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else{//slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch>9){
        reset();
      }
    }
  }
  
  if (newData == 0 || newData == 1023){//if clipping
    clipping = 1;//currently clipping
  }
  
  time++;//increment timer at rate of 38.5kHz
  
  ampTimer++;//increment amplitude timer
  if (abs(127-ADCH)>maxAmp){
    maxAmp = abs(127-ADCH);
  }
  if (ampTimer==1000){
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
  
}

void reset(){//clean out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}


void checkClipping(){//manage clipping indication
  if (clipping){//if currently clipping
    clipping = 0;
  }
}
// --------------------------------------------------------------- END PHYSICS -----------------------------------------------------------------------------

// --------------------------------------------------------------- MAIN ---------------------------------------------------------------------------------
// For normalizing huge and short deviations
const int LONG_FREQ_AR_LEN = 120;
double long_last_frequencies[LONG_FREQ_AR_LEN];
int long_freq_ar_i = 0;
const double FREQ_MAX_DIFF = 0.15f;

// For normalizing small short deviations
const int SHORT_FREQ_AR_LEN = 20;
double short_last_frequencies[SHORT_FREQ_AR_LEN];
int short_freq_ar_i = 0;

// Get average from double array
double get_av(double* ar, int len) {
  double sum = 0;
  for (int i = 0; i < len; i++) {
    if (ar > 0) {
      sum += ar[i];
    }
  }

  return sum/(double)len;
}

// Print freq + note info
void printFreqNote(double frequency, const Note* note) {
    Serial.print(frequency);
    Serial.print(" hz - maps to note: ");
    Serial.print(currentNote->note);
    if (currentNote->sharp) Serial.print("#");
    Serial.println();
}

void loop(){
  checkClipping();

  if (checkMaxAmp>ampThreshold){
    frequency = 38462.0/float(period);//calculate frequency timer rate/period
    if (isFreqLegal(frequency)) {

      // Ignore noise and big swings
      long_last_frequencies[long_freq_ar_i++] = frequency;
      if (long_freq_ar_i >= LONG_FREQ_AR_LEN) long_freq_ar_i = 0;
      float long_average_freq = get_av(long_last_frequencies, LONG_FREQ_AR_LEN);
      float diff = abs(long_average_freq - frequency);
      float max_diff = long_average_freq * FREQ_MAX_DIFF;

      if (diff < max_diff){
        // get average freq
        short_last_frequencies[short_freq_ar_i++] = frequency;
        if (short_freq_ar_i >= SHORT_FREQ_AR_LEN) short_freq_ar_i = 0;
        float short_average_freq = get_av(short_last_frequencies, SHORT_FREQ_AR_LEN);

        getNoteByFreq(currentNote, short_average_freq); // RECOGNIZE NOTE
        if (currentNote->valid) {
          //printFreq(short_average_freq, currentNote);
          displ->displayNote(currentNote, short_average_freq); // DISPLAY NOTE
        }

      }
    }
  }

  displ->resetIfTime(); // Clean display if inactive
  
  delay(10); // Dont' overwork yourself bro
    
}
// --------------------------------------------------------------- END MAIN -----------------------------------------------------------------------------
