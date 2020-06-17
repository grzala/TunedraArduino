/*
* Modified Arduino Frequency Detection
* by Nicole Grimwood
*
* For more information please visit: 
* https://www.instructables.com/id/Arduino-Guitar-Tuner/
* 
*
* This code is in the public domain.
*/

// --------------------------------------------------------------- NOTES AND TUNING -------------------------------------------------------------------------
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

double get_octave_multiplier(double freq) {
  double multiplier = 1.d;
  double last_octave_freq = 0;
  while (freq > FIRST_OCT_MAX_FREQ * multiplier) {
    multiplier *= 2.d;
    last_octave_freq = FIRST_OCT_MAX_FREQ * multiplier;
  }

  //Serial.println(last_octave_freq);

  return multiplier;
}

void getNoteByFreq(Note* note, double freq) {
  if (!isFreqLegal(freq)) {
    note->valid = false;
    return;
  }
  
  double multiplier = get_octave_multiplier(freq);
  //Serial.println(multiplier);

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

typedef enum {
  
} CharToInstr;

typedef DisplInstruction DI;

class Display {
  public:
    Display(int midPin, int upPin, int upRPin, int downRPin,
                        int downPin, int downLPin, int UpLPin, int sharpPin,
                        int rLED0, int gLED, int rLED1); 
    void clean();
    void cleanIndicator();
    void do_sth1();
    void do_sth2();
    void light(DI instruction, bool sharp);
    void light(unsigned int instruction);
    void lightSharp(bool light);
    void lightIndicator(int currentFreq, const Note* note);
    void write(DBN pin);
    void write(unsigned int pin);
    void displayNote(const Note* note, float frequency);
    void restIfTime();

  private:
    int pin_array[7];
    unsigned int currentlyDisplaying = 0;
    int sharpPin;
    bool currentSharpPinStatus = false;
    
    int indicatorBar[5];
    unsigned long time_at_last_display = 0;
    const unsigned int time_to_rest = 5000;
};

const double MAX_ANALOG = 255.d;
Display::Display(int midPin, int upPin, int upRPin, int downRPin,
                 int downPin, int downLPin, int UpLPin, int sharpPin,
                 int rLED0, int gLED, int rLED1) {
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
  this->indicatorBar[1] = gLED;
  this->indicatorBar[2] = rLED1;

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
}

void Display::cleanIndicator() {
  analogWrite(this->indicatorBar[0], 0);
  analogWrite(this->indicatorBar[1], 0);
  analogWrite(this->indicatorBar[2], 0);
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

// currentFreq must be beterrn min and max freq of note
void Display::lightIndicator(int currentFreq, const Note* note) {
  if (currentFreq < note->min_freq || currentFreq > note->max_freq) {
    return;
  }

  double max_dist_l = note->freq - note->min_freq;
  double max_dist_r = note->max_freq - note->freq;
  double max_dist_m = max_dist_r;

  double dist_l = currentFreq - note->min_freq;
  double dist_m = abs(currentFreq - note->freq);
  double dist_r = note->max_freq - currentFreq;

  int val_l = (int)((MAX_ANALOG * (max_dist_l - dist_l))/max_dist_l);
  int val_m = (int)((MAX_ANALOG * (max_dist_m - dist_m))/max_dist_m);
  int val_r = (int)((MAX_ANALOG * (max_dist_r - dist_r))/max_dist_r);

  val_l = max(val_l, 0);
  val_m = max(val_m, 0);
  val_r = max(val_r, 0);


  this->cleanIndicator();

  analogWrite(this->indicatorBar[0], val_l);
  analogWrite(this->indicatorBar[1], val_m);
  analogWrite(this->indicatorBar[2], val_r);

  
  Serial.println(dist_l);
  Serial.println(dist_m);
  Serial.println(dist_r);
  Serial.println();
  
}

void Display::displayNote(const Note* note, float frequency) {
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
  this->lightIndicator((int)frequency, note);
  this->time_at_last_display = millis();
}

void Display::restIfTime() {
  unsigned long currentTime = millis();
  if (currentTime - this->time_at_last_display > this->time_to_rest) {
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
  
  // (mid, up, upright, downright, down, leftdown, rightdown, sharp, red0, green, red1
  displ = new Display(3, 6, 7, 19, 5, 4, 2, 18, 8, 10, 12);
  
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
        //Serial.println("I AM HER2 ");
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        noMatch = 0;
      }
      else{//crossing midpoint but not match
        //Serial.println("I AM HER3 ");
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
    //Serial.println("clipping");
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
float long_last_frequencies[LONG_FREQ_AR_LEN];
int long_freq_ar_i = 0;
const float FREQ_MAX_DIFF = 0.15f;

// For normalizing small short deviations
const int SHORT_FREQ_AR_LEN = 60;
float short_last_frequencies[SHORT_FREQ_AR_LEN];
int short_freq_ar_i = 0;


float get_av(float* ar, int len) {
  float sum = 0;
  for (int i = 0; i < len; i++) {
    if (ar > 0) {
      sum += ar[i];
    }
  }

  return sum/(float)len;
}

int c = 0;
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

        getNoteByFreq(currentNote, short_average_freq);
        if (currentNote->valid) {
          Serial.print(frequency);
          Serial.print(" hz - maps to note: ");
          //Serial.print(getNoteName(note));
          Serial.print(currentNote->note);
          if (currentNote->sharp) Serial.print("#");
          Serial.println();
          displ->displayNote(currentNote, frequency);
        }

      }
    }
  }

  displ->restIfTime();

  if (c++ > 2000) c = 0;
  analogWrite(A1, c);
  
  delay(10);
    
}
// --------------------------------------------------------------- END MAIN -----------------------------------------------------------------------------
