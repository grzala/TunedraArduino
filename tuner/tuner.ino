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


// DISPLAY -----------------------------------------------------------------------------------------------------

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
                        int downPin, int downLPin, int UpLPin); 
    void clean();
    void do_sth1();
    void do_sth2();
    void light(DI instruction);
    void light(unsigned int instruction);
    void write(DBN pin);
    void write(unsigned int pin);

  private:
    int pin_array[7];
    unsigned int currentlyDisplaying = 0;
};

Display::Display(int midPin, int upPin, int upRPin, int downRPin,
                        int downPin, int downLPin, int UpLPin) {
  pinMode(midPin, OUTPUT);
  pinMode(upPin, OUTPUT);
  pinMode(upRPin, OUTPUT);
  pinMode(downRPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(downLPin, OUTPUT);
  pinMode(UpLPin, OUTPUT);
  
  this->pin_array[0] = midPin;
  this->pin_array[1] = upPin;
  this->pin_array[2] = upRPin;
  this->pin_array[3] = downRPin;
  this->pin_array[4] = downPin;
  this->pin_array[5] = downLPin;
  this->pin_array[6] = UpLPin;

  this->clean();
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

void Display::light(DI instruction) {
  this->light(static_cast<unsigned int>(instruction));
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

Display* displ;

// END DISPLAY -----------------------------------------------------------------------------------------------------


// NOTES -------------------------------------------------------------------------------------------------------
typedef struct {
  char note;
  bool sharp;
  float freq;
} Note;

String getNoteName(const Note& note) {
  String nam = (String)note.note;
  if (note.sharp) nam.concat("#");
  return nam;
}

const int MIN_FREQ = 15.892f;
const int MAX_FREQ = 508.565f;
const int NOTES_AMOUNT = 12;

typedef struct {
  int index;
  float max_freq;
  Note notes[12];
} Octave;

Octave octave0 = {
  0,
  31.385f,
  {
    {'C', false, 16.35f},
    {'C', true, 17.32f},
    {'D', false, 18.35f},
    {'D', true, 19.45f},
    {'E', false, 20.60f},
    {'F', false, 21.83f},
    {'F', true, 23.12f},
    {'G', false, 24.50f},
    {'G', true, 25.96f},
    {'A', false, 27.50f},
    {'A', true, 29.14f},
    {'B', false, 30.87f}
  }
};

Octave octave1 = {
  1,
  63.535f,
  {
    {'C', false, 32.70f},
    {'C', true, 34.65f},
    {'D', false, 36.71f},
    {'D', true, 38.89f},
    {'E', false, 41.20f},
    {'F', false, 43.65f},
    {'F', true, 46.25f},
    {'G', false, 49.00f},
    {'G', true, 51.91f},
    {'A', false, 55.00f},
    {'A', true, 58.27f},
    {'B', false, 61.74f}
  }
};

Octave octave2 = {
  2,
  127.14f,
  {
    {'C', false, 65.41f},
    {'C', true, 69.30f},
    {'D', false, 73.42f},
    {'D', true, 77.78f},
    {'E', false, 82.41f},
    {'F', false, 87.31f},
    {'F', true, 92.50f},
    {'G', false, 98.00f},
    {'G', true, 103.83f},
    {'A', false, 110.00f},
    {'A', true, 116.54f},
    {'B', false, 123.47f}
  }
};

Octave octave3 = {
  3,
  254.285f,
  {
    {'C', false, 130.81f},
    {'C', true, 138.59f},
    {'D', false, 146.83f},
    {'D', true, 155.56f},
    {'E', false, 164.81f},
    {'F', false, 174.61f},
    {'F', true, 185.00f},
    {'G', false, 196.00f},
    {'G', true, 207.65f},
    {'A', false, 220.00f},
    {'A', true, 233.08f},
    {'B', false, 246.94f}
  }
};

Octave octave4 = {
  4,
  508.565f,
  {
    {'C', false, 261.63f},
    {'C', true, 277.18f},
    {'D', false, 293.66f},
    {'D', true, 311.13f},
    {'E', false, 329.63f},
    {'F', false, 349.23f},
    {'F', true, 369.99f},
    {'G', false, 392.00f},
    {'G', true, 415.30f},
    {'A', false, 440.00f},
    {'A', true, 466.15f},
    {'B', false, 493.88f}
  }
};

// Sorted octaves array
Octave octaves[] = {
  octave0,
  octave1,
  octave2,
  octave3,
  octave4
};


// END FREQUENCIES --------------------------------------------------------------------------------------------------------------


void setup(){
  Serial.begin(115200);

  displ = new Display(3, 6, 7, 8, 5, 4, 2);
  
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

// PHYSICS ----------------------------------------------------------------------------------------------------

ISR(ADC_vect) {//when new ADC value ready
  //return; /////////////////////////////////// COMMENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  PORTB &= B11101111;//set pin 12 low
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
        PORTB |= B00010000;//set pin 12 high
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
        PORTB |= B00010000;//set pin 12 high
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

// END PHYSICS ----------------------------------------------------------------------------------------------------------


// TUNING ---------------------------------------------------------------------------------------------------------------

bool isFreqLegal(float freq) {
  return freq >= MIN_FREQ && freq < MAX_FREQ;
}

const Note& getNote(float freq) {
  int octave_i = 0;
  while (freq >= octaves[octave_i].max_freq) {
    octave_i++;
  }
  const Octave& octave = octaves[octave_i];
    
  float min_distance = MAX_FREQ;
  Note* closestNote;
  for (int note_i = 0; note_i < NOTES_AMOUNT; note_i++) {
    float distance = abs(freq - octave.notes[note_i].freq);
    if (distance < min_distance) {
      min_distance = distance;
      closestNote = &octave.notes[note_i];
    } else {
      break;
    }
  }
  
  return *closestNote;
}

// END TUNING -----------------------------------------------------------------------------------------------------------

const int FREQ_AR_LEN = 120;
float last_frequencies[FREQ_AR_LEN];
int freq_ar_i = 0;
const float FREQ_MAX_DIFF = 0.25f;

float get_freq_av() {
  float sum = 0;
  for (int i = 0; i < FREQ_AR_LEN; i++) {
    if (last_frequencies > 0) {
      sum += last_frequencies[i];
    }
  }

  return sum/(float)FREQ_AR_LEN;
}

void debug() {
  
  for (int octave_i = 0; octave_i < 5; octave_i++) {
    const Octave& octave = octaves[octave_i];
    for (int note_i = 0; note_i < NOTES_AMOUNT; note_i++) {
      const Note& note = octave.notes[note_i];
      Serial.println("Octave: " + (String)octave.index + " note: " + note.note + " freq: " + (String)note.freq);
    }
  }
}

void displayNote(char letter) {
  DI di = DI::A;
  switch(letter) {
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

  displ->light(di);
}

void loop(){
  checkClipping();

  if (checkMaxAmp>ampThreshold){
    frequency = 38462.0/float(period);//calculate frequency timer rate/period
    frequency+=1.f;
    if (isFreqLegal(frequency)) {
      last_frequencies[freq_ar_i++] = frequency;
      if (freq_ar_i >= FREQ_AR_LEN) freq_ar_i = 0;

      // Ignore noise and big swings
      float average_freq = get_freq_av();
      float diff = abs(average_freq - frequency);
      float max_diff = average_freq * FREQ_MAX_DIFF;
      if (diff < max_diff){
        const Note& note = getNote(frequency);
        Serial.print(frequency);
        Serial.print(" hz - maps to note: ");
        //Serial.print(getNoteName(note));
        Serial.print(note.note);
        if (note.sharp) Serial.print("#");
        Serial.println();

        displayNote(note.note);
      }
    }
  }
  delay(10);

  

  
}
