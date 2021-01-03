#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include <stdlib.h>



// Screen Params
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     7 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Microphone/FFT Params
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A5
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 7000; //Hz, must be less than 10000 due to ADC

const double freqCoeffFactor = 0.997; // Multiplication factor applied to FFT output to correct freq in Hz

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


// Notes/frequency params

#define NOTE_DIFF 0.02508 // Mean gap length between log10 frequencies

#define CORRECT_NOTE_DIFF 0.00125 // Mean gap length between log10 frequencies

#define NUM_LEVEL_BARS 5

#define LEVEL_BAR_WIDTH 10

#define VOLUME_THRES 650 // Threshold to update screen


// log10 freq values for octaves 3-7
double logFreqVals[72] =
                        {
                        1.815644149, 1.840733235, 1.86581438,
                        1.890867939, 1.915979914, 1.941063988,
                        1.966141733, 1.991226076, 2.016322854,
                        2.041392685, 2.066475014, 2.091561448,
                        2.116640946, 2.141731895, 2.166814799,
                        2.191897934, 2.216983559, 2.242069112,
                        2.267171728, 2.292256071, 2.317331935,
                        2.342422681, 2.367505009, 2.392591444,
                        2.417687541, 2.44276189, 2.467844794,
                        2.492941889, 2.51802673, 2.543111544,
                        2.568189986, 2.593286067, 2.618361931,
                        2.643452676, 2.668535005, 2.693621439,
                        2.718709237, 2.74379972, 2.768882185,
                        2.793964905, 2.819050138, 2.84414154,
                        2.869225851, 2.894310523, 2.919397155,
                        2.944482672, 2.969569659, 2.994655832,
                        3.019739233, 3.044825799, 3.06991218,
                        3.094998391, 3.120083428, 3.145168426,
                        3.170255847, 3.195340519, 3.220427151,
                        3.245512668, 3.270599655, 3.295683629,
                        3.320769228, 3.345855794, 3.370942176,
                        3.396028386, 3.421113424, 3.446199976,
                        3.471285842, 3.496370515, 3.521457147,
                        3.546542663, 3.571628486, 3.596714724
                        };

const int octaves[6] = {2,3,4,5,6,7};

const String notes[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

String noteToDisplay;

int octaveToDisplay;

double freqDiffToDisplay;


// Status LED pin
#define STATUS_LED 10


int getNearestElementIdx(double arr[], int n, double target) {
    int nearestIdx = 0;
    double nearestDiff;
    double tmpDiff;
    for (int aIdx = 0; aIdx<n; aIdx++){
        if (aIdx == 0) {
            nearestDiff = abs(arr[0]-target);
            continue;
        } 
        tmpDiff = abs(arr[aIdx]-target);

        if (tmpDiff < nearestDiff) {
            nearestIdx = aIdx;
            nearestDiff = tmpDiff;
        }
    }

    return nearestIdx;
}

void getNearestNoteAndOctave(double freq, String *note, int *octave, double *logFreqDiff){

    double logFreq = log10(freq);

    int nearestIdx = getNearestElementIdx(logFreqVals,72,logFreq);

    *logFreqDiff = logFreq - logFreqVals[nearestIdx];

    int noteIdx = nearestIdx % 12;

    int octaveIdx = floor(double(nearestIdx)/12.0);

    *note = notes[noteIdx];
    *octave = octaves[octaveIdx];

}

double computeFFT(){
    /*SAMPLING*/
    microseconds = micros();
    for(int i=0; i<samples; i++)
    {
        vReal[i] = analogRead(CHANNEL);
        vImag[i] = 0;
        while(micros() - microseconds < sampling_period_us){
        //empty loop
        }
        microseconds += sampling_period_us;
    }

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    return x*freqCoeffFactor;
}




void drawBlankFrame() {
    display.clearDisplay(); // Clear display buffer

    // Draw border
    display.drawLine(0, 0, display.width()-1, 0, SSD1306_WHITE);
    display.drawLine(display.width()-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
    display.drawLine(display.width()-1, display.height()-1, 0, display.height()-1, SSD1306_WHITE);
    display.drawLine(0, 0, 0, display.height()-1, SSD1306_WHITE);

    // Draw dividing line
    display.drawLine(0, 31, display.width()-1, 31, SSD1306_WHITE);

    // Draw intermediate vertical lines
    display.drawLine(54, 31, 54, display.height()-1, SSD1306_WHITE);
    display.drawLine(73, 31, 73, display.height()-1, SSD1306_WHITE);

}

void drawFreqBar(double freqDiff) {
    // check if close enough to note for center bar
    if (abs(freqDiff) <= CORRECT_NOTE_DIFF) {
        display.fillRect(56,33,16,30,SSD1306_WHITE);
    } else {
        int barIdx = floor(abs(freqDiff)/(NOTE_DIFF/1.95)*5);

        int rectXCoord;
        if (freqDiff > 0) { // If current freq greater than nearest note
            rectXCoord = 75+(barIdx*LEVEL_BAR_WIDTH);
        } else { // If current freq lower than nearest note
            rectXCoord = 43-(barIdx*LEVEL_BAR_WIDTH);
        }
        display.fillRect(rectXCoord,33,10,30,SSD1306_WHITE);
    }


}

void drawFrame(double freq,String note,int octave,double freqDiff) {
    drawBlankFrame();

    // Draw Note, octave, & frequency
    display.setTextColor(SSD1306_WHITE);        // Draw white text

    // Draw Note
    display.setTextSize(3);             
    display.setCursor(20,5);
    display.print(note);

    // Draw Octave
    display.setTextSize(1);             
    display.setCursor(60,20);
    display.print(octave);

    // Draw Frequency
    display.setTextSize(1);             
    display.setCursor(80,10);
    display.print(int(freq));
    display.print(F(" Hz"));

    drawFreqBar(freqDiff);

}

double computeMean(double arr[], int n){

    double sum = 0;

    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum/double(n);
}


void setup() {

    // Switch on status LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    sampling_period_us = round(1000000*(1.0/samplingFrequency));
    Serial.begin(9600);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        // Blink status LED to indicate error
        while (1){
            digitalWrite(STATUS_LED, HIGH);
            delay(500);               
            digitalWrite(STATUS_LED, LOW);
            delay(500);               
        }
    }

    // Show initial blank display
    drawBlankFrame();
    display.display();
    delay(2000); // Pause for 2 seconds


    // // Debug 
    // int nearestIdx;
    // double testVals[10] = {1.9, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9};
    // for (int i = 0; i < 10; i++) {
    //     nearestIdx = getNearestElementIdx(logFreqVals,60,testVals[i]);
    //     Serial.print("Val: ");
    //     Serial.print(testVals[i]);
    //     Serial.print(" | Idx: ");
    //     Serial.print(nearestIdx);
    //     Serial.println("");
    // }

    

}

void loop() {

    double freq = computeFFT();

    

    double meanVal = computeMean(vReal,samples);

    if (meanVal > VOLUME_THRES) {
        getNearestNoteAndOctave(freq, &noteToDisplay, &octaveToDisplay, &freqDiffToDisplay);
        drawFrame(freq,noteToDisplay,octaveToDisplay,freqDiffToDisplay);
    } else {
        drawBlankFrame();
    }
    display.display();
    // Serial.print("Mean vReal: ");
    // Serial.print(meanVal);
    // Serial.println("");


    

    // Serial.print("Freq: ");
    // Serial.print(freq);
    // Serial.print(" | Log Freq: ");
    // Serial.print(log10(freq));
    // Serial.print(" | Note: ");
    // Serial.print(noteToDisplay);
    // Serial.print(" | Octave: ");
    // Serial.print(octaveToDisplay);
    // Serial.print(" | Nearest Idx: ");
    // Serial.print(nearestIdx);
    // Serial.println("");

    


    delay(10);
}