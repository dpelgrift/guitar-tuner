#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>


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

// log10 freq values for octaves 3-7
double logFreqVals[60] =
                        {
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

int octaves[5] = {3,4,5,6,7};

String notes[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

void testDisplayNotes() {

    String notes[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

    display.setTextSize(4);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    
    for(int nIdx=0;nIdx<12;nIdx++) {
        display.setCursor(55,10);             // Start at top-left corner
        display.clearDisplay();

        display.println(notes[nIdx]);

        display.display();
        delay(2000);
    }
}

void testLines(){
    for(int i=0; i<display.width(); i+=3) {
        display.clearDisplay(); // Clear display buffer
        display.drawLine(i, 32, i, display.height()-1, SSD1306_WHITE);
        display.display(); // Update screen with each newly-drawn line
        delay(100);
    }
}



void testFFT(){
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
    /* Print the results of the sampling according to time */
    // Serial.println("Data:");
    // PrintVector(vReal, samples, SCL_TIME);
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    // Serial.println("Weighed data:");
    // PrintVector(vReal, samples, SCL_TIME);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    // Serial.println("Computed Real values:");
    // PrintVector(vReal, samples, SCL_INDEX);
    // Serial.println("Computed Imaginary values:");
    // PrintVector(vImag, samples, SCL_INDEX);
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    // Serial.println("Computed magnitudes:");
    // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    Serial.println(x*5.34, 6); //Print out what frequency is the most dominant.
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
    return x*5.34;
}


void renderFrame(double freq) {
    display.clearDisplay(); // Clear display buffer

    // Draw border
    display.drawLine(0, 0, display.width()-1, 0, SSD1306_WHITE);
    display.drawLine(display.width()-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
    display.drawLine(display.width()-1, display.height()-1, 0, display.height()-1, SSD1306_WHITE);
    display.drawLine(0, 0, 0, display.height()-1, SSD1306_WHITE);

    //
    display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    
    display.setCursor(40,5);

    display.println(freq,4);

    display.display();

}


void setup() {
    Serial.begin(9600);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();
    display.display();
}

void loop() {

    // testFFT();

    double freq = computeFFT();

    renderFrame(freq);

    delay(50);
}