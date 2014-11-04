#define FILTERTAPS 30 
#define MAXVAL 1023
#define MAXVALREL 256
#define MINVAL 0
#define AVGNUM 30
#define WAKEUPSAMP 2
#define BUFFER 40
#define FADEGAP 1
#define TIMECHANGEMIL 5000
#define FREQ 8000
#define SENDSERIAL 0

template<int filterTaps>
class FIR {
      public:
            //construct without coefs
            FIR() {
                  k = 0; //initialize so that we start to read at index 0
                  for (int i=0; i<filterTaps; i++) {      
                        values[i] = 0; // to have a nice start up, fill the array with 0's
                  }
                  //TODO calculate default gain?
                  //TODO calculate default coefs?
            }
            //construct with coefs
            FIR(float newGain, float *newCoefs) {
                  k = 0; //initialize so that we start to read at index 0
                  setGain(newGain);
                  for (int i=0; i<filterTaps; i++) {      
                        values[i] = 0; // to have a nice start up, fill the array with 0's
                  }
                  setCoefficients(newCoefs);
            }
            
            void setGain(float newGain) {gain = newGain;}

            void setCoefficients(float *newCoefs) {
                  for (int i=0; i<filterTaps; i++) {      
                        coef[i] = newCoefs[i];
                  }
            }
            //set coefficient at specified index
            void setCoefficient(int idx, float newCoef) { coef[idx] = newCoef; }
            
            float process(float in) {
                  float out = 0;                        // out is the return variable. It is set to 0 every time we call the filter!

                  values[k] = in;                        // store the input of the routine (contents of the 'in' variable) in the array at the current pointer position

                  for (int i=0; i<filterTaps; i++) {                              // we step through the array
                        out += coef[i] * values[(i + k) % filterTaps];      // ... and add and multiply each value to accumulate the output
                                                                                                //  (i + k) % filterTaps creates a cyclic way of getting through the array
                  }
                  
                  out /= gain;                        // We need to scale the output (unless the coefficients provide unity gain in the passband)

                  k = (k+1) % filterTaps;            // k is increased and wraps around the filterTaps, so next time we will overwrite the oldest saved sample in the array

                  return out;                              // we send the output value back to whoever called the routine
            }
            
      private:
            float values[filterTaps];
            
            float coef[filterTaps];
            
            //declare gain coefficient to scale the output back to normal
            float gain; // set to 1 and input unity to see what this needs to be
            
            int k; // k stores the index of the current array read to create a circular memory through the array
};


class RGBControl {
     public:
          RGBControl() {  
             ledPin=0;
             micVal=0;
             DCVal=-1;
             brightness=0;
             LastBrightness=0;
             micMax=0;
             micMin=255;//1023;
             counter=0;
             SumDCVal=0;
             UpFade = 0;
             DownFade = 0;
          }
          RGBControl operator=( RGBControl C2)
          {
            ledPin = C2.ledPin; 
            brightness = C2.brightness;
            LastBrightness = C2.LastBrightness;
            micVal = C2.micVal;
            DCVal = C2.DCVal;
            micMax = C2.micMax;
            micMin = C2.micMin;
            counter = C2.counter;
            SumDCVal = C2.SumDCVal;
            UpFade = C2.UpFade;
            DownFade = C2.DownFade;
          }
          void SetZero() { brightness=0; }
          void SetLedPin(int Num) { ledPin=Num; }
          void SetDCVal(int Num) { DCVal=Num; }
          void SetFade(int Up, int Down) { UpFade=Up; DownFade=Down;}
          void SetMinMax(int Min, int Max) { micMin=Min; micMax=Max;}
          void micVal2Brightness() {
            micVal = abs(micVal-DCVal); 
            //micVal = min(micVal+DCALLVal,MAXVAL);
            micMin=min(micVal,micMin);
            micMax=max(micVal,micMax);
            brightness = int((double(micVal-micMin)/double(micMax-micMin))*MAXVALREL)-1;
            brightness = min(max(brightness-20,0),MAXVALREL-1);
          }
          void WriteBright()
          {
            int counter=0;
             while ( abs(LastBrightness - brightness)> FADEGAP/2)
             {
               counter++;
               if (LastBrightness>brightness) {
                 LastBrightness = LastBrightness - FADEGAP;
                 if (counter>=DownFade) break;
               }
               else
              { 
                 LastBrightness = LastBrightness + FADEGAP;     
                 if (counter>=UpFade) break;      
              }
              analogWrite(ledPin, LastBrightness);
             }
          }
          int CalcDC() {
            if (DCVal==-1)
            {
              if (counter>=WAKEUPSAMP) SumDCVal=SumDCVal+micVal;
              counter++;
              if (counter==AVGNUM+WAKEUPSAMP )
              {
                DCVal=SumDCVal/AVGNUM; 
                   if (SENDSERIAL) { 
                        Serial.println(DCVal);
                      }
                return 1;
              }

              return 0;
            }
            else
            {
               return 1; 
            }
          }          
         int ledPin; 
         int brightness; 
         int LastBrightness;         
         int micVal;
      int micMax;
      int micMin;         
    private:
      int DCVal;
      int counter;
      int SumDCVal;
      int UpFade;
      int DownFade;
};

long TimeTMP1=0;
long TimeTMP2=0;
long timeTMPDT=0;

long timeChange=0;

long Time1=0;
long Time2=0;
long timeDT=0;
int mic_NoFilt[BUFFER]={0};
int micPin=0;
volatile int AutoPin=12;
volatile bool AutoMode=0;
RGBControl RCtrl,GCtrl,BCtrl;
FIR<FILTERTAPS> firR,firG,firB;


void setup()  { 
  RCtrl.SetFade(50,20);
  BCtrl.SetFade(50,7);
  GCtrl.SetFade(50,2);   
 // RCtrl.SetMinMax(0,100);
  BCtrl.SetMinMax(0,20);
  GCtrl.SetMinMax(0,10);   
  RCtrl.SetLedPin(5);
  GCtrl.SetLedPin(9);
  BCtrl.SetLedPin(3);  
  pinMode(RCtrl.ledPin, OUTPUT);
  pinMode(GCtrl.ledPin, OUTPUT);
  pinMode(BCtrl.ledPin, OUTPUT);  

 if (SENDSERIAL) {
 Serial.begin(115200);
 }
float Rcoef[FILTERTAPS] = {0.021987,0.024035,0.026037,0.027974,0.029828,0.03158,0.033212,0.034709,0.036054,0.037234,0.038238,0.039053,0.039672,0.040089,0.040298,0.040298,0.040089,0.039672,0.039053,0.038238,0.037234,0.036054,0.034709,0.033212,0.03158,0.029828,0.027974,0.026037,0.024035,0.021987};
//{0.030352,0.03572,0.04093,0.045848,0.050345,0.054301,0.057606,0.06017,0.061921,0.062808,0.062808,0.061921,0.06017,0.057606,0.054301,0.050345,0.045848,0.04093,0.03572,0.030352};
//{0.0070018,0.0094086,0.016214,0.026914,0.040488,0.055516,0.070343,0.083292,0.092868,0.097955,0.097955,0.092868,0.083292,0.070343,0.055516,0.040488,0.026914,0.016214,0.0094086,0.0070018};
float Bcoef[FILTERTAPS]  ={-0.011762,-0.038701,0.0050889,0.056559,0.015417,-0.034795,-0.0098747,-0.011244,-0.051569,0.030127,0.14934,0.018955,-0.22001,-0.12084,0.20886,0.20886,-0.12084,-0.22001,0.018955,0.14934,0.030127,-0.051569,-0.011244,-0.0098747,-0.034795,0.015417,0.056559,0.0050889,-0.038701,-0.011762};
//{-0.0059583,0.010091,-0.011563,0.0090001,0.080684,0.06634,-0.085756,-0.18041,-0.040071,0.18015,0.18015,-0.040071,-0.18041,-0.085756,0.06634,0.080684,0.0090001,-0.011563,0.010091,-0.0059583};
//{0.0039218,0.0041651,-0.0017636,-0.024773,-0.064743,-0.096112,-0.078892,0.0056742,0.12672,0.2157,0.2157,0.12672,0.0056742,-0.078892,-0.096112,-0.064743,-0.024773,-0.0017636,0.0041651,0.0039218};
float Gcoef[FILTERTAPS] ={0.014581,0.015791,-0.017186,-0.018813,0.020738,0.023056,-0.025904,-0.029495,0.034171,0.040525,-0.049674,-0.064014,0.089775,0.1498,-0.44965,0.44965,-0.1498,-0.089775,0.064014,0.049674,-0.040525,-0.034171,0.029495,0.025904,-0.023056,-0.020738,0.018813,0.017186,-0.015791,-0.014581};
//{-0.0063113,0.030439,0.034878,-0.0095334,-0.057106,-0.039805,0.051455,0.12769,0.042445,-0.54343,0.54343,-0.042445,-0.12769,-0.051455,0.039805,0.057106,0.0095334,-0.034878,-0.030439,0.0063113};
//{0.0012888,-0.0014432,-0.007129,-0.0119,-0.0030922,0.027927,0.065647,0.063596,-0.04974,-0.57121,0.57121,0.04974,-0.063596,-0.065647,-0.027927,0.0030922,0.0119,0.007129,0.0014432,-0.0012888};
      firR.setCoefficients(Rcoef);
      firR.setGain(1);  
    firB.setCoefficients(Bcoef);
      firB.setGain( 1.9663);           
      firG.setCoefficients(Gcoef);    
    firG.setGain(2.0864);   
 //     BCtrl.SetDCVal(0);   

  
  //set up continuous sampling of analog pin 0 (you don't need to understand this part, just know how to use it in the loop())
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  //if you want to add other things to setup(), do it here

   pinMode(2, INPUT); 
   pinMode(AutoPin, OUTPUT); 
//attachInterrupt(1, ChangeMode, RISING);
attachInterrupt(0, ChangeMode, RISING);

} 

void loop()  { 
  
// timeChange = millis();
// if ( (timeChange % TIMECHANGEMIL) > (TIMECHANGEMIL - 50)  )
// {
//  firTMP = firR;
//  TMPCtrl = RCtrl;
//  firR = firB;
//  RCtrl = BCtrl;  
//  firB = firG;
//  BCtrl = GCtrl;    
//  firG = firTMP;
//  GCtrl = TMPCtrl;   
// }
  
    if (AutoMode)
  //if (digitalRead(2)==HIGH)
  {
    RCtrl.SetZero();
    GCtrl.SetZero();
    BCtrl.SetZero();
    RCtrl.WriteBright();
    BCtrl.WriteBright();
    GCtrl.WriteBright();     
   if (SENDSERIAL) {  
    Serial.println("Auto!");
   }
    delay(100);
    
  }
  else
  {

  
 TimeTMP2=micros();
 timeTMPDT=TimeTMP2-TimeTMP1;
 TimeTMP1=micros();
  
  int Buff=0;
 
  while (Buff<BUFFER/2) {
      Time2= micros();
      if (  Time2 - Time1 >= 56) {
          Time2= micros();
          mic_NoFilt[Buff] = ADCH;//analogRead(micPin);
          timeDT = Time2 - Time1;
          Time1 = micros(); 
          Buff++;
      }

  }
  
   for ( Buff=0;Buff<BUFFER/2;Buff++){ 
  RCtrl.micVal = int(firR.process(float(mic_NoFilt[Buff])));
  GCtrl.micVal = int(firG.process(float(mic_NoFilt[Buff])));
  BCtrl.micVal = int(firB.process(float(mic_NoFilt[Buff]))); 
   }
    
 
  if ( RCtrl.CalcDC() && GCtrl.CalcDC() && BCtrl.CalcDC()) {
     if (SENDSERIAL) {
        Serial.print("  TIME: ");
        Serial.print(timeDT);
        Serial.print(",");        
        Serial.print(timeTMPDT);        
            Serial.print("  NoFiltVal: ");
          Serial.print(mic_NoFilt[BUFFER/2-1]);
          Serial.print("  Val:  R = ");
          Serial.print(RCtrl.micVal);
          Serial.print("  G = ");
          Serial.print(GCtrl.micVal);
          Serial.print("  B = ");
          Serial.print(BCtrl.micVal);
     }
          RCtrl.micVal2Brightness();
          GCtrl.micVal2Brightness();
          BCtrl.micVal2Brightness();  
           if (SENDSERIAL) {
          Serial.print("  MinMax:  R = ");
          Serial.print(RCtrl.micMin); Serial.print(","); Serial.print(RCtrl.micMax);
          Serial.print("  G = ");
          Serial.print(GCtrl.micMin); Serial.print(","); Serial.print(GCtrl.micMax);
          Serial.print("  B = ");  
          Serial.print(BCtrl.micMin); Serial.print(","); Serial.print(BCtrl.micMax);  
          Serial.print("  Bright:  R = ");
          Serial.print(RCtrl.brightness);
          Serial.print("  G = ");
          Serial.print(GCtrl.brightness); 
          Serial.print("  B = ");
          Serial.print(BCtrl.brightness);    
           }
          RCtrl.WriteBright();
          BCtrl.WriteBright();
          GCtrl.WriteBright(); 
         
           if (SENDSERIAL) {
           Serial.print("  LastBright:  R = ");
          Serial.print(RCtrl.LastBrightness);
          Serial.print("  G = ");
          Serial.print(GCtrl.LastBrightness); 
          Serial.print("  B = ");
          Serial.println(BCtrl.LastBrightness); 
           }
          }
  }
}



void ChangeMode()
{
  // Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"); 
  AutoMode=!AutoMode;
  if (AutoMode)  digitalWrite(AutoPin,HIGH);
  else digitalWrite(AutoPin,LOW);
}

