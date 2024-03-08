#include <PID_v1.h>
#include <Math.h>
#define outputA 2 //Encoder B
#define outputB 3 //Encoder A
#define pwmpin 4 
#define IN1 6//Motor IN1
#define IN2 5//Motor IN2
double val_volt = 0.00;
double vout_fit = 0.00;
const int SMOOTHING_WINDOW_SIZE_position = 15;
int _samples_position[SMOOTHING_WINDOW_SIZE_position];
int _curReadIndex_position = 0;
double _sampleTotal_position = 0.00;
double _sampleAvg_position = 0.00;
double position_max = 0.00;
double busposition;

double lin_maxlimit;
double lin_minlimit;
double angle_maxlimit;
double angle_minlimit;

unsigned long currenttimeAngle = 0;
unsigned long previoustimeAngle = 0;

double Angleconstant=0.3773584906;
double AngleMeasure=0;
double Angleoffset=786;

unsigned long timer = 0;
unsigned long timer_1 = 0;
unsigned long prevtimer1 =0;
int pos_prev = 0;
double pos_i = 0;
volatile long encoderValue = 0;


byte setup_drive_speed=105;
byte setup_zero_speed=0;

double Output_cart;
double Output, rot_Setpoint=0, lin_Setpoint=0;
double rot_kp=0, rot_ki=0, rot_kd=0.00;  
double lin_kp=0, lin_ki=0.0, lin_kd=0; 
double rot_input;
PID rot_Controller(&AngleMeasure,&Output,&rot_input,rot_kp,rot_ki,rot_kd,DIRECT);
PID lin_Controller(&pos_i,&Output_cart,&lin_Setpoint,lin_kp,lin_ki,lin_kd,DIRECT); 
double Err_angle;/////////no need?

unsigned long currentDebug = 0;
unsigned long previousDebug = 0;

unsigned long currentswig_up = 0;
unsigned long previousswing_up = 0;

double rawdata=0;
void setup() {
    // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(outputA,INPUT);
  pinMode(outputB,INPUT);
  pinMode(pwmpin,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(outputA),updateEncoder, RISING);
  lin_maxlimit=520;
  lin_minlimit=-520;
  angle_maxlimit=30;
  angle_minlimit=-30;
  rot_Controller.SetMode(AUTOMATIC);
  rot_Controller.SetOutputLimits(-255,255); 
  rot_Controller.SetSampleTime(2);///ok
  lin_Controller.SetMode(AUTOMATIC);
  lin_Controller.SetOutputLimits(-255,255);
  //lin_Controller.SetSampleTime(10);//ok but osillate
  lin_Controller.SetSampleTime(6);
}

void loop() {
  readposition();
  measure_Angleposition();
  self_balance();
  DebugLoop();
  // put your main code here, to run repeatedly:

}
void self_balance(){
  
//          lin_kp=0.000005; //////////
//          lin_ki=0.0000005; 
//          lin_kd=0.00000025;   
//          rot_kp=25;
//          rot_ki=95;
//          rot_kd=1.1; 

//          lin_kp=0.009; //////////5/10
//          lin_ki=0; 
//          lin_kd=0.0009;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5; 

//          lin_kp=0.0099; //////////5.1/10
//          lin_ki=0; 
//          lin_kd=0.00099;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5; 

//          lin_kp=0.00999; //////////5.2/10
//          lin_ki=0; 
//          lin_kd=0.000999;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5; 

//          lin_kp=0.01; //////////5.2/10 lin sampling 6 lin data 6
//          lin_ki=0; 
//          lin_kd=0.001;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5;

//          lin_kp=0.01; ///////5.3/10 lin sampling 6 lin data 6
//          lin_ki=0; 
//          lin_kd=0.00125;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5;

//          lin_kp=0.01; ///////5.4/10 lin sampling 6 lin data 6
//          lin_ki=0; 
//          lin_kd=0.00135;   
//          rot_kp=60;
//          rot_ki=110;
//          rot_kd=1.5;
//

          lin_kp=0.0125; ///////5.5/10 lin sampling 6 lin data 6
          lin_ki=0; 
          lin_kd=0.00135;   
          rot_kp=60;
          rot_ki=110;
          rot_kd=1.5;



                                 
    if(lin_maxlimit>pos_i and pos_i>lin_minlimit){
    if(angle_maxlimit>AngleMeasure and AngleMeasure>angle_minlimit){
                                  lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd);
                                  lin_Controller.Compute();
                                  rot_input=rot_Setpoint+Output_cart;
                                  rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd);
                                  rot_Controller.Compute();
                                  if(Output>0){
                                    Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_right(Output);
                                    }
                                    
                                  if(Output<0){
                                    Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_left(Output);
                                    Output=-Output; //necessary just for plotting purposes during data analysis
                                    }
                              
                                  if(Output==0){
                                    go_stop;
                                    Output=0;
                                    }
                               
     }
               else{   
                    go_stop();    
                    Output=0;
                  }
    }
        else{  
           
                    go_stop();    
                    Output=0;
                  }
  
  }
void DebugLoop(){
  currentDebug=millis();
  if(currentDebug-previousDebug>50){ 
      //Serial.print("Angle: "); 
//      Serial.print(rawdata);
//      Serial.print(",");
      Serial.print(-255);
      Serial.print(",");
      Serial.print(255);
      Serial.print(",");
      Serial.print(AngleMeasure);
      Serial.print(",");
//      //Serial.print("position : ");
      Serial.print(pos_i);
      Serial.print(",");
      //Serial.print("Output: ");
      Serial.println(Output);
    previousDebug=currentDebug;
    }
  }
void measure_Angleposition(){
  currenttimeAngle=millis();
  if(currenttimeAngle-previoustimeAngle>1){
   busposition = analogRead(A0);
  _sampleTotal_position = _sampleTotal_position - _samples_position[_curReadIndex_position];
  _samples_position[_curReadIndex_position] = busposition;
  _sampleTotal_position= _sampleTotal_position + _samples_position[_curReadIndex_position];
  _curReadIndex_position = _curReadIndex_position + 1;
  if (_curReadIndex_position >= SMOOTHING_WINDOW_SIZE_position) {
    _curReadIndex_position = 0;
  }
  _sampleAvg_position = _sampleTotal_position / SMOOTHING_WINDOW_SIZE_position; 
//  AngleMeasure=(Angleconstant*_sampleAvg_position)-Angleoffset;
  //AngleMeasure=((_sampleAvg_position-Angleoffset)*Angleconstant)+2; ///angle -201.5--->-201.6 best --->-201.9 ok
  //rawdata=((busposition-Angleoffset)*Angleconstant)-1.5;
  AngleMeasure=((_sampleAvg_position-Angleoffset)*Angleconstant)-1.5-6.05;
  previoustimeAngle=currenttimeAngle;
  }
}
void readposition(){
  timer_1 = millis();
  if (timer_1 - prevtimer1 > 6){
  noInterrupts();
  pos_i = encoderValue;
  interrupts();
  prevtimer1 = timer_1;
  }
}
void updateEncoder(){
   int bState = digitalRead(outputB);
   int counter = 0;
  if (bState>0) {
     encoderValue++;;
   } 
   else {
     encoderValue--;
   }
  //pos_i =+ encoderValue;
}
void go_left(double output){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  //analogWrite(pwmpin,output+offest_pwm);
  analogWrite(pwmpin,output);
  //Serial.println(output+offest_pwm);
  }
void go_right(double output){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(pwmpin,output);
  //analogWrite(pwmpin,output+offest_pwm);
  //Serial.println(output+offest_pwm);
  }
void go_stop(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  analogWrite(pwmpin,0);
  }
void go_brake(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  analogWrite(pwmpin,0);
  }
