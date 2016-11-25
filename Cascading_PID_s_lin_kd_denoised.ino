
#include <PID_v1.h>
#include <Encoder.h>

//Mechanical Sytem Properties
  //max linear travel ~8200 Encoder counts
  //full pendulum rotation =7200 Encoder counts

//Variable Definitions

  //Pin Definitions
    byte pin_photoint=13;
    byte pin_pwm=5;
    byte pin_left=7; //Left and right might change depending how you connected the dc motor. CHECK THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    byte pin_right=6;
    byte pin_lin_enc1=2;  //Need to use InterruptPin
    byte pin_lin_enc2=3;  //Need to use InterruptPin
    byte pin_rot_enc1=18;  //Need to use InterruptPin
    byte pin_rot_enc2=19;  //Need to use InterruptPin
    byte pin_poti1=A1;
    byte pin_poti2=A2;
    byte pin_poti3=A3;
    byte pin_poti4=A4;
    byte pin_poti5=A5;
    byte pin_poti6=A6;
    byte pin_poti7=A7;
    byte pin_joystick=A8;
    byte pin_button=15;
    byte pin_led=14;
   
  //Speeds
    byte setup_drive_speed=105; //speed[pwm] used to drive on travel limit runs 
    byte setup_zero_speed=55;  //speed[pwm] at which the cart completely stops moving, used for mapping the PWM Output

  //Linear and angle Limits for Controller
    int limit_lin_min=400;    //min Limit (in direction torwards limit switch--> right)
    int limit_lin_max=7800;   //max Limit (left)
    double limit_angle=200; //limit at which it becomes impossible to catch the pendulum again if exeeded, active control of pendulum is stopped and cart is stopped if exeeded. Limit works in both direction (+ and -) at the equilibrium point
    
 
  //Deadband Variables
    
    byte rot_deadband=0; //In counts in each direction
    
  //Miscellaneous Variables
    boolean button_current=LOW;
    boolean button_last=LOW;
    boolean poti_enable=LOW;
    int rot_offset=0;   //Offset from Vertical, System inherent error. Use if System always tends to run in one direction
    double Err_angle;
    double lastErr_angle=0;
    double timer;
    double lin_speed;
    double rot_speed;
    const int array_length=50;
    double lin_position_array[array_length];
    double angle_array[array_length];
    double timer_array[array_length];
    double angle;
    double lin_position;
    double lin_Setpoint_outer=4100;
    double lin_kd_avg;  //lin kD gain used in an additional controller that averages the kD over array_length controller cycles.

  //Buffer Arrays
   double timer_data[8000];
   double rot_Setpoint_data[8000];
   double lin_position_data[8000];
   double angle_data[8000];
   double Output_data[8000];


//Encoder Setups
  Encoder linEnc(pin_lin_enc1,pin_lin_enc2);                
  Encoder rotEnc(pin_rot_enc1,pin_rot_enc2);  
//PID Setup
  //PID Variables
    double Output, rot_Setpoint=0, lin_Setpoint=4100; //4100 is approx. middle of track
    double rot_kp=0, rot_ki=0, rot_kd=0.00;  
    double lin_kp=0, lin_ki=0.0, lin_kd=0;  
  //PID Controller initialisation
    PID rot_Controller(&angle,&Output,&rot_Setpoint,rot_kp,rot_ki,rot_kd,DIRECT);
    PID lin_Controller(&lin_position,&rot_Setpoint,&lin_Setpoint,lin_kp,lin_ki,lin_kd,DIRECT); 




void setup() {

       Serial.begin(115200); 
       
       pinMode(pin_photoint,INPUT);
       pinMode(pin_button,INPUT);
       pinMode(pin_led,OUTPUT); 
//Calibration Run       
       //Initialize Encoder counts
       linEnc.write(10000); //Choose so that linEnc.read() cant become 0 on intitial reference run --> choose bigger than maximum number of possible steps in whole travel
       rotEnc.write(-3600); //set when pendulum is hanging straight down due to gravity
       delay(500);
       
     
      

      

    //Driving towards the limit switch to reference linear position
      //give it a little bump to beginn cause setup_drive_speed is enough power to maintain the movement but not to start it initially
        go_right(180);
        delay(10);
      //moving with normal speed torwards limit switch until Limit Switch (Photointerrupter) is triggered            
        while (linEnc.read()!=0){
            go_right(setup_drive_speed);
            byte photoint=digitalRead(pin_photoint);
            if (photoint==0){ 
              break;
              }
            }
              
              go_stop();          //Stop cart
              delay(300);
              linEnc.write(0);    //Zero linear Encoder
              
    
     //Drive to the middel of the travelrange
          //little bump to start with
                    go_left(180);
                    delay(10);
          //Drive torwars middle of track until its reached
         while (linEnc.read()<lin_Setpoint){    
                go_left(setup_drive_speed);
              }
              
              go_stop();
              delay(500);           

//PID Settings
     rot_Controller.SetMode(AUTOMATIC);
     rot_Controller.SetOutputLimits(-255,255); 
     rot_Controller.SetSampleTime(2); //2
     lin_Controller.SetMode(AUTOMATIC);
     lin_Controller.SetOutputLimits(-300,300); 
     lin_Controller.SetSampleTime(2);
     

delay(300);
}


void loop() {
  
//lin_speed and angle_speed calculations
//a single controller cycle is not sufficient to calculate the speeds due to the "low" resolution of the encoders. In one cycle the linear position reading might not even change even though its runningg at full speed (depending von the sample time.
//The speeds therefore are calculated by averaging over array_lenght controller cycles
  //Pushing the linear postion values one step down in the array to make place at element [0] for the present linear position  
    for (int x=array_length; x>1; x--){
      lin_position_array[x-1]=lin_position_array[x-2];
    }
  //Pushing the angle values one step down in the array to make place at element [0] for the present angle    
    for (int x=array_length; x>1; x--){
      angle_array[x-1]=angle_array[x-2];
    }
 
  //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds    
    for (int x=array_length; x>1; x--){
      timer_array[x-1]=timer_array[x-2];
    }
    
  //reading the present values for timer, angle and linear postion and save them to element 0 in their arrays  
    timer=millis();
    timer_array[0]=timer;
    angle=rotEnc.read();
    angle_array[0]=angle;
    lin_position=linEnc.read();
    lin_position_array[0]=lin_position;
  //calculate lin_speed and rot_speed as averages over 5 controller cycles  
    lin_speed=(lin_position_array[0]-lin_position_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);
    rot_speed=(angle_array[0]-angle_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);


//Setting PID Gains via Potentiometers  
  
    lin_kp=double(analogRead(pin_poti1))/40000;               
    lin_ki=double(analogRead(pin_poti2))/60000;         
    lin_kd=double(analogRead(pin_poti3))/500000;   
    rot_kp=double(analogRead(pin_poti4))/100;
    rot_ki=double(analogRead(pin_poti5))/1;
    rot_kd=double(analogRead(pin_poti6))/20000;
    lin_kd_avg=double(analogRead(pin_poti7))/5;
  
  
  
  //Values viable if lin_ki,rot_kp,rot_ki,lin_kd_avg are aquired via Potentiometer. Probably a timing issue.
  //Overwrite Gains with fixed Values
  button_current = digitalRead(pin_button);

  if (button_current == HIGH&& button_last==LOW) {
    poti_enable=!poti_enable;
    delay(300);
  } 
  if (poti_enable==LOW){
    lin_kp=0.01645;
    lin_ki=0;
    lin_kd=0.000706;
    rot_kp=3,54;
    rot_ki=54;
    rot_kd=0.01549999;
    lin_kd_avg=152,4;
  }
  digitalWrite(pin_led,poti_enable);

//Reading lin_Setpoint via Joystick Potentiometer
  lin_Setpoint_outer=double(analogRead(pin_joystick))/0.125;  
  lin_Setpoint=lin_Setpoint_outer-lin_kd_avg*lin_speed;


//Main Controller Part

    if (lin_position>limit_lin_min &&lin_position<limit_lin_max){       //perform as long carriage is within the linear limits
                if (angle<limit_angle &&angle>(limit_angle*-1)){    //perform as long as pendulum is within the angular limits  
                                //Adaptive Tuning
                                  if (lin_position<(lin_Setpoint+100)&&lin_position>(lin_Setpoint-100)){
                                    lin_kp=lin_kp*1;
                                    lin_ki=lin_ki*1;
                                    lin_kd=lin_kd*1;
                                    rot_kp=rot_kp*1;
                                    rot_ki=rot_ki*1;
                                    rot_kd=rot_kd*1;
                                  }
                                //Execute linear PID controller
                                  lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd);
                                  lin_Controller.Compute();
                                  rot_Setpoint=rot_Setpoint+rot_offset;
                                //Execute angular PID controller
                                  Err_angle=rot_Setpoint-angle;
                                  rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd);
                                  rot_Controller.Compute();
                                  //Serial.print(Output);
                                  //Serial.print(",");
                                    
                                //Output Modulation (experimental feature)
                                  //Output=Output*1; //*1= no Modulation
                                  //Output=constrain(Output,-255,255);
                                    
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
                //stop carrigage if pendulum has tipped out of the angular limits
                  else{   
                    go_stop();    
                    Output=0;
                  }
                
              
    }
    //drive back into linear limits after they were exeeded
      else{ 
              //stop the cart almost immediatly before its running into a mechanical stop and gets damaged by giving a short burst at full throttle in the reverse direction   
                if (lin_position>limit_lin_max){go_right(255);delay(10);}
                if (lin_position<limit_lin_min){go_left(255);delay(10);}
                go_stop();         
                delay(200);
              //Drive the cart back into the limits after its stopped  
                if(linEnc.read()<limit_lin_min){      //if outside of right limit
                    while(linEnc.read()<limit_lin_min+800){ //drive it back into the travel range, just a little bit over the actual limit
                    go_left(180);
                    delay(10);
                    go_left(setup_drive_speed);
                    }
                    go_stop();
                }
                
                if(linEnc.read()>limit_lin_max){      //if outside of left limit
                    while(linEnc.read()>limit_lin_max-800){ //drive it back into the travel range, just a little bit over the actual limit
                    go_right(180);
                    delay(10);
                    go_right(setup_drive_speed);
                    }
                    go_stop();
                }
                
          }
/*
//Data Output to Serial Monitor for analysis
  Serial.print(timer);
  Serial.print(",");
  Serial.print(rot_Setpoint,5);
  Serial.print(",");
  Serial.print(lin_position);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.print(Output);
  Serial.print(",");
  Serial.print(lin_speed,10);
  Serial.print(",");
  Serial.print(rot_speed,10);
  Serial.print(",");
  Serial.print("\t");
  Serial.print(lin_kp,10);
  Serial.print(",");
  Serial.print(lin_ki,10);
  Serial.print(",");
  Serial.print(lin_kd,10);
  Serial.print(",");
  Serial.print(lin_kd_avg,10);
  Serial.print(",");
  Serial.print("\t");
  Serial.print(rot_kp,10);
  Serial.print(",");
  Serial.print(rot_ki,10);
  Serial.print(",");
  Serial.println(rot_kd,10);
*/
/*
Serial.print(analogRead(pin_poti1));
Serial.print(",");
Serial.print("\t");
Serial.print(analogRead(pin_poti2));
Serial.print(",");
Serial.print("\t");
Serial.print(analogRead(pin_poti3));
Serial.print(",");
Serial.print("\t");
Serial.print(analogRead(pin_poti4));
Serial.print(",");
Serial.print("\t");
Serial.print(analogRead(pin_poti5));
Serial.print(",");
Serial.print("\t");
Serial.print(analogRead(pin_poti6));
Serial.print(",");
Serial.print("\t");
Serial.println(analogRead(pin_poti7));
*/



}
//End Loop

//Procedure Declarations
  //move cart to the left with 'velocity'
    void go_left(int velocity){       
       digitalWrite(pin_right,LOW); 
       digitalWrite(pin_left,HIGH);
       analogWrite(pin_pwm,velocity);
      }
      
  //move cart to the right with 'velocity'
    void go_right(int velocity){      
       digitalWrite(pin_right,HIGH); 
       digitalWrite(pin_left,LOW);
       analogWrite(pin_pwm,velocity);
    }
    
  //stop the cart
    void go_stop(){                   
      analogWrite(pin_pwm,0);
    }


