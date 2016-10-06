#include <PID_v1.h>
#include <Encoder.h>

//Mechanical Sytem Properties
  //max linear travel ~8200 Encoder counts
  //full pendulum rotation =7200 Encoder counts

//Variable Definitions

  //Pin Definitions
    byte pin_photoint=13;
    byte pin_pwm=6;
    byte pin_left=7; //Left and right might change depending how you connected the dc motor. CHECK THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    byte pin_right=8;
    byte pin_lin_enc1=2;
    byte pin_lin_enc2=3;
    byte pin_rot_enc1=18;
    byte pin_rot_enc2=19;
    byte pin_poti2=A6;
    byte pin_poti3=A7;
    byte pin_poti7=A8;
    byte pin_poti8=A9;
    byte pin_poti9=A10;
   
  //Speeds
    byte setup_drive_speed=105; //speed[pwm] used to drive on travel limit runs 
    byte setup_zero_speed=60;  //speed[pwm] at which the cart completely stops moving, used for mapping the PWM Output

  //Linear and angle Limits for Controller
    int limit_lin_min=800;    //min Limit (in direction torwards limit switch--> right)
    int limit_lin_max=7400;   //max Limit (left)
    double limit_angle=200; //limit at which it becomes impossible to catch the pendulum again if exeeded, active control of pendulum is stopped and cart is stopped if exeeded. Limit works in both direction (+ and -) at the equilibrium point
    
 
  //Deadband Variables
    byte output_deadband=0;
    byte rot_deadband=0; //In counts in each direction
    
  //Miscellaneous Variables
    int rot_offset=0;   //Offset from Vertical, System inherent error. Use if System always tends to run in one direction
    double Err_angle;
    double lastErr_angle=0;
    double timer;
    double lin_speed;
    double rot_speed;
    double lin_position_array[5];
    double angle_array[5];
    double timer_array[5];
    double angle;
    double lin_position;



//Encoder Setups
  Encoder linEnc(pin_lin_enc1,pin_lin_enc2);                
  Encoder rotEnc(pin_rot_enc1,pin_rot_enc2);  
//PID Setup
  //PID Variables
    double Output, rot_Setpoint, lin_Setpoint=4100; //4100 is approx. middle of track
    double rot_kp=0, rot_ki=0, rot_kd=0.00;  
    double lin_kp=0, lin_ki=0.0, lin_kd=0;  
  //PID Controller initialisation
    PID rot_Controller(&angle,&Output,&rot_Setpoint,rot_kp,rot_ki,rot_kd,DIRECT);
    PID lin_Controller(&lin_position,&rot_Setpoint,&lin_Setpoint,lin_kp,lin_ki,lin_kd,DIRECT); 




void setup() {

       Serial.begin(115200); 
       
       pinMode(pin_photoint,INPUT);

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
     rot_Controller.SetSampleTime(3);
     lin_Controller.SetMode(AUTOMATIC);
     lin_Controller.SetOutputLimits(-300,300); 
     lin_Controller.SetSampleTime(3);
     

delay(300);
}


void loop() {
  
//lin_speed and angle_speed calculations
//a single controller cycle is not sufficient to calculate the speeds due to the "low" resolution of the encoders. The speeds therefore are calculated by averaging over 5 controller cycles
  //Pushing the linear postion values one step down in the array to make place at element [0] for the present linear position  
    lin_position_array[4]=lin_position_array[3]; 
    lin_position_array[3]=lin_position_array[2];
    lin_position_array[2]=lin_position_array[1];
    lin_position_array[1]=lin_position_array[0];
  //Pushing the angle values one step down in the array to make place at element [0] for the present angle    
    angle_array[4]=angle_array[3];
    angle_array[3]=angle_array[2];
    angle_array[2]=angle_array[1];
    angle_array[1]=angle_array[0]; 
  //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds    
    timer_array[4]=timer_array[3];
    timer_array[3]=timer_array[2];
    timer_array[2]=timer_array[1];
    timer_array[1]=timer_array[0];
  //reading the present values for timer, angle and linear postion and save them to element 0 in their arrays  
    timer=millis();
    timer_array[0]=timer;
    angle=rotEnc.read();
    angle_array[0]=angle;
    lin_position=linEnc.read();
    lin_position_array[0]=lin_position;
  //calculate lin_speed and rot_speed as averages over 5 controller cycles  
    lin_speed=(lin_position_array[0]-lin_position_array[4])/(timer_array[0]-timer_array[4]);
    rot_speed=(angle_array[0]-angle_array[4])/(timer_array[0]-timer_array[4]);


//Setting PID Gains via Potentiometers  
  lin_kp=double(analogRead(pin_poti2))/400000;        //0.0012750000
  lin_ki=double(analogRead(pin_poti3))/60000;         //0
  //lin_kd=double(analogRead(pin_poti3))/10000000;    //0
  rot_kp=double(analogRead(pin_poti7))/100;           //4.6799998283
  rot_ki=double(analogRead(pin_poti8))/1;             //208.0000000000
  rot_kd=double(analogRead(pin_poti9))/20000;         //0






//Main Controller Part

    if (lin_position>limit_lin_min &&lin_position<limit_lin_max){       //perform as long carriage is within the linear limits
                if (angle<limit_angle &&angle>(limit_angle*-1)){    //perform as long as pendulum is within the angular limits  
                                //Execute linear PID controller
                                  lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd);
                                  lin_Controller.Compute();
                                  rot_Setpoint=rot_Setpoint+rot_offset;
                                //Execute angular PID controller
                                  Err_angle=rot_Setpoint-angle;
                                  rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd);
                                  rot_Controller.Compute();
                             
                                    
                                //Output Modulation (experimental feature)
                                  //Output=Output*1; //*1= no Modulation
                                  //Output=constrain(Output,-255,255);
                                    
                                  if(Output>0+output_deadband){
                                    Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_right(Output);
                                    }
                                    
                                  if(Output<0-output_deadband){
                                    Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_left(Output);
                                    Output=-Output; //necessary just for plotting purposes during data analysis
                                    }
                              
                                  if(Output<output_deadband&&Output>-output_deadband){
                                    go_stop;
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
  Serial.print("\t");
  Serial.print(rot_kp,10);
  Serial.print(",");
  Serial.print(rot_ki,10);
  Serial.print(",");
  Serial.println(rot_kd,10);



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


