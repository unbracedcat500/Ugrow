int motor_positive = 7;
int motor_negative = 8;
int motor_pin = 9;
int motor_speed = 200;
int motor_input_up = 2;
int motor_input_down = 4;


char c;
void setup() {
  Serial.begin(9600);
  pinMode(motor_positive,OUTPUT);
  pinMode(motor_negative,OUTPUT);
  pinMode(motor_pin,OUTPUT);
  pinMode(motor_input_up,INPUT_PULLUP);
  pinMode(motor_input_down,INPUT_PULLUP);
}

void loop() {
  if(digitalRead(motor_input_up) == LOW)
  {
    motor_up();
    Serial.println("up");
  }
  else if(digitalRead(motor_input_down) == LOW)
  {
    motor_down();
    Serial.println("down");
  }
  else{
    stop_motor();
    Serial.println("Stop");
  }
  
 
}
void motor_up()
{
  digitalWrite(motor_positive,LOW);
  digitalWrite(motor_negative,HIGH);
  run_motor(255);
  Serial.println("+up");
  stop_motor();
}
void motor_down()
{
  digitalWrite(motor_positive,HIGH);
  digitalWrite(motor_negative,LOW);
  run_motor(200);
  Serial.println("-down");
  stop_motor();
  
}
void run_motor(int speed_of_motor)
{
  analogWrite(motor_pin,speed_of_motor);
  //delay(50);
   
}
void stop_motor()
{
  digitalWrite(motor_positive,HIGH);
  digitalWrite(motor_negative,HIGH);
  analogWrite(motor_pin,0); 
}

