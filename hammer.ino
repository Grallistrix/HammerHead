//IR sensors
#define Q1 12
#define Q2 11
#define Q3 8
#define Q4 7
#define Q5 4

//disable/enable motor driver
#define SLP 2

/*sterownie: aby lf jechał do przodu (patrząc od tyłu LF) prawy silnik ma się kręcić w prawo (na pin 5 podajemy sygnał PWM a pin 6 zwieramy do masy),
  silnik lewy (na pin 10 sygnał PWM pin 9 do GND)
*/
//PWM control pins RM-right motor, LM-left motor, _l-left rotation
#define RM_l 5
#define RM_r 6
#define LM_r 9
#define LM_l 10            

bool motors_on = 0;
const unsigned int MOTOR_SPEED = 70;
const unsigned int MAX_SPEED = 160;
const unsigned int MIN_SPEED = 0;

//sensors reading
int q1 = 0;
int q2 = 0;
int q3 = 0;
int q4 = 0;
int q5 = 0;

int sum = 0;
int prev_sum = 0;

int goal = 3500;
int flag = 0;
int error = 0;

float kP = MAX_SPEED / goal;
float kD = kP * 10;
float PID = 0.0;
int reading = 0;
int previous_error = 0;
int P = 0;
int D = 0;

void setup() {
    //Motors
    pinMode(LM_l, OUTPUT);
    pinMode(LM_r, OUTPUT);
    pinMode(RM_r, OUTPUT);
    pinMode(RM_l, OUTPUT);
    //tact switch to start a program
    pinMode(3, INPUT_PULLUP);
    //driver enable/disable
    pinMode(SLP, OUTPUT);
    //IR sensors reading
    pinMode(Q1, INPUT);
    pinMode(Q2, INPUT);
    pinMode(Q3, INPUT);
    pinMode(Q4, INPUT);
    pinMode(Q5, INPUT);
    digitalWrite(SLP, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(9, LOW);
    //Serial.begin(9600);
}
void loop() {
    if (digitalRead(3) == HIGH) //Skip loop if not powered on
      return;

      motors_on = 1;
      digitalWrite(SLP, motors_on);
      digitalWrite(6, LOW);
      digitalWrite(9, LOW);

      while (true) {
          if (flag == 0) {
                //rozpędzanie lf-a do MOTOR_SPEED na samym początku potem już stosowany jest algorytm PID - sugerowane zastąpienie funkcji delay() przerwaniem na timerach w ramach refaktoryzacji
              for (float i = 0; i <= MOTOR_SPEED; i+=0.4) {
                   analogWrite(5, i);
                   analogWrite(10, i);
                   delay(10);
               }
            }
          else {
              Reading();
              if (reading == 777) {
                  break;
              }
              PID_counter();
              Movement();
              //delay(5);
            }
            flag += 1;

        }
    
}

//odczyt wartości np. jeśli suma = 1 to znaczy, że czujnik q1 widzi linię czyli ten najbardziej z lewej patrząc od tyłu lf-a a pozostałe nie itd.
void Reading() {

    q1 = digitalRead(Q1) * 1;
    q2 = digitalRead(Q2) * 10;
    q3 = digitalRead(Q3) * 100;
    q4 = digitalRead(Q4) * 1000;
    q5 = digitalRead(Q5) * 10000;

    sum = q1 + q2 + q3 + q4 + q5;

  switch(sum){
    case 0:
        if (prev_sum == 1)
            reading = -1000;     
        else if (prev_sum == 10000)
            reading = 5000;
        break;
    case 100:
        reading = 2000;
        break;
    case 10000:
        reading = 5000;
        break;
    case 1:
        reading=0;
        break;
    case 11:
        reading = 500;
        break;
    case 10:
        reading=1000;
        break;
    case 1000:
        reading=3000;
        break;  
    case 1100:
        reading=2500;
        break;
    case 11000:
        reading=3500;
        break;
    default:
        reading=777;
  }
    prev_sum = sum;
}

void Movement() {
    //spróbować zamiast funkcji constrain() użyć if-ów
    analogWrite(10, constrain(MOTOR_SPEED - PID, MIN_SPEED, MAX_SPEED));
    analogWrite(5, constrain(MOTOR_SPEED + PID, MIN_SPEED, MAX_SPEED));
}

//można pomyśleć nad dodaniem członu całkującego
void PID_counter() {
    error = goal - reading;
    //error = reading;
    P = error;
    D = error - previous_error;
    previous_error = error;
    PID = (kP * P) + (kD * D);
}
