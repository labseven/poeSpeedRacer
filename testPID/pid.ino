//PID control loop using Adafrut motor shield and a thermistor
//Henry Rachootin, Adam Selker
//QEA Fall 2017
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>

const int mainTempPin = A0, auxTempPin = A1, extra5VPin = 9;
const int heaterPin = 1; //I think this might actually be a pin *on the shield*

const float goal = 35; //Celsius
const float resistance = 9.3, voltage = 11.2, maxPower = 3.75; //Watts
const float kp = 1, ki = 0.01, kd = 1;

bool heating = false; //Are we running the heater (with pid control)?
long unsigned int lastTime, loopTime = 0; //0 to indicate first loop (i.e. don't set anything)
float p, i, d, temp, lastTemp, err, desiredPower, actualPower;
int pwm;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *myMotor = AFMS.getMotor(heaterPin); // Select which 'port' M1, M2, M3 or M4. In this case, M1

float getTemp(float v) {
  float V = v / 1024.0 * 5.0; //Don't blame me, blame the QEA people
  const float Vs = 5; // supply voltage
  const float Rpullup = 10e3; // pullup resistor on circuit is 10k
  const float R0 = 10e3; // this is the resistance at 25 celcuis, from datasheet
  const float T0 = 298.15; // this is the temperature in K at 25 Celcius.
  const float B = 3950; // this is from the data sheet
  const float Rt = Rpullup*V/(Vs-V);
  const float r_inf = R0*exp(-B/T0);

  return B/(log(Rt/r_inf))-273;
}

void checkInputCommands() {
 if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == 'h') {
      myMotor->run(FORWARD);
      heating = true;
    } else if (receivedChar == 'n') {
      myMotor->run(RELEASE);
      heating = false;
    }
  }
}

void setup() { 

  pinMode(mainTempPin, INPUT);
  pinMode(auxTempPin, INPUT);

  pinMode(extra5VPin, OUTPUT);
  digitalWrite(extra5VPin, HIGH);

  myMotor->run(RELEASE); //Stop the motor at the beginning

  Serial.begin(9600);

  lastTime = millis();
  loopTime = 0;
}


void loop() { 

  checkInputCommands(); //See if we need to start or stop heating

  temp = getTemp(analogRead(mainTempPin));
  err = goal - temp;

  p = err * kp;
  i += err * loopTime * ki;
  if (!heating) i = 0; //Set after calclating to keep loop time more constant
  if (loopTime) d = (err - lastTemp) / loopTime * kd; //Don't divide by 0!
  else d = 0;

  Serial.println("P: " + String(p) + " I: " + String(i) + "D: " + String(d));

  desiredPower = p + i + d;
  actualPower = (desiredPower<0 ? 0 : (desiredPower>maxPower ? maxPower : desiredPower)); //Limit power to possible range
  pwm = heating ? 255*actualPower*resistance/(voltage*voltage) : 0;

  myMotor->setSpeed(pwm);
  if(heating) myMotor->run(FORWARD);

  Serial.print(temp);
  Serial.print("\t");
  Serial.print(getTemp(analogRead(auxTempPin)));
  Serial.print("\t");
  Serial.print(desiredPower);
  Serial.print("\t");
  Serial.println(actualPower);

  loopTime = millis() - lastTime;
  lastTime = millis();

}
