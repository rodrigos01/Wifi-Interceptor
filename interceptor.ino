#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ultrasonic.h>

Servo servo;

const int THROTTLE = 255;

const int RIGHT_FRONT = 5;
const int RIGHT_BACK = 6;

const int LEFT_FRONT = 9;
const int LEFT_BACK = 10;

const int SIDE_RIGHT = 1;
const int SIDE_LEFT = 2;

const int GEAR_R = -1;
const int GEAR_N = 0;
const int GEAR_D = 1;

int current_degrees = 0;
const int SERVO_PORT = 7;

//RX pino 12, TX pino 11
SoftwareSerial esp8266(2, 3);
const byte RST=4;

int throttle=0; //0:127
int throttle_right=0;
int throttle_left=0;
int gear = 0; //1 = D, 0 = N, -1 = R

//Define os pinos
#define pino_trigger 11
#define pino_echo 12
 
//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(pino_trigger, pino_echo);

int loopStep = 0;
int signals[] = {0, 0, 0};
int signal0, signal180, signal90;
int start;
boolean isTurning;

void setup() {
  Serial.begin(9600);
  setupServo();
  setupWifi();
  setupCar();
  start=millis();
}

void loop() {
  ////Serial.println(loopStep);
  int elapsed = millis() - start;
  if(elapsed > 5000) {
    park();
    turnServo(0);
    signal0 = wifiLoopStep();
    Serial.print("First signal: ");
    Serial.println(signal0); 
    if(signal0 == 1) {
      Serial.print("Retrying signal");
      signal0 = wifiLoopStep();  
    }
    turnServo(90);
    signal90 = wifiLoopStep();
    turnServo (180);
    signal180 = wifiLoopStep();
    turnServo(0);

    signals[0] = signal0;
    signals[1] = signal90;
    signals[2] = signal180;
    sort(signals, 3);
    goToStrongest(signals[0]);
    start = millis();
  } else {
    float proximity = getProximity();
    if(proximity < 50) {
      if(!isTurning) {
        Serial.println("Danger Ahead. Turning");
        goToStrongest(signals[1]);
        isTurning = true;
      }
    } else {
      isTurning = false;
      gear = GEAR_D;
      turnCar(90);
      moveCar();
    }
  }
}

void goToStrongest(int strongest) {
  //Serial.print("Max read: ");
    ////Serial.println(strongest);
    if (strongest == signal0) {
      turnCar(0);
    } else if (strongest == signal180) {
      turnCar(180);
    } else if (strongest == signal90) {
      turnCar(90);
    }
    moveCar();
}

void sort(int* arr, int size) {
  int aux;  
  for (int i = size - 1; i >= 1; i--) {        
    for (int j = 0; j < i; j++) {          
      if (arr[j] > arr[j + 1]) {                
        aux = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = aux;        
      }
    }
  }
}

/********
 * PROXIMITY
 */
 
void setupProximity()
{
  //Serial.println("Reading sensor data");
}
 
float getProximity()
{
  //Le as informacoes do sensor, em cm e pol
  float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  //Exibe informacoes no serial monitor
  ////Serial.print("Distancia em cm: ");
  ////Serial.println(cmMsec);
  return cmMsec;
}

/*********
* SERVO  *
**********/

void setupServo() {
  
}

void attachServo() {
  if(!servo.attached()) {
    servo.attach(SERVO_PORT);
  }
}

void detachServo() {
  if(servo.attached()) {
    servo.detach();
  }
}

void turnServo(int degrees) {
  ////Serial.print("Move to: ");
  ////Serial.println(degrees);
  attachServo();
  while (degrees != current_degrees) {
    current_degrees = (degrees > current_degrees) ? current_degrees + 1 : current_degrees - 1;
    servo.write(current_degrees);
    delay(10);
  }
  detachServo();
}

/*********
*  WIFI  *
**********/

const String target = "Isobar IWS Brazil";

void setupWifi() {
  //Serial.println("------------");
  pinMode(RST, OUTPUT);
  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST,HIGH);
  delay(1000);
  esp8266.begin(19200);

  //Serial.println("ESP8266 Demo - aguardando 1 segundo");
  delay(1000);
  Serial.println("Enviando RST");
  esp8266.println("AT+RST");  
  Serial.println(recebeResposta());
  Serial.println(recebeResposta()); // o reset tem 2 respostas - OK (antes) e ready (depois) 
  Serial.println("Selecionando modo de operacao misto (AP e estacao)");
  esp8266.println("AT+CWMODE=3");  
  Serial.println(recebeResposta());
  Serial.println("Monitorando sinal");
}

int wifiLoopStep() {
  esp8266.println("AT+CWLAP");  
  String resposta = recebeResposta();
  int db = getTargetDb(resposta);
  Serial.print("Signal Acquired: ");
  Serial.println(db);
  return db*-1;
}

String recebeResposta() {
  int limite=10000;
  unsigned long chegada=millis();
  boolean continuar=true; 
  String S="";
  unsigned long ultimochar=0;
  while (continuar) { 
    if (esp8266.available()) {
      char c = esp8266.read();
      ultimochar=millis();
      S=S+c;
      //Serial.print(c);
      if (c==10) {  // LF, fim da linha recebida
        byte p=S.indexOf(13);
        String S1=S.substring(0,p);
        if (S1=="OK") continuar=false;
        if (S1=="ready") continuar=false;
        if (S1=="no change") continuar=false;
        if (S1=="ERROR") continuar=false;
      }  
    }  
    if (millis()-chegada > limite) {
      continuar=false;
      //Serial.println("Timeout");
    }
  }
  return S;
}

int getTargetDb(String search) {

  int fim = 0;
  Serial.println(search);
  while(true) {
    int inicio = search.indexOf("\"" + target + "\",", fim) +1;
    if(inicio==0 || inicio == -1) {
      break;
    }
    fim = search.indexOf(")", inicio);
    if(fim == 0 || fim == -1) {
      break;
    }
    String entry = search.substring(inicio, fim);
    String* rede = detalhaSSID(entry);
    String dbString = rede[1];
    Serial.println(dbString);
    int db = dbString.toInt();
    return db;
  }
}

String* detalhaSSID(String linha) {

  int inicio = 0;
  String* rede = new String[5];
  for(int index = 0; index < 5; index++) {
    int fim = linha.indexOf(",", inicio);
    String entry = linha.substring(inicio, fim);
    rede[index] = entry;
    inicio = fim+1;
  }
  return rede;
  
}

/*********
*  CAR   *
**********/

void setupCar() {
  pinMode(RIGHT_FRONT, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(LEFT_FRONT, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
}

void carLoopStep() {
  gear = GEAR_D;
  turnCar(90);
  moveCar();
  delay(2000);
  gear = GEAR_R;
  moveCar();
  delay(2000);
  gear = GEAR_D;
  turnCar(0);
  moveCar();
  delay(500);
  turnCar(180);
  moveCar();
  delay(1000);
  turnCar(90);
  moveCar();
  delay(2000);
}

void moveCar() {
  if(gear == GEAR_D) {
    moveFront();
  } else if(gear == GEAR_R) {
    moveBack();
  } else if (gear == GEAR_N) {
    park();
  }
}

void turnCar(double angle) {
  int power = map(angle, 0, 180, 255, 127);
  Serial.print("power ");
  int side;
  if(angle > 0) {
    Serial.print("left ");
    throttle_right = power;
    throttle_left = THROTTLE;
  } else if(angle < 0) {
    Serial.print("right ");
    throttle_right = THROTTLE;
    throttle_left = power;
  } else {
    throttle_right = THROTTLE;
    throttle_left = THROTTLE;
  }
  Serial.println(power);
}

void moveMotor(int side, int powerFront, int powerBack) {
  int front;
  int back;
  if(side == 1) {
    front = RIGHT_FRONT;
    back = RIGHT_BACK;
  } else if(side == 2) {
    front = LEFT_FRONT;
    back = LEFT_BACK;
  }

  analogWrite(front, powerFront);
  analogWrite(back, powerBack);
}

void moveFront() {
  moveMotor(SIDE_RIGHT, throttle_right, 0);
  moveMotor(SIDE_LEFT, throttle_left, 0);
}

void moveBack() {
  moveMotor(SIDE_RIGHT, 0, throttle_right);
  moveMotor(SIDE_LEFT, 0, throttle_left);
}

void park() {
  moveMotor(SIDE_RIGHT, 0, 0);
  moveMotor(SIDE_LEFT, 0, 0);
  gear = GEAR_N;
}

