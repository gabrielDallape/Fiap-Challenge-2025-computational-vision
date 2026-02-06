#include <Servo.h>

// ================= PINAGEM =================
const int BASE_EN   = 5;
const int BASE_DIR  = 6;
const int BASE_STEP = 7;
const int BASE_END  = 4;
const int GARRA_EN  = 8;
const int GARRA_DIR = 9;
const int GARRA_STEP= 10;
const int GARRA_END = 3;
const int SERVO_PIN = 2;
const int RELE_PIN  = 11;
Servo myServo;

// ================= LIMITES & ESCALAS =================
#define PULSE_US       24
#define EN_DELAY_US    300
#define HOME_LIMIT     90000L
const long MAX_STEPS = 52000;
const long MAX_LOGIC = 10000;

// ======= ZERO FICTÍCIO CENTRAL =======
const long GARRA_OFFSET_STEPS = 33000;  // Meio da esteira
const long GARRA_NEG_RANGE    = 4500;

// ================= DIREÇÕES =================
bool BASE_FORWARD_HIGH  = true;
bool GARRA_FORWARD_HIGH = false;

// ================= ESTADO ===================
long basePos  = 0;
long garraPos = 0;

// ================= AJUSTES =================
unsigned long BASE_CRUISE_SPS = 8000UL;
unsigned long GARRA_SPEED_SPS = 20000UL;
bool BASE_CRUISE_DIR_FORWARD = true;

// Servo config
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
unsigned long SERVO_STEP_PERIOD_US = 3000UL; // servo rápido

// Coordenadas das caixas
const long CAIXA_A_BASE = 2000;
const long CAIXA_B_BASE = 40000;

// ================= RELÉ ====================
bool RELE_ACTIVE_HIGH = false;
inline void releOn()  { digitalWrite(RELE_PIN, RELE_ACTIVE_HIGH ? HIGH : LOW); }
inline void releOff() { digitalWrite(RELE_PIN, RELE_ACTIVE_HIGH ? LOW : HIGH); }
inline void releOffSafe() { releOff(); }

// ================= Helpers =================
inline void setEnable(int enPin, bool on){ digitalWrite(enPin, on ? LOW : HIGH); delayMicroseconds(EN_DELAY_US); }
inline void setDir(int dirPin, bool forward, bool forwardHigh){
  digitalWrite(dirPin, forward ? (forwardHigh ? HIGH : LOW)
                               : (forwardHigh ? LOW : HIGH));
}
inline bool endAtivo(int pin){ return digitalRead(pin) == LOW; }

// =================== Escalas ==========================
long escalaParaStepsGarra(long gLogic){
  gLogic = constrain(gLogic, 0, MAX_LOGIC);
  long zeroLogico = MAX_LOGIC / 2;
  long delta = gLogic - zeroLogico;

  if (delta == 0) return GARRA_OFFSET_STEPS;

  if (delta < 0) {
    long steps = GARRA_OFFSET_STEPS + (delta * (GARRA_OFFSET_STEPS - 18000)) / zeroLogico;
    return max(18000L, steps);
  } else {
    long steps = GARRA_OFFSET_STEPS + (delta * (47000 - GARRA_OFFSET_STEPS)) / zeroLogico;
    return min(47000L, steps);
  }
}

// ======= HOMING NORMAL =======
void homingSimultaneo(){
  Serial.println("[HOMING] start");

  setEnable(BASE_EN, true);
  setDir(BASE_DIR, false, BASE_FORWARD_HIGH);
  while (!endAtivo(BASE_END)) {
    digitalWrite(BASE_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(BASE_STEP, LOW); delayMicroseconds(5);
  }
  setEnable(BASE_EN, false);
  basePos = 0;

  setEnable(GARRA_EN, true);
  setDir(GARRA_DIR, false, GARRA_FORWARD_HIGH);
  while (!endAtivo(GARRA_END)) {
    digitalWrite(GARRA_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(GARRA_STEP, LOW); delayMicroseconds(10);
  }

  setDir(GARRA_DIR, true, GARRA_FORWARD_HIGH);
  for (long i = 0; i < GARRA_OFFSET_STEPS; i++) {
    digitalWrite(GARRA_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(GARRA_STEP, LOW); delayMicroseconds(40);
  }

  setEnable(GARRA_EN, false);
  garraPos = GARRA_OFFSET_STEPS;
  Serial.println("[HOMING] OK");
}

// ======= HOMING RÁPIDO ENTRE ITENS =======
void homingSimultaneoRapido(){
  Serial.println("[HOMING RÁPIDO] start");

  setEnable(BASE_EN, true);
  setDir(BASE_DIR, false, BASE_FORWARD_HIGH);
  while (!endAtivo(BASE_END)) {
    digitalWrite(BASE_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(BASE_STEP, LOW); delayMicroseconds(5);
  }
  setEnable(BASE_EN, false);
  basePos = 0;

  setEnable(GARRA_EN, true);
  setDir(GARRA_DIR, false, GARRA_FORWARD_HIGH);
  while (!endAtivo(GARRA_END)) {
    digitalWrite(GARRA_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(GARRA_STEP, LOW); delayMicroseconds(5);
  }

  setDir(GARRA_DIR, true, GARRA_FORWARD_HIGH);
  for (long i = 0; i < GARRA_OFFSET_STEPS; i++) {
    digitalWrite(GARRA_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(GARRA_STEP, LOW); delayMicroseconds(20);
  }

  setEnable(GARRA_EN, false);
  garraPos = GARRA_OFFSET_STEPS;
  Serial.println("[HOMING RÁPIDO] OK");
}

// ===== BASE CRUZEIRO =====
bool baseCruiseOn = false;
unsigned long baseCruisePer = 0, baseCruiseNext = 0;

void baseCruiseStart(unsigned long customSpeed = 0){
  if (baseCruiseOn) return;
  unsigned long sps = (customSpeed > 0 ? customSpeed : BASE_CRUISE_SPS);
  sps = constrain(sps, 500UL, 100000UL);

  setEnable(BASE_EN, true);
  setDir(BASE_DIR, BASE_CRUISE_DIR_FORWARD, BASE_FORWARD_HIGH);

  baseCruisePer = max((unsigned long)(PULSE_US + 2), 1000000UL / sps);
  baseCruiseNext = micros() + baseCruisePer;
  baseCruiseOn = true;
}

void baseCruiseStop(){ baseCruiseOn = false; setEnable(BASE_EN, false); }

void tickBaseCruise(){
  if (!baseCruiseOn) return;
  unsigned long now = micros();
  if ((long)(now - baseCruiseNext) >= 0) {
    digitalWrite(BASE_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(BASE_STEP, LOW);
    baseCruiseNext += baseCruisePer;
    basePos += (BASE_CRUISE_DIR_FORWARD ? +1 : -1);
  }
}

// ===== SERVO =====
void servoDescerSubirComRele(int alvoDeg){
  alvoDeg = constrain(alvoDeg, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int atual = SERVO_MIN_ANGLE;
  unsigned long nextTick = micros();

  while (atual < alvoDeg) {
    tickBaseCruise();
    unsigned long now = micros();
    if ((long)(now - nextTick) >= 0) {
      atual++; myServo.write(atual);
      nextTick += SERVO_STEP_PERIOD_US;
    }
  }

  releOn();
  unsigned long t0 = millis();
  while (millis() - t0 < 250) tickBaseCruise();

  while (atual > SERVO_MIN_ANGLE) {
    tickBaseCruise();
    unsigned long now = micros();
    if ((long)(now - nextTick) >= 0) {
      atual--; myServo.write(atual);
      nextTick += SERVO_STEP_PERIOD_US;
    }
  }
}

// ===== GARRA + DELAY =====
void moverGarraComDelay(long destGarra, unsigned long delayMs, unsigned int velInput, int servoDeg){
  destGarra = constrain(destGarra, 18000L, 47000L);
  long deltaG = destGarra - garraPos;
  bool dirG = (deltaG > 0);
  long remG = labs(deltaG);
  bool garraPronta = false;

  unsigned long velReal = map(velInput, 0, 999, 500, 100000);
  BASE_CRUISE_SPS = velReal;

  setEnable(GARRA_EN, true);
  setDir(GARRA_DIR, dirG, GARRA_FORWARD_HIGH);

  unsigned long per = 1000000UL / (GARRA_SPEED_SPS * 1.3);
  unsigned long tStart = millis();

  while (remG > 0) {
    digitalWrite(GARRA_STEP, HIGH); delayMicroseconds(PULSE_US);
    digitalWrite(GARRA_STEP, LOW); delayMicroseconds(per - PULSE_US);
    remG--;
    garraPos += (dirG ? +1 : -1);
    if (!garraPronta && remG == 0) garraPronta = true;
  }
  setEnable(GARRA_EN, false);

  while (millis() - tStart < delayMs) tickBaseCruise();

  if (garraPronta) {
    baseCruiseStart(BASE_CRUISE_SPS);
    servoDescerSubirComRele(servoDeg);
  }
}

// ===== SOLTAR NA CAIXA =====
void dwellNaCaixaESoltarComTimeout(){
  unsigned long inicio = millis();
  bool soltou = false;
  while (millis() - inicio < 60) {
    tickBaseCruise();
    if (!soltou && millis() - inicio >= 50) {
      releOffSafe();
      soltou = true;
    }
  }
  Serial.println("[CAIXA] Item solto.");
}

// ===== IR PARA CAIXA (RÁPIDO) =====
void irParaCaixa(long baseTarget, unsigned long velAtual){
  baseCruiseStop();
  Serial.print("[CAIXA] Indo para destino "); Serial.println(baseTarget);

  unsigned long velBoost = velAtual * 2.2;
  unsigned long garraBoost = GARRA_SPEED_SPS * 1.6;
  velBoost = constrain(velBoost, 500UL, 100000UL);
  garraBoost = constrain(garraBoost, 5000UL, 60000UL);

  baseTarget = constrain(baseTarget, 0, MAX_STEPS);
  bool dirB = (baseTarget > basePos);
  long deltaB = abs(baseTarget - basePos);
  long remB = deltaB;

  setEnable(BASE_EN, true);
  setDir(BASE_DIR, dirB, BASE_FORWARD_HIGH);
  setEnable(GARRA_EN, true);
  setDir(GARRA_DIR, false, GARRA_FORWARD_HIGH);

  unsigned long perB = 1000000UL / velBoost;
  unsigned long perG = 1000000UL / garraBoost;
  unsigned long nextB = micros() + perB;
  unsigned long nextG = micros() + perG;
  unsigned long endB = 0, endG = 0;
  bool highB = false, highG = false, garraOk = false;

  unsigned long inicio = millis();
  const unsigned long TIMEOUT_MS = 3500;

  while ((!garraOk || remB > 0) || highB || highG) {
    unsigned long now = micros();
    if (millis() - inicio > TIMEOUT_MS) break;

    if (remB > 0 && !highB && (long)(now - nextB) >= 0) {
      digitalWrite(BASE_STEP, HIGH);
      highB = true;
      endB = now + PULSE_US;
      basePos += (dirB ? +1 : -1);
      remB--;
      nextB += perB;
    }
    if (highB && (long)(now - endB) >= 0) {
      digitalWrite(BASE_STEP, LOW);
      highB = false;
    }

    if (!garraOk) {
      if (!highG && (long)(now - nextG) >= 0) {
        digitalWrite(GARRA_STEP, HIGH);
        highG = true;
        endG = now + PULSE_US;
        garraPos -= 1;
        nextG += perG;
      }
      if (highG && (long)(now - endG) >= 0) {
        digitalWrite(GARRA_STEP, LOW);
        highG = false;
      }
      if (endAtivo(GARRA_END)) {
        garraOk = true;
        garraPos = 0;
      }
    }
  }

  setEnable(BASE_EN, false);
  setEnable(GARRA_EN, false);
  dwellNaCaixaESoltarComTimeout();
  homingSimultaneoRapido(); // ⚡ usa homing rápido após soltar item
}

// ===== COMANDO =====
void processarComando(String cmd){
  cmd.trim();
  int s1 = cmd.indexOf(' ');
  int s2 = cmd.indexOf(' ', s1 + 1);
  int s3 = cmd.indexOf(' ', s2 + 1);
  int s4 = cmd.indexOf(' ', s3 + 1);
  if (s1 < 0 || s2 < 0 || s3 < 0 || s4 < 0) {
    Serial.println("Formato: <G> <A|B> <DelayMs> <ServoDeg> <Vel(0–999)>");
    return;
  }

  long gLogic = cmd.substring(0, s1).toInt();
  char cx = toupper(cmd.substring(s1 + 1, s2)[0]);
  unsigned long dMs = cmd.substring(s2 + 1, s3).toInt();
  int servoDeg = cmd.substring(s3 + 1, s4).toInt();
  unsigned int vCruise = cmd.substring(s4 + 1).toInt();
  vCruise = constrain(vCruise, 0, 999);

  long gSteps = escalaParaStepsGarra(gLogic);
  long baseDestino = (cx == 'A') ? CAIXA_A_BASE : CAIXA_B_BASE;

  moverGarraComDelay(gSteps, dMs, vCruise, servoDeg);
  unsigned long velReal = map(vCruise, 0, 999, 500, 100000);
  irParaCaixa(baseDestino, velReal);
}

// ===== SETUP =====
void setup(){
  Serial.begin(115200);
  pinMode(BASE_STEP, OUTPUT); pinMode(BASE_DIR, OUTPUT); pinMode(BASE_EN, OUTPUT);
  pinMode(GARRA_STEP, OUTPUT); pinMode(GARRA_DIR, OUTPUT); pinMode(GARRA_EN, OUTPUT);
  pinMode(BASE_END, INPUT_PULLUP); pinMode(GARRA_END, INPUT_PULLUP);
  pinMode(RELE_PIN, OUTPUT);

  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(SERVO_MIN_ANGLE);
  releOffSafe();

  homingSimultaneo(); // primeiro homing lento
  Serial.println("Formato: <G> <A|B> <DelayMs> <ServoDeg> <Vel(0–999)>");
}

// ===== LOOP =====
void loop(){
  tickBaseCruise();
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) { processarComando(buf); buf = ""; }
    } else buf += c;
  }
}
