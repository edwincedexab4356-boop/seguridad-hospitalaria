/*
  INFINITY TEC - Hospital Inteligente
  Control y avisos por Bluetooth (HC-05) desde la pagina web, o por el cable USB (monitor serie).

  Placa: Arduino UNO R4 WiFi
  Librerias (Gestor de librerias): "LiquidCrystal I2C" (Frank de Brabander) y "Servo".
            Wire, EEPROM y Arduino_LED_Matrix ya vienen con la placa.

  CONEXIONES
    Sensor PIR (quirofano)    VCC 5V, GND, OUT -> D7
    LED azul   (quirofano)    D6  -> 2x220 seguidas -> pata larga del LED; pata corta -> GND
    LED Habitacion 1          D8  -> 2x220 seguidas -> LED -> GND
    LED Habitacion 2          D11 -> 2x220 seguidas -> LED -> GND
    LED amarillo (pasillo)    D5  -> 2x220 seguidas -> LED -> GND
    LED verde  (puerta abierta)  D3 -> 2x220 seguidas -> LED -> GND
    LED rojo   (alarma)          D4 -> 2x220 seguidas -> LED -> GND
    Buzzer activo             cable ROJO (+) -> D2, cable NEGRO (-) -> GND
    Servo puerta de ENTRADA   MARRON -> GND, ROJO -> 5V, AMARILLO (senal) -> D9
    Servo puerta QUIROFANO    MARRON -> GND, ROJO -> 5V, AMARILLO (senal) -> D10
    Ultrasonico HC-SR04 (en la entrada, mirando hacia afuera)   VCC 5V, GND, TRIG -> D12, ECHO -> D13
                              (los 2 servos juntos piden mucha corriente: si la placa se reinicia o la
                               LCD se borra al moverlos, dales 5 V de otra fuente y une los GND)
    Pantalla LCD 16x2 I2C     VCC 5V, GND, SDA -> SDA, SCL -> SCL   (la direccion se busca sola)
    Bluetooth HC-05 (opcional, para la app)   VCC 5V, GND, TXD -> D0 (RX)
                              RXD <- D1 (TX) con divisor: D1 -> 220 -> RXD, y 2 de 220 seguidas de RXD a GND

  QUE HACE
    - PIR del quirofano: cuando detecta movimiento se prende la luz AZUL (se apaga 10 s despues del ultimo
      movimiento). Si la vigilancia esta prendida y el quirofano NO esta "en uso" = ALARMA (intruso):
      LED rojo parpadea, buzzer suena, la puerta del quirofano se cierra y se prenden las habitaciones.
    - Puertas (servos): se abren desde la app y se cierran solas a los 10 s (o con la orden de cerrar).
      Mientras una puerta esta abierta el LED verde esta prendido. Con alarma la del quirofano no abre.
    - Ultrasonico de la entrada: si alguien se acerca a menos de DIST cm, la puerta de entrada se abre sola
      (puerta automatica) y se queda abierta mientras haya alguien; se cierra 10 s despues.
    - Habitaciones 1 y 2 y luz amarilla del pasillo: se prenden y apagan desde la app (se guardan).
    - LCD: estado, puertas abiertas con cuenta atras, alarmas. La matriz de la placa muestra el estado.
    - Al encender, el PIR necesita unos 30 s para calmarse: en ese tiempo no da alarma.

  ORDENES (una por linea, por Bluetooth o por el monitor serie a 9600):
    ON  OFF (vigilancia)   ALERTA  SILENCIAR  ESTADO
    ENTRADA:1 / ENTRADA:0        (abre / cierra la puerta de entrada)
    PUERTA_Q:1 / PUERTA_Q:0      (abre / cierra la puerta del quirofano)
    USO:1 / USO:0                (quirofano en uso: el PIR solo prende la luz azul, sin alarma)
    HAB:n:1 / HAB:n:0            (luz de la habitacion n = 1 o 2; n = 0 las dos)
    AMARILLA:1 / AMARILLA:0      (luz amarilla del pasillo)
    DIST:n                       (cm, 5-150: a que distancia se abre sola la entrada)
    AUTO:1 / AUTO:0              (entrada automatica con el ultrasonico prendida / apagada)
    ANG_E:n / ANG_Q:n            (grados que abre la entrada / el quirofano, 10-180; se guarda)
    GIRO:n                       (calibracion del servo 360 de la entrada: ms que tarda en girar 90 grados, 50-2000)
    INVERTIR                     (si la puerta de entrada abre hacia el lado contrario)
  ENVIA cada 0,5 s:
    ST,sistema,alerta,origen,pir,azul,entrada,puerta_q,seg_entrada,seg_q,hab1,hab2,amarilla,en_uso,lcd_ok,calentando,distancia,persona,dist_entrada,auto,ang_e,ang_q,giro_ms,invertir
    origen: 0 ninguno, 1 intruso en quirofano, 3 manual (app)
  Y ademas:  EV,texto      LC,linea1,linea2 (lo que muestra la LCD)
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "Arduino_LED_Matrix.h"

#define BT Serial1   // HC-05 en D0/D1 (en el R4 el USB va aparte)

const byte P_BUZ = 2, P_VERDE = 3, P_ROJO = 4, P_AMAR = 5, P_AZUL = 6, P_PIR = 7;
const byte P_HAB1 = 8, P_SERVO_E = 9, P_SERVO_Q = 10, P_HAB2 = 11, P_TRIG = 12, P_ECHO = 13;
const int DIST_MAX = 300;
const int CERRADA = 0;     // grados del servo del quirofano cuando esta cerrada
// El servo de la ENTRADA es de 360 (rotacion continua): no sabe en que angulo esta,
// asi que se limita a 90 grados por TIEMPO: gira giroMs milisegundos y se detiene.
const int PARADO = 90;     // con 90 el servo de 360 se queda quieto
const int VELOC = 35;      // cuanto se aleja de 90 al girar (mas = mas rapido)
const unsigned long PUERTA_MS = 10000;          // tiempo abierta antes de cerrarse sola
const unsigned long AZUL_MS = 10000;            // luz azul tras el ultimo movimiento
const unsigned long CALENTAR_MS = 30000;        // PIR calibrandose al encender

Servo servoE, servoQ;
ArduinoLEDMatrix matriz;
LiquidCrystal_I2C *lcd = 0;
byte lcdDir = 0;

// ---------- estado ----------
bool sistema = true, alerta = false, enUso = false;
byte origen = 0;
bool pir = false;
unsigned long azulHasta = 0;
bool entrada = false, puertaQ = false;
unsigned long entradaHasta = 0, puertaQHasta = 0;
bool hab1 = true, hab2 = true, amarilla = true;
int distEntrada = 20;      // cm: mas cerca = alguien en la entrada
bool autoEntrada = true;   // abre sola con el ultrasonico
int giroMs = 400;          // ms que tarda en girar 90 grados (se ajusta con GIRO:n)
int angE = 90;             // grados que abre la entrada (ANG_E:n, 10-180)
int angQ = 90;             // grados que abre el quirofano (ANG_Q:n, 10-180)
long objE = 0;             // hasta donde debe girar la entrada
bool invertir = false;     // INVERTIR si abre hacia el lado contrario
long posE = 0;             // cuanto ha girado la entrada (0 = cerrada, giroMs = 90 grados)
int dirE = 0;              // 1 abriendo, -1 cerrando, 0 quieto
unsigned long tGiro = 0;
int dist = DIST_MAX;
bool persona = false;
unsigned long pitidoHasta = 0;
String linUsb, linBt, lcd1, lcd2;

// ---------- matriz de la placa (8 x 12) ----------
const char *F_CRUZ[8] = {"000000000000","000001100000","000001100000","000111111000","000111111000","000001100000","000001100000","000000000000"};
const char *F_BIEN[8] = {"000000000000","000000000010","000000000110","000000001100","011000011000","001100110000","000111100000","000011000000"};
const char *F_ALER[8] = {"000001100000","000001100000","000001100000","000001100000","000001100000","000000000000","000001100000","000001100000"};
const char *F_OJO[8]  = {"000000000000","000011110000","001100001100","010001100010","010001100010","001100001100","000011110000","000000000000"};
const char *F_NADA[8] = {"000000000000","000000000000","000000000000","000000000000","000000000000","000000000000","000000000000","000000000000"};
const char **frameActual = 0;
void dibujar(const char **f) {
  if (f == frameActual) return;
  frameActual = f;
  uint8_t fr[8][12];
  for (byte r = 0; r < 8; r++) for (byte c = 0; c < 12; c++) fr[r][c] = f[r][c] == '1';
  matriz.renderBitmap(fr, 8, 12);
}

// ---------- envio ----------
void enviar(const String &s) { Serial.println(s); BT.println(s); }
void evento(const String &s) { enviar("EV," + s); }
long segRestantes(bool abierta, unsigned long hasta) {
  if (!abierta) return 0;
  long r = (long)(hasta - millis());
  return r > 0 ? (r + 999) / 1000 : 0;
}
bool azul() { return millis() < azulHasta; }
bool calentando() { return millis() < CALENTAR_MS; }

void enviarEstado() {
  String s = "ST,";
  s += sistema; s += ','; s += alerta; s += ','; s += origen; s += ',';
  s += pir; s += ','; s += azul(); s += ',';
  s += entrada; s += ','; s += puertaQ; s += ',';
  s += segRestantes(entrada, entradaHasta); s += ','; s += segRestantes(puertaQ, puertaQHasta); s += ',';
  s += hab1; s += ','; s += hab2; s += ','; s += amarilla; s += ','; s += enUso; s += ',';
  s += (lcd != 0); s += ','; s += calentando(); s += ',';
  s += dist; s += ','; s += persona; s += ','; s += distEntrada; s += ','; s += autoEntrada; s += ',';
  s += angE; s += ','; s += angQ; s += ','; s += giroMs; s += ','; s += invertir;
  enviar(s);
}

// ---------- memoria ----------
const byte FIRMA = 0xC2;
void guardar() {
  EEPROM.update(0, FIRMA);
  EEPROM.update(1, hab1);
  EEPROM.update(2, hab2);
  EEPROM.update(3, amarilla);
  EEPROM.update(4, sistema);
  EEPROM.update(5, distEntrada);
  EEPROM.update(6, autoEntrada);
  EEPROM.update(7, giroMs / 10);
  EEPROM.update(8, invertir);
  EEPROM.update(9, angE);
  EEPROM.update(10, angQ);
}
void cargar() {
  byte f = EEPROM.read(0);
  if (f != 0xC1 && f != FIRMA) return;   // primera vez: valores de fabrica
  hab1 = EEPROM.read(1) == 1;
  hab2 = EEPROM.read(2) == 1;
  amarilla = EEPROM.read(3) == 1;
  sistema = EEPROM.read(4) == 1;
  if (f == FIRMA) { distEntrada = constrain(EEPROM.read(5), 5, 150); autoEntrada = EEPROM.read(6) == 1; }
  byte g = EEPROM.read(7);
  if (f == FIRMA && g >= 5 && g <= 200) {
    giroMs = g * 10; invertir = EEPROM.read(8) == 1;
    byte a = EEPROM.read(9), b = EEPROM.read(10);
    if (a >= 10 && a <= 180) angE = a;
    if (b >= 10 && b <= 180) angQ = b;
  }
}

// ---------- ultrasonico ----------
int medir() {
  digitalWrite(P_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(P_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(P_TRIG, LOW);
  unsigned long us = pulseIn(P_ECHO, HIGH, 20000);   // 20 ms = unos 3 m
  if (!us) return DIST_MAX;
  return min((int)(us / 58), DIST_MAX);
}

// ---------- puertas ----------
// servo de 360: gira hacia un lado hasta completar giroMs y se suelta (detach) para que no siga
void girarEntrada(int d) {
  dirE = d;
  tGiro = millis();
  if (!servoE.attached()) servoE.attach(P_SERVO_E);
  int v = (d > 0) != invertir ? PARADO + VELOC : PARADO - VELOC;
  servoE.write(v);
}
void pararEntrada() {
  dirE = 0;
  servoE.write(PARADO);
  servoE.detach();
  digitalWrite(P_SERVO_E, LOW);
}
long msAbierta() { return (long)giroMs * angE / 90; }
void irEntrada(long obj) {   // gira hasta la posicion obj (en ms de giro)
  objE = obj;
  if (obj == posE) { if (dirE) pararEntrada(); return; }
  int d = obj > posE ? 1 : -1;
  if (d != dirE) girarEntrada(d);
}
void tickEntrada() {
  if (!dirE) return;
  unsigned long ahora = millis();
  posE += (long)(ahora - tGiro) * dirE;
  tGiro = ahora;
  if ((dirE > 0 && posE >= objE) || (dirE < 0 && posE <= objE)) { posE = objE; pararEntrada(); }
}
void moverEntrada(bool abrir) {
  if (abrir) entradaHasta = millis() + PUERTA_MS;
  if (abrir == entrada) return;
  entrada = abrir;
  irEntrada(abrir ? msAbierta() : 0);
  if (abrir) pitidoHasta = millis() + 120;
  evento(abrir ? "Puerta de entrada abierta" : "Puerta de entrada cerrada");
}
void moverQuirofano(bool abrir) {
  if (abrir && alerta) { evento("Alarma activa: la puerta del quirofano no se abre"); return; }
  if (abrir) puertaQHasta = millis() + PUERTA_MS;
  if (abrir == puertaQ) return;
  puertaQ = abrir;
  servoQ.write(abrir ? angQ : CERRADA);
  if (abrir) pitidoHasta = millis() + 120;
  evento(abrir ? "Puerta del quirofano abierta" : "Puerta del quirofano cerrada");
}

// ---------- alarma ----------
void alertar(byte o, const String &texto) {
  if (alerta) return;
  alerta = true; origen = o;
  moverQuirofano(false);
  evento(texto);
  enviarEstado();
}
void silenciar(const char *como) {
  if (!alerta) return;
  alerta = false; origen = 0;
  evento(como);
}

// ---------- LCD ----------
void pantalla() {
  String l1, l2;
  if (alerta) {
    l1 = "!! ALERTA !!";
    l2 = origen == 1 ? "Intruso quirofan" : "Alerta manual";
  } else if (entrada || puertaQ) {
    l1 = entrada && puertaQ ? "2 PUERTAS ABIERT" : entrada ? "ENTRADA ABIERTA" : "QUIROFANO ABIERT";
    long s = max(segRestantes(entrada, entradaHasta), segRestantes(puertaQ, puertaQHasta));
    l2 = "Cierra en " + String(s) + " s";
  } else if (calentando()) {
    l1 = "INFINITY TEC";
    l2 = "Prep. sensor " + String((CALENTAR_MS - millis()) / 1000 + 1) + "s";
  } else {
    l1 = sistema ? "VIGILANDO" : "VIGILANCIA OFF";
    if (azul()) l2 = enUso ? "Quirof: operando" : "Movim. quirofano";
    else l2 = enUso ? "Quirofano en uso" : "Todo en orden";
  }
  while (l1.length() < 16) l1 += ' ';
  while (l2.length() < 16) l2 += ' ';
  l1 = l1.substring(0, 16); l2 = l2.substring(0, 16);
  if (l1 == lcd1 && l2 == lcd2) return;
  lcd1 = l1; lcd2 = l2;
  if (lcd) { lcd->setCursor(0, 0); lcd->print(l1); lcd->setCursor(0, 1); lcd->print(l2); }
  String a = l1, b = l2; a.trim(); b.trim();
  enviar("LC," + a + "," + b);
}

// ---------- ordenes ----------
bool valor(const String &s, int pos) { return s.substring(pos).toInt() == 1; }
void orden(String s) {
  s.trim(); s.toUpperCase();
  if (s == "ON") { sistema = true; guardar(); evento("Vigilancia prendida"); }
  else if (s == "OFF") { sistema = false; silenciar("Alarma apagada"); guardar(); evento("Vigilancia apagada"); }
  else if (s == "ALERTA") alertar(3, "ALERTA MANUAL desde la app");
  else if (s == "SILENCIAR") silenciar("Alarma silenciada desde la app");
  else if (s == "ESTADO") {
    if (!lcd) buscarLcd();   // por si se conecto despues de prender
    if (lcd) { lcd->clear(); lcd->backlight(); lcd1 = ""; evento("Pantalla LCD en 0x" + String(lcdDir, HEX) + ": si no ves letras, gira el tornillo azul de atras"); }
    else evento("No se encuentra la pantalla LCD: revisa SDA, SCL, 5V y GND");
  }
  else if (s.startsWith("ENTRADA:")) moverEntrada(valor(s, 8));
  else if (s == "ABRIR") moverEntrada(true);
  else if (s.startsWith("DIST:")) { distEntrada = constrain(s.substring(5).toInt(), 5, 150); guardar(); evento("La entrada se abre a menos de " + String(distEntrada) + " cm"); }
  else if (s.startsWith("GIRO:")) {
    giroMs = constrain(s.substring(5).toInt(), 50, 2000) / 10 * 10; guardar();   // posE no cambia: al cerrar deshace lo que giro de verdad
    evento("Calibrado: 90 grados = " + String(giroMs) + " ms de giro");
  }
  else if (s.startsWith("ANG_E:")) {
    angE = constrain(s.substring(6).toInt(), 10, 180); guardar();
    if (entrada) irEntrada(msAbierta());
    evento("La entrada abre " + String(angE) + " grados");
  }
  else if (s.startsWith("ANG_Q:")) {
    angQ = constrain(s.substring(6).toInt(), 10, 180); guardar();
    if (puertaQ) servoQ.write(angQ);
    evento("El quirofano abre " + String(angQ) + " grados");
  }
  else if (s == "INVERTIR") { if (dirE) pararEntrada(); invertir = !invertir; guardar(); evento(invertir ? "Entrada: giro invertido" : "Entrada: giro normal"); }
  else if (s.startsWith("AUTO:")) { autoEntrada = valor(s, 5); guardar(); evento(autoEntrada ? "Entrada automatica prendida" : "Entrada automatica apagada"); }
  else if (s.startsWith("PUERTA_Q:")) moverQuirofano(valor(s, 9));
  else if (s.startsWith("USO:")) { enUso = valor(s, 4); evento(enUso ? "Quirofano en uso (sin alarma del PIR)" : "Quirofano libre"); }
  else if (s.startsWith("HAB:")) {
    int n = s.substring(4, 5).toInt();
    bool v = valor(s, 6);
    if (n == 0 || n == 1) hab1 = v;
    if (n == 0 || n == 2) hab2 = v;
    guardar();
  }
  else if (s.startsWith("AMARILLA:")) { amarilla = valor(s, 9); guardar(); }
  else { evento("Orden desconocida: " + s); return; }
  enviarEstado();
}
void leer(Stream &p, String &lin) {
  while (p.available()) {
    char c = p.read();
    if (c == '\n' || c == '\r') { if (lin.length()) orden(lin); lin = ""; }
    else if (lin.length() < 40) lin += c;
  }
}

// La pantalla se busca en todas las direcciones donde suelen venir (0x20-0x27 y 0x38-0x3F)
void buscarLcd() {
  lcdDir = 0;
  for (byte d = 0x20; d <= 0x3F && !lcdDir; d++) {
    if (d > 0x27 && d < 0x38) continue;
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) lcdDir = d;
  }
  if (!lcdDir) return;
  lcd = new LiquidCrystal_I2C(lcdDir, 16, 2);
  delay(50);
  lcd->init(); lcd->init();   // algunas pantallas necesitan dos veces
  lcd->backlight();
  lcd->clear();
  lcd->setCursor(0, 0); lcd->print("  INFINITY TEC");
  lcd->setCursor(0, 1); lcd->print("Hospital intelig");
}

// ================= setup / loop =================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(P_BUZ, OUTPUT); pinMode(P_VERDE, OUTPUT); pinMode(P_ROJO, OUTPUT); pinMode(P_AMAR, OUTPUT);
  pinMode(P_AZUL, OUTPUT); pinMode(P_HAB1, OUTPUT); pinMode(P_HAB2, OUTPUT);
  pinMode(P_PIR, INPUT);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT); digitalWrite(P_TRIG, LOW);
  cargar();
  pinMode(P_SERVO_E, OUTPUT); digitalWrite(P_SERVO_E, LOW);   // el de 360 queda suelto y quieto: se supone cerrado
  servoQ.attach(P_SERVO_Q); servoQ.write(CERRADA);
  matriz.begin();
  dibujar(F_CRUZ);
  Wire.begin();
  buscarLcd();
  // prueba de luces al encender
  const byte leds[] = {P_VERDE, P_ROJO, P_AMAR, P_AZUL, P_HAB1, P_HAB2};
  for (byte p : leds) digitalWrite(p, HIGH);
  digitalWrite(P_BUZ, HIGH); delay(150); digitalWrite(P_BUZ, LOW); delay(850);
  for (byte p : leds) digitalWrite(p, LOW);
  evento("Hospital inteligente listo");
  if (!lcd) evento("No se encuentra la pantalla LCD: revisa SDA, SCL, 5V y GND");
}

void loop() {
  unsigned long ahora = millis();
  leer(Serial, linUsb);
  leer(BT, linBt);

  // ---- PIR del quirofano ----
  bool p = digitalRead(P_PIR) == HIGH;
  if (p) azulHasta = ahora + AZUL_MS;
  if (p && !pir) {
    evento("Movimiento en el quirofano");
    if (sistema && !enUso && !calentando()) alertar(1, "ALARMA: alguien entro al quirofano");
  }
  pir = p;

  // ---- ultrasonico de la entrada (cada 100 ms) ----
  static unsigned long tUltra = 0;
  if (ahora - tUltra >= 100) {
    tUltra = ahora;
    dist = medir();
    bool pr = dist < distEntrada;
    if (pr && !persona) evento("Alguien llega a la entrada (" + String(dist) + " cm)");
    persona = pr;
    if (persona && autoEntrada) moverEntrada(true);   // abre y mantiene abierta mientras haya alguien
  }

  // ---- puertas que se cierran solas ----
  tickEntrada();
  if (entrada && (long)(ahora - entradaHasta) >= 0) moverEntrada(false);
  if (puertaQ && (long)(ahora - puertaQHasta) >= 0) moverQuirofano(false);

  // ---- luces, buzzer, matriz y LCD ----
  bool parpadeo = (ahora / 250) % 2;
  digitalWrite(P_AZUL, azul());
  digitalWrite(P_ROJO, alerta && parpadeo);
  digitalWrite(P_VERDE, (entrada || puertaQ) && !alerta);
  digitalWrite(P_HAB1, hab1 || alerta);
  digitalWrite(P_HAB2, hab2 || alerta);
  digitalWrite(P_AMAR, amarilla || alerta);
  digitalWrite(P_BUZ, (alerta && (ahora / 400) % 2) || ahora < pitidoHasta);

  if (alerta) dibujar(parpadeo ? F_ALER : F_NADA);
  else if (entrada || puertaQ) dibujar(F_BIEN);
  else if (azul()) dibujar(F_OJO);
  else dibujar(sistema ? F_CRUZ : F_NADA);

  static unsigned long tSt = 0, tLcd = 0;
  if (ahora - tLcd >= 250) { tLcd = ahora; pantalla(); }
  if (ahora - tSt >= 500) { tSt = ahora; enviarEstado(); }
}
