/*
  INFINITY TEC - Sistema Inteligente de Seguridad Hospitalaria
  Control y avisos por Bluetooth (modulo HC-05 / HC-06) desde el simulador web.

  Placa: Arduino UNO (R3 o R4). No necesita librerias extra.

  CONEXIONES
    HC-05  TXD -> D10        RXD -> D11 (con divisor 1k/2k: el RXD del HC-05 es de 3.3 V)
           VCC -> 5V         GND -> GND
    Sensor PIR 1 (Farmacia)   OUT -> D2   VCC 5V, GND
    Sensor PIR 2 (Quirofano)  OUT -> D3   VCC 5V, GND
    Boton de panico           D4 -> boton -> GND   (usa la resistencia interna)
    LED verde  (sistema encendido)  D7 -> 220 ohm -> GND
    LED rojo   (alerta)             D6 -> 220 ohm -> GND
    Buzzer activo                   D8 -> buzzer (+), buzzer (-) -> GND

  ORDENES QUE RECIBE (una por linea):  ON  OFF  ALERTA  SILENCIAR  ESTADO
  ENVIA cada segundo:  ST,<sistema>,<alerta>,<pir1>,<pir2>,<origen>
     origen: 0 ninguno, 1 farmacia, 2 quirofano, 3 manual (app), 4 boton de panico
  Y avisos sueltos:    EV,<texto>
*/

#include <SoftwareSerial.h>

SoftwareSerial bt(10, 11);   // RX, TX

const int PIN_PIR1 = 2;
const int PIN_PIR2 = 3;
const int PIN_BOTON = 4;
const int PIN_LED_ROJO = 6;
const int PIN_LED_VERDE = 7;
const int PIN_BUZZER = 8;

bool sistema = true;    // vigilancia encendida
bool alerta = false;
int origen = 0;
unsigned long ultimoEnvio = 0, ultimoBeep = 0;
bool beep = false;
String linea = "";

void enviar(const String &s) { bt.println(s); Serial.println(s); }

void enviarEstado() {
  String s = "ST,";
  s += sistema ? "1" : "0"; s += ",";
  s += alerta ? "1" : "0"; s += ",";
  s += digitalRead(PIN_PIR1) == HIGH ? "1" : "0"; s += ",";
  s += digitalRead(PIN_PIR2) == HIGH ? "1" : "0"; s += ",";
  s += String(origen);
  enviar(s);
}

void activarAlerta(int o, const char *texto) {
  if (alerta) return;
  alerta = true;
  origen = o;
  enviar(String("EV,") + texto);
  enviarEstado();
}

void silenciar() {
  alerta = false;
  origen = 0;
  digitalWrite(PIN_BUZZER, LOW);
  enviar("EV,Alerta silenciada");
}

void orden(String c) {
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return;
  if (c == "ON")             { sistema = true;  enviar("EV,Sistema encendido"); }
  else if (c == "OFF")       { sistema = false; silenciar(); enviar("EV,Sistema apagado"); }
  else if (c == "ALERTA")    { activarAlerta(3, "ALERTA MANUAL desde la app"); }
  else if (c == "SILENCIAR") { silenciar(); }
  else if (c == "ESTADO")    { }
  else                       { enviar("EV,Orden desconocida: " + c); }
  enviarEstado();
}

void setup() {
  Serial.begin(9600);
  bt.begin(9600);            // velocidad de fabrica del HC-05
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  enviar("EV,Sistema de seguridad listo");
}

void loop() {
  // 1) Ordenes por Bluetooth (y por el monitor serie, para probar)
  while (bt.available()) {
    char ch = bt.read();
    if (ch == '\n' || ch == '\r') { orden(linea); linea = ""; }
    else if (linea.length() < 20) linea += ch;
  }
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { orden(linea); linea = ""; }
    else if (linea.length() < 20) linea += ch;
  }

  // 2) Sensores (solo con el sistema encendido)
  if (sistema) {
    if (digitalRead(PIN_PIR1) == HIGH) activarAlerta(1, "Movimiento en FARMACIA (zona restringida)");
    if (digitalRead(PIN_PIR2) == HIGH) activarAlerta(2, "Movimiento en QUIROFANO (zona restringida)");
  }
  // El boton de panico funciona siempre, aunque el sistema este apagado
  if (digitalRead(PIN_BOTON) == LOW) activarAlerta(4, "Boton de panico presionado");

  // 3) Luces y sirena
  digitalWrite(PIN_LED_VERDE, sistema);
  unsigned long ahora = millis();
  if (alerta) {
    if (ahora - ultimoBeep >= 250) { ultimoBeep = ahora; beep = !beep; }
    digitalWrite(PIN_BUZZER, beep);
    digitalWrite(PIN_LED_ROJO, beep);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
  }

  // 4) Latido: la app sabe que el equipo esta vivo
  if (ahora - ultimoEnvio >= 1000) { ultimoEnvio = ahora; enviarEstado(); }
}
