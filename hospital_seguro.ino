/*
  INFINITY TEC - Hospital Inteligente (Sistema de Seguridad Hospitalaria)
  Control y avisos por Bluetooth (HC-05) desde la pagina web.

  Placa: Arduino UNO R4 WiFi
  Librerias (Gestor de librerias): "MFRC522" (GithubCommunity) y "LiquidCrystal I2C" (Frank de Brabander).
            Las demas (SPI, Wire, EEPROM, Servo, Arduino_LED_Matrix) ya vienen con la placa.

  CONEXIONES
    Bluetooth HC-05   VCC 5V, GND, TXD -> D0 (RX)
                      RXD <- D1 (TX) con divisor: D1 -> 1k -> RXD, y 2k de RXD a GND
    Lector RFID RC522 (va a 3.3V, NO a 5V)
                      3.3V, GND, SDA D10, MOSI D11, MISO D12, SCK D13, RST -> 3.3V (IRQ sin conectar)
    Ultrasonico HC-SR04 (puerta de Farmacia y pasillo)   VCC 5V, GND, TRIG D6, ECHO D7
    Pantalla LCD 16x2 I2C   VCC 5V, GND, SDA -> SDA (A4), SCL -> SCL (A5)   (direccion 0x27 o 0x3F, se busca sola)
    Servo SG90 (puerta de Farmacia)   naranja -> A2, rojo 5V, cafe GND
    Boton de llamada (Habitacion 1)   una pata a D9, la otra a GND
    Luces de los 8 cuartos: modulo PCF8574 (direccion 0x20) en los MISMOS cables SDA/SCL que la LCD, VCC 5V, GND.
                      Cada LED: 5V -> 1k -> pata larga del LED; pata corta -> P0..P7 del PCF8574
                      P0 Hab 1, P1 Hab 2, P2 Hab 3, P3 Farmacia, P4 Quirofano, P5 Consultorio, P6 Deposito, P7 Recepcion
    LED verde    (acceso permitido)   D3 -> 470 ohm -> GND
    LED rojo     (alarma)             D4 -> 470 ohm -> GND
    2 LEDs amarillos (pasillo, noche) D5 -> 1k -> LED -> GND  (cada LED con su resistencia, los dos al D5)
    2 LEDs blancos   (pasillo, dia)   D8 -> 1k -> LED -> GND  (cada LED con su resistencia, los dos al D8)
    Buzzer activo                     + D2, - GND
    Fotoresistencia                   5V -> LDR -> A0, y 10k de A0 a GND  (lejos de los LEDs del pasillo)
    Sensor de temperatura LM35 (cara plana hacia ti): izquierda 5V, centro A1, derecha GND

  QUE HACE
    - RFID: tarjeta registrada = la puerta (servo) se abre 8 s, LED verde, pitido y "Hola, nombre" en la LCD.
            Tarjeta desconocida = LED rojo + 3 pitidos. 3 intentos en 1 minuto = ALARMA.
            De noche, si "solo jefes" esta activo, solo entran las tarjetas marcadas como jefe.
    - Ultrasonico: si alguien se queda frente a la puerta de Farmacia sin tarjeta 8 s = ALARMA.
            De noche, si alguien camina por el pasillo (< 100 cm), las luces blancas se prenden 10 s.
    - Pasillo: de dia 2 LEDs blancos; de noche 2 amarillos suaves. Con alarma, blancos siempre.
    - Temperatura: si pasa del limite = ALARMA (vuelve a avisar solo si baja y sube otra vez).
    - Boton de la Habitacion 1: llamada a la enfermera (pitido suave y aviso en la app). Se atiende
            desde la app o apretando otra vez el boton.
    - Luces de cuartos: se prenden y apagan desde la app. De noche se apagan solas las de las
            habitaciones, consultorio, deposito y farmacia (pacientes duermen); al amanecer vuelven.
            La de Farmacia se prende sola al abrir la puerta. Con alarma se prenden todas y parpadea
            la del cuarto con problema; con llamada parpadea la de la Habitacion 1.
    - LCD: temperatura, dia/noche, bienvenida, alarmas y llamadas. La matriz de la placa muestra el estado.

  ORDENES (una por linea, por Bluetooth o por el monitor serie a 9600):
    ON  OFF  ALERTA  SILENCIAR  ESTADO  ABRIR  ATENDER  LISTA
    REGISTRAR (la proxima tarjeta queda registrada; 15 s)  CANCELAR  BORRAR (borra todas)
    DIST:n (cm, 5-150)   LUZ:n (%, 5-90)   TMAX:n (grados, 20-60)   JEFES:1/0 (de noche solo jefes)
    CUARTO:n:0/1 (luz del cuarto n, 1-8; n=0 todos)
    TARJ:UID:jefe(0/1):NOMBRE   (pone nombre y rango a una tarjeta registrada; nombre hasta 10 letras)
  ENVIA cada 0,5 s:
    ST,sistema,alerta,origen,distancia,persona,espera_s,acceso,luz%,noche,temp*10,temp_ok,
       dist_puerta,luz_noche,temp_max,tarjetas,registrando,luces_blancas,llamada,solo_jefes,cuartos(bits: 1 Hab1, 2 Hab2, 4 Hab3, 8 Farmacia,
       16 Quirofano, 32 Consultorio, 64 Deposito, 128 Recepcion)
    origen: 0 ninguno, 1 persona sin tarjeta, 2 temperatura alta, 3 manual (app), 4 tarjeta no autorizada
  Y ademas:  EV,texto    TJ,uid,ok,motivo (0 desconocida, 1 permitida, 2 cerrada de noche, 3 registrada)
             LC,linea1,linea2 (lo que muestra la LCD)    TL,uid,jefe,nombre ... TL,FIN (lista de tarjetas)
*/

#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "Arduino_LED_Matrix.h"

#define BT Serial1   // HC-05 en D0/D1 (en el R4 el USB va aparte)

const byte P_BUZ = 2, P_VERDE = 3, P_ROJO = 4, P_AMAR = 5, P_TRIG = 6, P_ECHO = 7, P_BLANCA = 8;
const byte P_BOTON = 9, P_SS = 10;
const byte P_LDR = A0, P_TEMP = A1, P_SERVO = A2;
const int PASILLO_CM = 100;             // de noche, alguien a menos de esto = movimiento en el pasillo
const unsigned long BLANCA_MS = 10000;  // tiempo con luz blanca al detectar movimiento de noche
const byte SERVO_CERRADA = 0, SERVO_ABIERTA = 90;

MFRC522 rfid(P_SS, MFRC522::UNUSED_PIN);   // RST del lector va fijo a 3.3V
ArduinoLEDMatrix matriz;
Servo puerta;
LiquidCrystal_I2C *lcd = 0;

// ---------- ajustes (se guardan en la memoria) ----------
int distPuerta = 30;   // cm: mas cerca = hay alguien en la puerta
int luzNoche = 30;     // %: menos luz = noche
int tempMax = 30;      // grados: mas = alarma
bool soloJefes = true; // de noche solo entran los jefes
const byte MAX_TARJ = 10, MAX_NOM = 10;
byte nTarj = 0;
byte tarjLen[MAX_TARJ];
byte tarjUid[MAX_TARJ][7];
bool tarjJefe[MAX_TARJ];
char tarjNom[MAX_TARJ][MAX_NOM + 1];

const unsigned long ESPERA_MS = 8000;   // tiempo para pasar la tarjeta
const unsigned long ACCESO_MS = 8000;   // puerta abierta tras una tarjeta valida
const int DIST_MAX = 300;

// ---------- estado ----------
bool sistema = true, alerta = false;
byte origen = 0;
int dist = DIST_MAX;
bool persona = false;
unsigned long tVistaPersona = 0, esperaFin = 0, accesoHasta = 0;
byte negados = 0;
unsigned long tNegado = 0, negadoHasta = 0, pitidoHasta = 0;
bool registrando = false;
unsigned long regHasta = 0;
int luz = 100;
bool noche = false;
unsigned long blancaHasta = 0;
float temp = 0;
bool tempOk = false, avisoCalor = false;
bool llamada = false;
byte cuartos = 0xFF, cuartosDia = 0xFF;   // luces de los cuartos (1 bit cada uno)
byte pcf = 0;                            // direccion del PCF8574 (0 = no esta)
const char *CUARTO_NOM[8] = {"Habitacion 1", "Habitacion 2", "Habitacion 3", "Farmacia", "Quirofano 1", "Consultorio", "Deposito", "Recepcion"};
String linUsb, linBt;
String ultimaUid;
unsigned long tUltimaUid = 0;
byte lcdMotivo = 0; String lcdNombre; unsigned long lcdHasta = 0;   // mensaje de la ultima tarjeta
String lcd1, lcd2;

// ---------- matriz de la placa (8 x 12) ----------
const char *F_CRUZ[8] = {"000000000000","000001100000","000001100000","000111111000","000111111000","000001100000","000001100000","000000000000"};
const char *F_PREG[8] = {"000011110000","000110011000","000000011000","000000110000","000001100000","000001100000","000000000000","000001100000"};
const char *F_BIEN[8] = {"000000000000","000000000010","000000000110","000000001100","011000011000","001100110000","000111100000","000011000000"};
const char *F_MAL[8]  = {"000000000000","000110001100","000011011000","000001110000","000001110000","000011011000","000110001100","000000000000"};
const char *F_ALER[8] = {"000001100000","000001100000","000001100000","000001100000","000001100000","000000000000","000001100000","000001100000"};
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

bool lucesBlancas(unsigned long ahora) { return !noche || ahora < blancaHasta || alerta; }

void enviarEstado() {
  unsigned long ahora = millis();
  String s = "ST,";
  s += sistema; s += ','; s += alerta; s += ','; s += origen; s += ',';
  s += dist; s += ','; s += persona; s += ',';
  s += esperaFin ? (long)(esperaFin - ahora + 999) / 1000 : 0; s += ',';
  s += (ahora < accesoHasta); s += ',';
  s += luz; s += ','; s += noche; s += ',';
  s += (int)round(temp * 10); s += ','; s += tempOk; s += ',';
  s += distPuerta; s += ','; s += luzNoche; s += ','; s += tempMax; s += ',';
  s += nTarj; s += ','; s += registrando; s += ',';
  s += lucesBlancas(ahora); s += ','; s += llamada; s += ','; s += soloJefes; s += ','; s += cuartos;
  enviar(s);
}

// ---------- memoria ----------
const byte FIRMA = 0xB4;
void guardar() {
  EEPROM.update(0, FIRMA);
  EEPROM.update(1, distPuerta);
  EEPROM.update(2, luzNoche);
  EEPROM.update(3, tempMax);
  EEPROM.update(4, nTarj);
  EEPROM.update(5, soloJefes);
  EEPROM.update(6, cuartos);
  for (byte i = 0; i < MAX_TARJ; i++) {
    EEPROM.update(10 + i * 8, tarjLen[i]);
    for (byte j = 0; j < 7; j++) EEPROM.update(11 + i * 8 + j, tarjUid[i][j]);
    EEPROM.update(100 + i * 12, tarjJefe[i]);
    for (byte j = 0; j <= MAX_NOM; j++) EEPROM.update(101 + i * 12 + j, tarjNom[i][j]);
  }
}
void cargar() {
  for (byte i = 0; i < MAX_TARJ; i++) { tarjJefe[i] = false; tarjNom[i][0] = 0; }
  byte f = EEPROM.read(0);
  if (f != 0xB3 && f != FIRMA) return;   // primera vez: valores de fabrica
  distPuerta = constrain(EEPROM.read(1), 5, 150);
  luzNoche = constrain(EEPROM.read(2), 5, 90);
  tempMax = constrain(EEPROM.read(3), 20, 60);
  nTarj = min((int)EEPROM.read(4), (int)MAX_TARJ);
  for (byte i = 0; i < MAX_TARJ; i++) {
    tarjLen[i] = min((int)EEPROM.read(10 + i * 8), 7);
    for (byte j = 0; j < 7; j++) tarjUid[i][j] = EEPROM.read(11 + i * 8 + j);
  }
  if (f == FIRMA) {   // la version anterior (0xB3) no tenia nombres
    soloJefes = EEPROM.read(5) == 1;
    cuartos = cuartosDia = EEPROM.read(6);
    for (byte i = 0; i < MAX_TARJ; i++) {
      tarjJefe[i] = EEPROM.read(100 + i * 12) == 1;
      for (byte j = 0; j <= MAX_NOM; j++) tarjNom[i][j] = EEPROM.read(101 + i * 12 + j);
      tarjNom[i][MAX_NOM] = 0;
      for (byte j = 0; tarjNom[i][j]; j++) if (tarjNom[i][j] < 32 || tarjNom[i][j] > 126) { tarjNom[i][0] = 0; break; }
    }
  }
}

// ---------- alarma ----------
void alertar(byte o, const String &texto) {
  if (alerta) return;
  alerta = true; origen = o;
  evento(texto);
  enviarEstado();
}
void silenciar(const char *como) {
  if (!alerta) return;
  alerta = false; origen = 0;
  evento(como);
}

// ---------- tarjetas ----------
String uidTexto(byte *u, byte n) {
  String s;
  for (byte i = 0; i < n; i++) { if (u[i] < 16) s += '0'; s += String(u[i], HEX); }
  s.toUpperCase();
  return s;
}
int buscarTarjeta(byte *u, byte n) {
  for (byte i = 0; i < nTarj; i++) {
    if (tarjLen[i] != n) continue;
    bool igual = true;
    for (byte j = 0; j < n; j++) if (tarjUid[i][j] != u[j]) { igual = false; break; }
    if (igual) return i;
  }
  return -1;
}
int buscarTexto(const String &id) {
  for (byte i = 0; i < nTarj; i++) if (uidTexto(tarjUid[i], tarjLen[i]) == id) return i;
  return -1;
}
String nombreDe(int i) { return tarjNom[i][0] ? String(tarjNom[i]) : uidTexto(tarjUid[i], tarjLen[i]); }
void avisoLcd(byte motivo, const String &nombre) { lcdMotivo = motivo; lcdNombre = nombre; lcdHasta = millis() + 3000; }

void tarjetaLeida(byte *u, byte n) {
  n = min(n, (byte)7);
  String id = uidTexto(u, n);
  unsigned long ahora = millis();
  if (id == ultimaUid && ahora - tUltimaUid < 1500) { tUltimaUid = ahora; return; }  // la misma tarjeta sigue encima
  ultimaUid = id; tUltimaUid = ahora;

  if (registrando) {
    registrando = false;
    if (buscarTarjeta(u, n) >= 0) evento("La tarjeta " + id + " ya estaba registrada");
    else if (nTarj >= MAX_TARJ) evento("Memoria llena: ya hay 10 tarjetas");
    else {
      tarjLen[nTarj] = n;
      for (byte j = 0; j < 7; j++) tarjUid[nTarj][j] = j < n ? u[j] : 0;
      tarjJefe[nTarj] = false; tarjNom[nTarj][0] = 0;
      nTarj++; guardar();
      evento("Tarjeta " + id + " registrada");
    }
    pitidoHasta = ahora + 150;
    avisoLcd(3, id);
    enviar("TJ," + id + ",1,3");
    return;
  }
  int i = buscarTarjeta(u, n);
  if (i >= 0 && noche && soloJefes && !tarjJefe[i]) {
    negadoHasta = ahora + 1500;
    avisoLcd(2, nombreDe(i));
    enviar("TJ," + id + ",0,2");
    evento("Farmacia cerrada de noche: " + nombreDe(i) + " no es jefe");
    return;
  }
  enviar("TJ," + id + "," + String(i >= 0) + "," + String(i >= 0));
  if (i >= 0) {
    accesoHasta = ahora + ACCESO_MS; esperaFin = 0; negados = 0;
    pitidoHasta = ahora + 150;
    avisoLcd(1, nombreDe(i));
    evento("Acceso permitido a Farmacia: " + nombreDe(i));
    if (alerta && (origen == 1 || origen == 4)) silenciar("Alarma desactivada con una tarjeta autorizada");
  } else {
    if (ahora - tNegado > 60000UL) negados = 0;
    tNegado = ahora; negados++;
    negadoHasta = ahora + 1500;
    avisoLcd(0, id);
    evento("Tarjeta NO autorizada: " + id + " (intento " + String(negados) + ")");
    if (negados >= 3 && sistema) alertar(4, "ALARMA: 3 intentos con tarjeta no autorizada");
  }
}
void listaTarjetas() {
  for (byte i = 0; i < nTarj; i++) enviar("TL," + uidTexto(tarjUid[i], tarjLen[i]) + "," + String(tarjJefe[i]) + "," + String(tarjNom[i]));
  enviar("TL,FIN");
}

// ---------- sensores ----------
int medir() {
  digitalWrite(P_TRIG, LOW); delayMicroseconds(3);
  digitalWrite(P_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(P_TRIG, LOW);
  unsigned long us = pulseIn(P_ECHO, HIGH, 18000UL);
  if (us == 0) return DIST_MAX;
  return min((long)(us / 58), (long)DIST_MAX);
}
void leerTemp() {
  long suma = 0;
  for (byte i = 0; i < 16; i++) suma += analogRead(P_TEMP);
  float mv = (suma / 16.0) * 5000.0 / 16383.0;   // 14 bits, 5 V
  float t = mv / 10.0;                           // LM35: 10 mV por grado
  tempOk = t > 1 && t < 70;                      // sin sensor (pata al aire) o mal conectado da valores raros
  temp = tempOk ? (temp == 0 ? t : temp * 0.7 + t * 0.3) : t;
}

// ---------- pantalla LCD ----------
String a16(String s) { s.replace(",", " "); if (s.length() > 16) s = s.substring(0, 16); while (s.length() < 16) s += ' '; return s; }
void pantalla(unsigned long ahora) {
  static const char *ORI[5] = {"", "Intruso Farmacia", "Quirofano calor", "Alerta manual", "Tarjeta falsa x3"};
  String l1, l2;
  if (alerta) { l1 = (ahora / 500) % 2 ? "!!! ALARMA !!!" : ""; l2 = ORI[origen < 5 ? origen : 0]; }
  else if (ahora < lcdHasta) {
    if (lcdMotivo == 1)      { l1 = "Acceso OK"; l2 = "Hola, " + lcdNombre; }
    else if (lcdMotivo == 2) { l1 = "FARMACIA CERRADA"; l2 = "Noche: solo jefe"; }
    else if (lcdMotivo == 3) { l1 = "Tarjeta nueva"; l2 = lcdNombre + " OK"; }
    else                     { l1 = "ACCESO NEGADO"; l2 = "Tarjeta no reg."; }
  }
  else if (llamada)     { l1 = "LLAMADA ENFERM."; l2 = "Habitacion 1"; }
  else if (registrando) { l1 = "Pase la tarjeta"; l2 = "nueva por lector"; }
  else if (esperaFin)   { l1 = "Acerque tarjeta"; l2 = "Quedan " + String((long)(esperaFin - ahora + 999) / 1000) + " s"; }
  else {
    l1 = tempOk ? "Quirof. " + String(temp, 1) + " C" : "Temp: sin sensor";
    l2 = String(noche ? "Noche" : "Dia") + (sistema ? "" : " APAGADO");
  }
  l1 = a16(l1); l2 = a16(l2);
  if (l1 == lcd1 && l2 == lcd2) return;
  lcd1 = l1; lcd2 = l2;
  if (lcd) { lcd->setCursor(0, 0); lcd->print(l1); lcd->setCursor(0, 1); lcd->print(l2); }
  String t1 = l1, t2 = l2; t1.trim(); t2.trim();
  enviar("LC," + t1 + "," + t2);
}

// ---------- ordenes ----------
void orden(String s) {
  s.trim(); s.toUpperCase();
  if (s.length() == 0) return;
  int dos = s.indexOf(':');
  String cmd = dos < 0 ? s : s.substring(0, dos);
  int val = dos < 0 ? -1 : s.substring(dos + 1).toInt();

  if (cmd == "ON")             { sistema = true; evento("Sistema encendido"); }
  else if (cmd == "OFF")       { sistema = false; esperaFin = 0; silenciar("Alerta silenciada"); evento("Sistema apagado"); }
  else if (cmd == "ALERTA")    { alertar(3, "ALERTA MANUAL desde la app"); }
  else if (cmd == "SILENCIAR") { silenciar("Alerta silenciada desde la app"); negados = 0; }
  else if (cmd == "ABRIR")     { accesoHasta = millis() + ACCESO_MS; esperaFin = 0; pitidoHasta = millis() + 150; evento("Puerta de Farmacia abierta desde la app"); }
  else if (cmd == "REGISTRAR") { registrando = true; regHasta = millis() + 15000; evento("Acerque la tarjeta nueva al lector (15 s)"); }
  else if (cmd == "CANCELAR")  { if (registrando) { registrando = false; evento("Registro de tarjeta cancelado"); } }
  else if (cmd == "BORRAR")    { nTarj = 0; guardar(); evento("Se borraron todas las tarjetas"); }
  else if (cmd == "ATENDER")   { if (llamada) { llamada = false; evento("Llamada de la Habitacion 1 atendida"); } }
  else if (cmd == "LISTA")     { listaTarjetas(); }
  else if (cmd == "DIST" && val >= 0) { distPuerta = constrain(val, 5, 150); guardar(); evento("Distancia de la puerta = " + String(distPuerta) + " cm"); }
  else if (cmd == "LUZ" && val >= 0)  { luzNoche = constrain(val, 5, 90); guardar(); evento("Noche con menos de " + String(luzNoche) + " % de luz"); }
  else if (cmd == "TMAX" && val >= 0) { tempMax = constrain(val, 20, 60); avisoCalor = false; guardar(); evento("Temperatura maxima = " + String(tempMax) + " C"); }
  else if (cmd == "JEFES" && val >= 0) { soloJefes = val == 1; guardar(); evento(soloJefes ? "De noche la Farmacia es solo para jefes" : "De noche entran todas las tarjetas"); }
  else if (cmd == "CUARTO" && val >= 0 && val <= 8) {
    int b = s.indexOf(':', dos + 1);
    bool on = b > 0 && s.substring(b + 1).toInt() == 1;
    byte m = val == 0 ? 0xFF : (1 << (val - 1));
    cuartos = on ? (cuartos | m) : (cuartos & ~m);
    if (!noche) cuartosDia = cuartos;
    guardar();
    evento(String("Luz de ") + (val == 0 ? "todos los cuartos" : CUARTO_NOM[val - 1]) + (on ? " encendida" : " apagada"));
  }
  else if (cmd == "TARJ" && dos > 0) {
    // TARJ:UID:jefe:NOMBRE
    String r = s.substring(dos + 1);
    int a = r.indexOf(':'), b = a < 0 ? -1 : r.indexOf(':', a + 1);
    if (a < 0) return;
    String id = r.substring(0, a);
    int i = buscarTexto(id);
    if (i < 0) { evento("La tarjeta " + id + " no esta registrada"); return; }
    tarjJefe[i] = r.substring(a + 1, b < 0 ? r.length() : b).toInt() == 1;
    String nom = b < 0 ? "" : r.substring(b + 1);
    nom.trim();
    byte k = 0;
    for (unsigned int j = 0; j < nom.length() && k < MAX_NOM; j++) { char c = nom[j]; if (c >= 32 && c <= 126 && c != ',') tarjNom[i][k++] = c; }
    tarjNom[i][k] = 0;
    guardar();
    evento("Tarjeta " + id + " = " + nombreDe(i) + (tarjJefe[i] ? " (jefe)" : ""));
  }
  else if (cmd == "ESTADO") { lcd1 = ""; }
  else {
    // Solo se contesta a algo que parezca una orden (corta, letras/numeros/:). Asi el ruido o un
    // cable que devuelve lo que la placa envia no arma un bucle de "Orden desconocida".
    bool pareceOrden = s.length() <= 14 && !s.startsWith("EV,") && !s.startsWith("ST,") && !s.startsWith("TJ,");
    for (unsigned int i = 0; pareceOrden && i < s.length(); i++) {
      char c = s[i];
      if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ':')) pareceOrden = false;
    }
    if (pareceOrden) enviar("EV,Orden desconocida: " + s);
    return;
  }
  enviarEstado();
}
void leer(Stream &p, String &lin) {
  while (p.available()) {
    char c = p.read();
    if (c == '\n' || c == '\r') { if (lin.length()) orden(lin); lin = ""; }
    else if (lin.length() < 40) lin += c;
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(P_BUZ, OUTPUT); pinMode(P_VERDE, OUTPUT); pinMode(P_ROJO, OUTPUT);
  pinMode(P_AMAR, OUTPUT); pinMode(P_BLANCA, OUTPUT);
  pinMode(P_BOTON, INPUT_PULLUP);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT);
  digitalWrite(P_TRIG, LOW);
  analogReadResolution(14);
  cargar();
  SPI.begin();
  rfid.PCD_Init();
  matriz.begin();
  dibujar(F_CRUZ);
  puerta.attach(P_SERVO);
  puerta.write(SERVO_CERRADA);
  // pantalla: se busca en 0x27 y en 0x3F; luces de cuartos (PCF8574) en 0x20-0x26 o 0x38-0x3E
  Wire.begin();
  for (byte d = 0x20; d <= 0x3E && !pcf; d++) {
    if (d == 0x27 || (d > 0x27 && d < 0x38)) continue;
    Wire.beginTransmission(d); if (Wire.endTransmission() == 0) pcf = d;
  }
  byte dir = 0;
  for (byte d : {0x27, 0x3F}) { Wire.beginTransmission(d); if (Wire.endTransmission() == 0) { dir = d; break; } }
  if (dir) { lcd = new LiquidCrystal_I2C(dir, 16, 2); lcd->init(); lcd->backlight(); lcd->setCursor(0, 0); lcd->print("  INFINITY TEC"); lcd->setCursor(0, 1); lcd->print("Hospital intelig"); }
  // prueba de luces al encender
  digitalWrite(P_VERDE, HIGH); digitalWrite(P_ROJO, HIGH); digitalWrite(P_AMAR, HIGH); digitalWrite(P_BLANCA, HIGH); digitalWrite(P_BUZ, HIGH);
  delay(150); digitalWrite(P_BUZ, LOW); delay(850);
  digitalWrite(P_VERDE, LOW); digitalWrite(P_ROJO, LOW); digitalWrite(P_AMAR, LOW);
  leerTemp();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  evento("Hospital inteligente listo");
  evento(v == 0x00 || v == 0xFF ? "No se encuentra el lector RFID: revisa sus cables (3.3V)" : "Lector RFID listo (" + String(nTarj) + " tarjetas registradas)");
  if (!dir) evento("No se encuentra la pantalla LCD: revisa SDA, SCL y 5V");
  if (!pcf) evento("No se encuentra el modulo de luces de cuartos (PCF8574): revisa SDA, SCL y 5V");
  if (!tempOk) evento("El sensor de temperatura no da una lectura valida: revisa sus patas");
  enviarEstado();
}

void loop() {
  unsigned long ahora = millis();
  leer(Serial, linUsb);
  leer(BT, linBt);

  // ---- tarjeta RFID ----
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    tarjetaLeida(rfid.uid.uidByte, rfid.uid.size);
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
  if (registrando && ahora > regHasta) { registrando = false; evento("Registro de tarjeta cancelado (tiempo agotado)"); }

  // ---- boton de llamada de la Habitacion 1 ----
  static bool botonAntes = HIGH;
  static unsigned long tBoton = 0;
  bool boton = digitalRead(P_BOTON);
  if (boton != botonAntes && ahora - tBoton > 40) {
    tBoton = ahora; botonAntes = boton;
    if (boton == LOW) {
      llamada = !llamada;
      evento(llamada ? "LLAMADA: la Habitacion 1 pide una enfermera" : "Llamada de la Habitacion 1 cancelada desde la habitacion");
      enviarEstado();
    }
  }

  // ---- ultrasonico en la puerta de Farmacia y el pasillo ----
  static unsigned long tDist = 0;
  static int m[3] = {DIST_MAX, DIST_MAX, DIST_MAX};
  static byte im = 0;
  if (ahora - tDist >= 80) {
    tDist = ahora;
    m[im] = medir(); im = (im + 1) % 3;
    int a = m[0], b = m[1], c = m[2];
    dist = max(min(a, b), min(max(a, b), c));   // mediana: un eco falso no cuenta
    persona = dist < distPuerta;
    if (persona) tVistaPersona = ahora;
    if (noche && dist < PASILLO_CM) {
      if (ahora >= blancaHasta) evento("Movimiento en el pasillo de noche: luces blancas 10 s");
      blancaHasta = ahora + BLANCA_MS;
    }
    if (!sistema) esperaFin = 0;
    else if (persona && ahora > accesoHasta && !alerta && !esperaFin) {
      esperaFin = ahora + ESPERA_MS;
      evento("Persona en la puerta de Farmacia: acerque su tarjeta");
    }
    if (esperaFin && ahora - tVistaPersona > 2000) { esperaFin = 0; evento("La persona se alejo de la puerta"); }
    if (esperaFin && ahora >= esperaFin) { esperaFin = 0; alertar(1, "ALARMA: persona sin tarjeta en la puerta de Farmacia"); }
  }

  // ---- luz y temperatura ----
  static unsigned long tLuz = 0;
  if (ahora - tLuz >= 250) {
    tLuz = ahora;
    luz = (long)analogRead(P_LDR) * 100 / 16383;
    if (!noche && luz < luzNoche) {
      noche = true; blancaHasta = 0; evento("Anochece: pasillo en modo noche (luces amarillas)");
      cuartosDia = cuartos; cuartos &= 0x90;   // quedan Quirofano y Recepcion
      evento("Noche: se apagan las luces de habitaciones, consultorio, deposito y farmacia");
    }
    else if (noche && luz > luzNoche + 5) {
      noche = false; evento("Amanece: pasillo con luces blancas");
      cuartos = cuartosDia; evento("Dia: las luces de los cuartos vuelven como estaban");
    }
  }
  static unsigned long tTemp = 0;
  if (ahora - tTemp >= 500) {
    tTemp = ahora;
    leerTemp();
    if (tempOk) {
      if (temp >= tempMax && !avisoCalor) {
        avisoCalor = true;
        if (sistema) alertar(2, "ALARMA: temperatura alta (" + String(temp, 1) + " C)");
        else evento("Temperatura alta (" + String(temp, 1) + " C), pero el sistema esta apagado");
      } else if (temp < tempMax - 1) avisoCalor = false;
    }
  }

  // ---- luces, buzzer, servo, matriz y LCD ----
  bool parpadeo = (ahora / 250) % 2;
  bool acceso = ahora < accesoHasta;
  bool negado = ahora < negadoHasta;
  digitalWrite(P_VERDE, acceso || (sistema && !alerta && (ahora % 2000) < 80));   // latido cada 2 s
  digitalWrite(P_ROJO, alerta ? parpadeo : negado || (esperaFin && (ahora / 500) % 2));
  digitalWrite(P_AMAR, noche);
  digitalWrite(P_BLANCA, lucesBlancas(ahora));
  // luces de cuartos
  byte ef = cuartos;
  if (acceso) ef |= 8;                         // Farmacia se ilumina al abrir la puerta
  if (llamada) ef = (ef & ~1) | ((ahora / 500) % 2);
  if (alerta) {
    ef = 0xFF;
    int b = (origen == 1 || origen == 4) ? 3 : origen == 2 ? 4 : -1;
    if (b >= 0 && !parpadeo) ef &= ~(1 << b);
  }
  static int efAntes = -1;
  if (pcf && ef != efAntes) { efAntes = ef; Wire.beginTransmission(pcf); Wire.write((byte)~ef); Wire.endTransmission(); }   // LED prende con 0
  static bool abierta = false;
  if (acceso != abierta) { abierta = acceso; puerta.write(abierta ? SERVO_ABIERTA : SERVO_CERRADA); }
  bool buz = false;
  unsigned long f3 = ahora % 3000;
  if (alerta) buz = parpadeo;
  else if (negado) buz = ((negadoHasta - ahora) / 120) % 2;                     // 3 pitidos
  else if (esperaFin) buz = (ahora % 1000) < 50;                                 // aviso cada segundo
  else if (llamada) buz = f3 < 70 || (f3 > 200 && f3 < 270);                    // ding-ding cada 3 s
  if (ahora < pitidoHasta) buz = true;
  digitalWrite(P_BUZ, buz);

  if (alerta) dibujar(parpadeo ? F_ALER : F_NADA);
  else if (negado) dibujar(F_MAL);
  else if (acceso) dibujar(F_BIEN);
  else if (esperaFin || registrando) dibujar(F_PREG);
  else if (llamada) dibujar((ahora / 500) % 2 ? F_CRUZ : F_NADA);
  else dibujar(F_CRUZ);

  static unsigned long tLcd = 0;
  if (ahora - tLcd >= 250) { tLcd = ahora; pantalla(ahora); }

  // ---- latido para la app ----
  static unsigned long tEstado = 0;
  if (ahora - tEstado >= 500) { tEstado = ahora; enviarEstado(); }
}
