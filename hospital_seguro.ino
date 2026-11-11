/*
  INFINITY TEC - Hospital Inteligente (Sistema de Seguridad Hospitalaria)
  Control y avisos por Bluetooth (HC-05) desde la pagina web.

  Placa: Arduino UNO R4 WiFi
  Libreria: "MFRC522" (de GithubCommunity) desde el Gestor de librerias.
            Las demas (SPI, EEPROM, Arduino_LED_Matrix) ya vienen con la placa.

  CONEXIONES
    Bluetooth HC-05   VCC 5V, GND, TXD -> D0 (RX)
                      RXD <- D1 (TX) con divisor: D1 -> 1k -> RXD, y 2k de RXD a GND
    Lector RFID RC522 (va a 3.3V, NO a 5V)
                      3.3V, GND, RST D9, SDA D10, MOSI D11, MISO D12, SCK D13 (IRQ sin conectar)
    Ultrasonico HC-SR04 (puerta de Farmacia)   VCC 5V, GND, TRIG D6, ECHO D7
    LED verde    (acceso permitido)   D3 -> 220 ohm -> GND
    LED rojo     (alarma)             D4 -> 220 ohm -> GND
    LED amarillo (luz del pasillo)    D5 -> 220 ohm -> GND
    Buzzer activo                     + D2, - GND
    Fotoresistencia                   5V -> LDR -> A0, y 10k de A0 a GND
    Sensor de temperatura LM35 (cara plana hacia ti): izquierda 5V, centro A1, derecha GND

  QUE HACE
    - RFID: tarjeta registrada = acceso a Farmacia 8 s (LED verde + pitido corto).
            Tarjeta desconocida = LED rojo + 3 pitidos. 3 intentos en 1 minuto = ALARMA.
            Una tarjeta registrada tambien apaga la alarma de la puerta.
    - Ultrasonico: si alguien se queda frente a la puerta de Farmacia y no pasa una
            tarjeta valida en 8 s = ALARMA.
    - Fotoresistencia: de noche enciende la luz del pasillo (LED amarillo).
    - Temperatura: si pasa del limite = ALARMA (vuelve a avisar solo si baja y sube otra vez).
    - La matriz de la placa muestra el estado: cruz (normal), ? (esperando tarjeta),
      visto bueno (acceso), X (tarjeta negada), ! (alarma).

  ORDENES (una por linea, por Bluetooth o por el monitor serie a 9600):
    ON  OFF  ALERTA  SILENCIAR  ESTADO  ABRIR
    REGISTRAR (la proxima tarjeta queda registrada; 15 s)  CANCELAR  BORRAR (borra todas)
    DIST:n (cm, 5-150)   LUZ:n (%, 5-90)   TMAX:n (grados, 20-60)
  ENVIA cada 0,5 s:
    ST,sistema,alerta,origen,distancia,persona,espera_s,acceso,luz%,noche,temp*10,temp_ok,
       dist_puerta,luz_noche,temp_max,tarjetas,registrando
    origen: 0 ninguno, 1 persona sin tarjeta, 2 temperatura alta, 3 manual (app), 4 tarjeta no autorizada
  Y ademas:  EV,texto      TJ,uid,1/0 (tarjeta leida: 1 autorizada, 0 no)
*/

#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include "Arduino_LED_Matrix.h"

#define BT Serial1   // HC-05 en D0/D1 (en el R4 el USB va aparte)

const byte P_BUZ = 2, P_VERDE = 3, P_ROJO = 4, P_AMAR = 5, P_TRIG = 6, P_ECHO = 7;
const byte P_RST = 9, P_SS = 10;
const byte P_LDR = A0, P_TEMP = A1;

MFRC522 rfid(P_SS, P_RST);
ArduinoLEDMatrix matriz;

// ---------- ajustes (se guardan en la memoria) ----------
int distPuerta = 30;   // cm: mas cerca = hay alguien en la puerta
int luzNoche = 30;     // %: menos luz = noche
int tempMax = 30;      // grados: mas = alarma
const byte MAX_TARJ = 10;
byte nTarj = 0;
byte tarjLen[MAX_TARJ];
byte tarjUid[MAX_TARJ][7];

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
float temp = 0;
bool tempOk = false, avisoCalor = false;
String linUsb, linBt;
String ultimaUid;
unsigned long tUltimaUid = 0;

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
  s += nTarj; s += ','; s += registrando;
  enviar(s);
}

// ---------- memoria ----------
void guardar() {
  EEPROM.update(0, 0xB3);
  EEPROM.update(1, distPuerta);
  EEPROM.update(2, luzNoche);
  EEPROM.update(3, tempMax);
  EEPROM.update(4, nTarj);
  for (byte i = 0; i < MAX_TARJ; i++) {
    EEPROM.update(10 + i * 8, tarjLen[i]);
    for (byte j = 0; j < 7; j++) EEPROM.update(11 + i * 8 + j, tarjUid[i][j]);
  }
}
void cargar() {
  if (EEPROM.read(0) != 0xB3) return;   // primera vez: valores de fabrica
  distPuerta = constrain(EEPROM.read(1), 5, 150);
  luzNoche = constrain(EEPROM.read(2), 5, 90);
  tempMax = constrain(EEPROM.read(3), 20, 60);
  nTarj = min((int)EEPROM.read(4), (int)MAX_TARJ);
  for (byte i = 0; i < MAX_TARJ; i++) {
    tarjLen[i] = min((int)EEPROM.read(10 + i * 8), 7);
    for (byte j = 0; j < 7; j++) tarjUid[i][j] = EEPROM.read(11 + i * 8 + j);
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
      nTarj++; guardar();
      evento("Tarjeta " + id + " registrada");
    }
    pitidoHasta = ahora + 150;
    enviar("TJ," + id + ",1");
    return;
  }
  bool ok = buscarTarjeta(u, n) >= 0;
  enviar("TJ," + id + "," + String(ok));
  if (ok) {
    accesoHasta = ahora + ACCESO_MS; esperaFin = 0; negados = 0;
    pitidoHasta = ahora + 150;
    evento("Acceso permitido a Farmacia: tarjeta " + id);
    if (alerta && (origen == 1 || origen == 4)) silenciar("Alarma desactivada con una tarjeta autorizada");
  } else {
    if (ahora - tNegado > 60000UL) negados = 0;
    tNegado = ahora; negados++;
    negadoHasta = ahora + 1500;
    evento("Tarjeta NO autorizada: " + id + " (intento " + String(negados) + ")");
    if (negados >= 3 && sistema) alertar(4, "ALARMA: 3 intentos con tarjeta no autorizada");
  }
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
  tempOk = t > 1 && t < 110;                     // sin sensor o mal conectado da valores raros
  temp = tempOk ? (temp == 0 ? t : temp * 0.7 + t * 0.3) : t;
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
  else if (cmd == "DIST" && val >= 0) { distPuerta = constrain(val, 5, 150); guardar(); evento("Distancia de la puerta = " + String(distPuerta) + " cm"); }
  else if (cmd == "LUZ" && val >= 0)  { luzNoche = constrain(val, 5, 90); guardar(); evento("Noche con menos de " + String(luzNoche) + " % de luz"); }
  else if (cmd == "TMAX" && val >= 0) { tempMax = constrain(val, 20, 60); avisoCalor = false; guardar(); evento("Temperatura maxima = " + String(tempMax) + " C"); }
  else if (cmd == "ESTADO") { }
  else { enviar("EV,Orden desconocida: " + s); return; }
  enviarEstado();
}
void leer(Stream &p, String &lin) {
  while (p.available()) {
    char c = p.read();
    if (c == '\n' || c == '\r') { if (lin.length()) orden(lin); lin = ""; }
    else if (lin.length() < 24) lin += c;
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(P_BUZ, OUTPUT); pinMode(P_VERDE, OUTPUT); pinMode(P_ROJO, OUTPUT); pinMode(P_AMAR, OUTPUT);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT);
  digitalWrite(P_TRIG, LOW);
  analogReadResolution(14);
  cargar();
  SPI.begin();
  rfid.PCD_Init();
  matriz.begin();
  dibujar(F_CRUZ);
  // prueba de luces al encender
  digitalWrite(P_VERDE, HIGH); digitalWrite(P_ROJO, HIGH); digitalWrite(P_AMAR, HIGH); digitalWrite(P_BUZ, HIGH);
  delay(150); digitalWrite(P_BUZ, LOW); delay(450);
  digitalWrite(P_VERDE, LOW); digitalWrite(P_ROJO, LOW); digitalWrite(P_AMAR, LOW);
  leerTemp();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  evento("Hospital inteligente listo");
  evento(v == 0x00 || v == 0xFF ? "No se encuentra el lector RFID: revisa sus cables (3.3V)" : "Lector RFID listo (" + String(nTarj) + " tarjetas registradas)");
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

  // ---- ultrasonico en la puerta de Farmacia ----
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
    if (!noche && luz < luzNoche) { noche = true; evento("Anochece: luz del pasillo encendida"); }
    else if (noche && luz > luzNoche + 5) { noche = false; evento("Hay luz: luz del pasillo apagada"); }
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

  // ---- luces, buzzer y matriz ----
  bool parpadeo = (ahora / 250) % 2;
  bool acceso = ahora < accesoHasta;
  bool negado = ahora < negadoHasta;
  digitalWrite(P_VERDE, acceso || (sistema && !alerta && (ahora % 2000) < 80));   // latido cada 2 s
  digitalWrite(P_ROJO, alerta ? parpadeo : negado || (esperaFin && (ahora / 500) % 2));
  digitalWrite(P_AMAR, noche);
  bool buz = false;
  if (alerta) buz = parpadeo;
  else if (negado) buz = ((negadoHasta - ahora) / 120) % 2;                     // 3 pitidos
  else if (esperaFin) buz = (ahora % 1000) < 50;                                 // aviso cada segundo
  if (ahora < pitidoHasta) buz = true;
  digitalWrite(P_BUZ, buz);

  if (alerta) dibujar(parpadeo ? F_ALER : F_NADA);
  else if (negado) dibujar(F_MAL);
  else if (acceso) dibujar(F_BIEN);
  else if (esperaFin || registrando) dibujar(F_PREG);
  else dibujar(F_CRUZ);

  // ---- latido para la app ----
  static unsigned long tEstado = 0;
  if (ahora - tEstado >= 500) { tEstado = ahora; enviarEstado(); }
}
