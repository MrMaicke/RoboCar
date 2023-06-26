/*
 * Fábio Ferreira de Souza
 * Node MCU ESP 8266
 * Controlado por roboremo.com
 * e usando sensor ultrasonico
*/

#include <ESP8266WiFi.h>
#include <Servo.h>
#include "UltrasonicMult.h" // adaptado de por necessidade de timeout: https://github.com/ErickSimoes/Ultrasonic

// Motor direito
#define MOTOR_E1    D1
#define MOTOR_E2    D2
#define MOTOR_EV    D0

// Motor esquerdo
#define MOTOR_D1    D3
#define MOTOR_D2    D4
#define MOTOR_DV    D5
#define MINVEL      150

// Sensor de distancia HC-SR04
#define TRIGGERPIN  D6
#define ECHOPIN     D7

// Servo motor da cabeça
#define CABECA      D8
#define ANGAJUSTE   25 // AJUSTE AO ANGULO INICIAL DA CABEÇA
#define ANGESQ      20 // ANGULO ESQUERDA
#define ANGCENTRO   45 // ANGULO CENTRAL
#define ANGDIR      70 // ANGULO DIREITA
#define MAXDIST     70


//how many clients should be able to telnet to this ESP8266
const char* ssid1 = "rede1";
const char* password1 = "12345";

const char* ssid2 = "rede2";
const char* password2 = "12345";

WiFiServer server(9000);
WiFiClient clientApp;
Servo cabeca;
UltrasonicMult ultrasonic(TRIGGERPIN, ECHOPIN, MAXDIST * 3 * CM);

#define MAXBUFFER 100
char comando[MAXBUFFER];
char buf[MAXBUFFER];
int pos = 0;
int posValor = 0;
int ang = ANGCENTRO; // prevendo 90 de visão
int Velocidade = 0;
int Direcao = 0;
int Sentido = 0;
int ciclo = 0;
int distanciaD = 0;
int distanciaC = 0;
int distanciaE = 0;
int angDir = 1;

void setup() {

  Serial.begin(115200);

  // Configuração dos pinos
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);
  pinMode(MOTOR_DV, OUTPUT);
  pinMode(MOTOR_E1, OUTPUT);
  pinMode(MOTOR_E2, OUTPUT);
  pinMode(MOTOR_EV, OUTPUT);

  cabeca.attach(CABECA);
  cabeca.write(ANGAJUSTE + ANGCENTRO);

  digitalWrite(MOTOR_D1, LOW);
  digitalWrite(MOTOR_D2, LOW);
  analogWrite(MOTOR_DV, 0);
  digitalWrite(MOTOR_E1, LOW);
  digitalWrite(MOTOR_E2, LOW);
  analogWrite(MOTOR_EV, 0);

  Serial.print("\nNodeMCU8266 RoboCar RoboRemo V2\n");

  WiFi.begin(ssid1, password1);
  Serial.print("\nConectando ");
  Serial.println(ssid1);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20)
  {
    delay(500);
  }

  if (i == 21) {
    Serial.print("Erro ao conectar-se a ");
    Serial.println(ssid1);

    Serial.print("\nConectando ");
    Serial.println(ssid2);
    WiFi.begin(ssid2, password2);
    i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 20)
    {
      delay(500);
    }
    if (i == 21) {
      Serial.print("Erro ao conectar-se a ");
      Serial.println(ssid1);
      Serial.println("FIM");
      while (1)
      {
        delay(1000); // Loop infinito
      }
    }
  }

  // Tudo OK, vamos começar!
  server.begin();
  server.setNoDelay(false);

  Serial.print(WiFi.localIP());
  Serial.println(":9000 to connect");
}

void MotoresParar()
{
  digitalWrite(MOTOR_D1, LOW);
  digitalWrite(MOTOR_D2, LOW);
  digitalWrite(MOTOR_E1, LOW);
  digitalWrite(MOTOR_E2, LOW);
}

void MotoresAceleracaoDirecao()
{
  // Apenas define a velocidade de 0 a 900 relacionando a um fator diferencia de -123 a +123 que define a direção
  int md = Velocidade + Direcao;
  int me = Velocidade - Direcao;

  // Motor direito
  if (md > MINVEL)
  {
    digitalWrite(MOTOR_D1, HIGH);
    digitalWrite(MOTOR_D2, LOW);
    analogWrite(MOTOR_DV, md);
  }
  else if (md < -MINVEL)
  {
    digitalWrite(MOTOR_D1, LOW);
    digitalWrite(MOTOR_D2, HIGH);
    analogWrite(MOTOR_DV, -md);
  }
  else // parado
  {
    digitalWrite(MOTOR_D1, LOW);
    digitalWrite(MOTOR_D2, LOW);
  }

  // Motor Esquerdo
  if (me > MINVEL)
  {
    digitalWrite(MOTOR_E1, LOW);
    digitalWrite(MOTOR_E2, HIGH);
    analogWrite(MOTOR_EV, me);
  }
  else if (me < -MINVEL)
  {
    digitalWrite(MOTOR_E1, HIGH);
    digitalWrite(MOTOR_E2, LOW);
    analogWrite(MOTOR_EV, -me);
  }
  else // parado
  {
    digitalWrite(MOTOR_E1, LOW);
    digitalWrite(MOTOR_E2, LOW);
  }
}

bool VerificaConexao()
{
  // Nova conexão
  if (server.hasClient()) {
    if (!clientApp || clientApp.connected()) {
      if (clientApp) clientApp.stop();
      clientApp = server.available();
      Serial.print("New client: ");
      Serial.println(clientApp.remoteIP());
    }
  }

  // Se não tiver nada conectado ou algo para ler, finaliza o loop
  if (!clientApp || !clientApp.connected())
  {
    digitalWrite(MOTOR_D1, LOW);
    digitalWrite(MOTOR_D2, LOW);
    digitalWrite(MOTOR_E1, LOW);
    digitalWrite(MOTOR_E2, LOW);
    cabeca.write(ANGAJUSTE + ANGCENTRO);
    Serial.println("\nDesconectado!");
    delay(1000);
    return false;
  }
}

bool LeComando()
{
  // Encontra o comando e a posição do valor (separador)
  while (clientApp.available()) {
    comando[pos] = clientApp.read();
    if (comando[pos] == '\n')
    {
      comando[pos] = 0;
      break;
    }
    else if (comando[pos] == ' ')
    {
      posValor = pos;
    }
    pos++;
    if (pos >= MAXBUFFER)
    {
      pos = 0; // reinicia (perde o ultimo comando!)
      posValor = 0;
    }
  }

  // Validação para ver se achou algo, caso contrario continua sem alteração nenhuma
  if (pos <= posValor)
    return false;

  return true;
}

void ProcessaComando()
{
  // Forma com pouco processamento para obter o comando e valor
  char* cValor = comando;
  int valor = 0;
  if (posValor > 0)
  {
    cValor += posValor + 1; // Avança o ponteiro para a posição de inicio do buffer
    comando[posValor] = 0; // Finaliza a string do comando
    valor = atoi(cValor);
  }

  // Reinicia variávei spara receber proximo comando
  pos = 0;
  posValor = 0;

  if (strcmp(comando, "sentido") == 0)
  {
    if (valor == 0) // Parar
    {
      Sentido = 0;
      cabeca.write(ANGAJUSTE + ANGCENTRO);
      MotoresParar();
    }
    else if (valor == 1) // Para Frente
    {
      Sentido = 1;
      cabeca.write(ANGAJUSTE + ANGCENTRO);
    }
    else if (valor == 2) // Automatico
    {
      Sentido = 2; // Ativa modo automatio
    }
    else // para traz
    {
      Sentido = -1;
      cabeca.write(ANGAJUSTE + ANGCENTRO);
    }
  }
  else if (Sentido == 2)
  {
    // Se estiver em modo automatico ignora qualquer outro comando
    return;
  }
  else if (strcmp(comando, "vel") == 0)
  {
    Velocidade = valor * Sentido;
    MotoresAceleracaoDirecao();
  }
  else if (strcmp(comando, "dir") == 0)
  {
    Direcao = valor * -1;
    MotoresAceleracaoDirecao();
  }
  else
  {
    Serial.print(comando);
    Serial.print("=");
    Serial.print(cValor);
    Serial.println(".");
  }
}

void Cabeca()
{
  // lê a distancia atual de onde a cabeça já estáposicionada
  int distancia = ultrasonic.distanceRead(CM);
  if (distancia == 0)
    distancia = MAXDIST;  

  if (ang == ANGESQ)
  {
    distanciaD = distancia;
    ang = ANGCENTRO;
    angDir = 1;
  }
  else if (ang == ANGCENTRO)
  {
    distanciaC = distancia;
    if (angDir == 1)
      ang = ANGDIR;
    else
      ang = ANGESQ;
  }
  else
  {
    distanciaE = distancia;
    ang = ANGCENTRO;
    angDir = -1;
  }

  sprintf(buf, "A %d %d => E %d C %d D %d => V %d D %d", ang, distancia, distanciaE, distanciaC, distanciaD, Velocidade, Direcao);
  Serial.println(buf);

  sprintf(buf, "DE %d\nDC %d\nDD %d\n", distanciaE, distanciaC, distanciaD);
  clientApp.write((const char *)buf, strlen(buf));

  cabeca.write(ANGAJUSTE + ang);
}

void Automatico()
{
  ciclo++;
  if (ciclo % 100 == 0)
    Cabeca();

  if (distanciaC < 15 || distanciaD < 7 || distanciaE < 7)
  {
    // Gira
    Velocidade = 0;
    if (distanciaD < distanciaE)
      Direcao = (2 * MINVEL + 50 );
    else
      Direcao = -( 2 * MINVEL + 50);
  }
  else if (distanciaC < MAXDIST/ 4)
  {
    // Anda devagar
    Velocidade = MINVEL + MAXDIST;
    Direcao = (distanciaE - distanciaD) * 4;
  }
  else if (distanciaC < MAXDIST/ 2)
  {
    // Anda devagar
    Velocidade = MINVEL + MAXDIST * 2;
    Direcao = (distanciaE - distanciaD) * 2;
  }
  else
  {
    // Corre na velocidade maxima
    Velocidade = MINVEL + MAXDIST * 3;
    Direcao = (distanciaE - distanciaD);
  }

  MotoresAceleracaoDirecao();
}

void loop() {

  delay(1);
  // A cada ciclo faz apenas uma tarefa

  if (!VerificaConexao())
    return;
  else if (LeComando())
    ProcessaComando();
  else if (Sentido == 2)
  {
    // Em modo automatico ativa a leitura da cabeça
    Automatico();
  }
}
