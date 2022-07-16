/*

  - ESP32 DEVKIT V1
  - ARDUINO IDE V1.8.12
  - Bibliotecas
    - TFT_eSPI: 2.3.70 https://github.com/Bodmer/TFT_eSPI
    - ADS1X15: 0.3.3
    - EncoderStepCounter: 1.1.0
    - Thermistor
    - KeyPad_I2C: 3.0.0
    - EEPROM: Nativa da IDE
    - Wire: Nativa da IDE
    - SPI: Nativa da IDE


  13/07
  mudança da leitura do encoder, sem biblioteca por interrupcao
  mudança do botao OUT
  leitura de temperatura sem biblioteca






  Tecla A => SHIFT
  Tecla B => MEMORIAS
  Tecla C => SET (VSET/ISET)
  Tecla D => ENTER
  Tecla * => Seta esquerda <
  Tecla # => Seta direita >

  Shift + 1 => liga CH1
  Shift + 2 => liga CH2
  Shift + 3 => liga CH3
  Shift + 4 => seleciona CH1
  Shift + 5 => seleciona CH2
  Shift + 6 => seleciona CH3
  Shift + 7 => tracking
  Shift + 0 => calibracao
  Shift + D => menu


-----------------------------------------------------------------
  PINOUT

  ESP32
  GPIO 0  -                   PWM at boot
  GPIO 1  - TX                debug output at boot

  GPIO 2  - TFT_DC            LED onboard (30PINS BOARD)
  GPIO 3  - RX                HIGH at boot
  GPIO 4  -
  GPIO 5  - ENCODER DT        PWM at boot
  GPIO 12 - RL1_CH1
  GPIO 13 - RL2_CH1
  GPIO 14 - FAN               PWM at boot
  GPIO 15 - TFT_CS
  GPIO 16 - ENCODER SW
  GPIO 17 - ENCODER CLK
  GPIO 18 - TFT_SCLK
  GPIO 19 - TFT_MISO
  GPIO 21 - SDA I2C
  GPIO 22 - SCL I2C
  GPIO 23 - TFT_MOSI
  GPIO 25 - BUZZER            DAC1
  GPIO 26 - NTC               DAC2
  GPIO 27 - RL2_CH2
  GPIO 32 - EXT_TRIG
  GPIO 33 - RL1_CH2
  GPIO 34 - CH1_CC            INPUT ONLY nao tem pullup
  GPIO 35 - B_OUT             INPUT ONLY nao tem pullup
  GPIO 36 - CH3_CC            INPUT ONLY nao tem pullup
  GPIO 39 - CH2_CC            INPUT ONLY nao tem pullup
-----------------------------------------------------------------

  Display       ESP32
  VCC             3V3
  GND             GND
  TFT_CS           15
  TFT_RST         3V3
  TFT_DC            2
  TFT_MOSI         23
  TFT_SCLK         18
  LED             3V3
  TFT_MISO         19


  SDA              21
  SCL              22

  ENCODER       ESP32
  DT                5
  CLK              17
  SW               16

  RELE1 CH1        12
  RELE2 CH1        13
  RELE1 CH2        33
  RELE2 CH2        27

  BUZZER           25
  FAN              14
  NTC              26
  EXT TRIGGER      32
  BOTAO OUT        35
  CH1_CC           34
  CH2_CC           39
  CH3_CC           36

-----------------------------------------------------------------

  TFT_BLACK 0x0000       ///<   0,   0,   0
  TFT_NAVY 0x000F        ///<   0,   0, 123
  TFT_DARKGREEN 0x03E0   ///<   0, 125,   0
  TFT_DARKCYAN 0x03EF    ///<   0, 125, 123
  TFT_MAROON 0x7800      ///< 123,   0,   0
  TFT_PURPLE 0x780F      ///< 123,   0, 123
  TFT_OLIVE 0x7BE0       ///< 123, 125,   0
  TFT_LIGHTGREY 0xC618   ///< 198, 195, 198
  TFT_DARKGREY 0x7BEF    ///< 123, 125, 123
  TFT_BLUE 0x001F        ///<   0,   0, 255
  TFT_GREEN 0x07E0       ///<   0, 255,   0
  TFT_CYAN 0x07FF        ///<   0, 255, 255
  TFT_RED 0xF800         ///< 255,   0,   0
  TFT_MAGENTA 0xF81F     ///< 255,   0, 255
  TFT_YELLOW 0xFFE0      ///< 255, 255,   0
  TFT_WHITE 0xFFFF       ///< 255, 255, 255
  TFT_ORANGE 0xFD20      ///< 255, 165,   0
  TFT_GREENYELLOW 0xAFE5 ///< 173, 255,  41
  TFT_PINK 0xFC18        ///< 255, 130, 198

#define COLOR1 TFT_YELLOW
#define COLOR2 TFT_CYAN
#define COLOR3 TFT_MAGENTA
#define COLOR4 TFT_WHITE 
#define COLOR5 TFT_GREEN
#define COLOR6 TFT_BLUE
#define COLOR7 TFT_RED
#define COLOR8 TFT_ORANGE
#define COLOR9 TFT_LIGHTGREY

*/

#define VERSAO 1.3

// PINAGEM DE CONEXAO
//------------------------------------------------------------------------

#define NTC_PIN 26      // Sensor de temperatura LM35 ou NTC
#define FAN_PIN 14      // Fan PWM pino 14 canal 0
#define PWM_FAN 0       // PWM Fan canal 0
#define BUZZER_PIN 25   // Buzzer
#define B_OUT 35        // Botao OUT
#define EXT_TRIG_PIN 32 // Triger Externo
#define encoderSW 16    // Botao do encoder
#define enc_CLK 17      // Encoder clock
#define enc_DT 5        // Encode data

#define RL1_CH1 12 // Rele 1 chaveador do canal 1
#define RL2_CH1 13 // Rele 2 chaveador do canal 1
#define RL1_CH2 33 // Rele 1 chaveador do canal 2
#define RL2_CH2 27 // Rele 2 chaveador do canal 1

#define CC_CH1 34 // Indicador modo CC canal 1
#define CC_CH2 39 // Indicador modo CC canal 2
#define CC_CH3 36 // Indicador modo CC canal 3

//--------------------------------------------------------------------------

#define V_MAX 32.00    // Tensão máxima de saída dos canais 1 e 2
#define V_MAX_CH3 6.20 // Tensão máxima de saída do canal 3
#define I_MAX 3.200    // Corrente máxima de saída para os 3 canais
#define VREF 3.3       // Tensão de referência para os 3 canais
#define DIV_TENSAO 10  // Valor do divisor de tensão para os 3 canais

#define RL1_SWITCH 6.5  // Tensao para ligar/desligar rele 1
#define RL2_SWITCH 19.0 // Tensao para ligar/desligar rele 2

#define COLOR1 TFT_YELLOW  // cor canal 1
#define COLOR2 TFT_CYAN    // cor canal 2
#define COLOR3 TFT_MAGENTA // cor canal 3
#define COLOR4 TFT_WHITE   // cor rodape

#define ITENS_MENU_CONFIG 4 // Quantidade de itens no menu de config
#define ITENS_MENU_CH1_2 6  // Quantidade de itens no menu do canal 1 e 2
#define ITENS_MENU_CH3 6    // Quantidade de itens no menu do canal 3

//-------------------ENDEREÇOS DE MODULOS------------------------------------------------

#define CH1 3 // CANAL DO TCA9548 para CH1
#define CH2 5 // CANAL DO TCA9548 para CH2
#define CH3 6 // CANAL DO TCA9548 para CH3

#define DAC_V_CH1 0x60 // Endereço HEX para o DAC_V do canal 1
#define DAC_I_CH1 0x61 // Endereço HEX para o DAC_I do canal 1
#define DAC_V_CH2 0x60 // Endereço HEX para o DAC_V do canal 2
#define DAC_I_CH2 0x61 // Endereço HEX para o DAC_I do canal 2
#define DAC_V_CH3 0x60 // Endereço HEX para o DAC_V do canal 3
#define DAC_I_CH3 0x61 // Endereço HEX para o DAC_I do canal 3
#define PWR_ON 0x40    // Modo POWER DOWN ligadao (desliga saida do DAC)
#define PWR_OFF 0x42   // Modo POWER DOWN desligado (liga da saida do DAC)

#define KPD_ADR 0x20 // Endereco HEX KeyBoard I2C

//---------------------------------BIBLIOTECAS-------------------------------------------
#include <EEPROM.h>

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

#include <EncoderStepCounter.h>
EncoderStepCounter encoder(enc_DT, enc_CLK);

#include <ADS1X15.h>
ADS1115 ADS_CH1(0x48); // Endereço HEX para o ADS do canal 1
ADS1115 ADS_CH2(0x48); // Endereço HEX para o ADS do canal 2
ADS1115 ADS_CH3(0x48); // Endereço HEX para o ADS do canal 3

#include <Keypad_I2C.h>
const byte ROWS = 4; // four rows
const byte COLS = 4; // four columns
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte rowPins[ROWS] = {3, 2, 1, 0}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {7, 6, 5, 4}; // connect to the column pinouts of the keypad
Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KPD_ADR);

//----------------------------VARIAVEIS GLOBAIS------------------------------------------

float MEMORY_VSET_CH1[10]; // Memórias canal 1
float MEMORY_ISET_CH1[10]; // Memórias canal 1
float MEMORY_VSET_CH2[10]; // Memórias canal 2
float MEMORY_ISET_CH2[10]; // Memórias canal 2
float MEMORY_VSET_CH3[10]; // Memórias canal 3
float MEMORY_ISET_CH3[10]; // Memórias canal 3
float MEMORY_VSET_ALL[13]; // Memórias todos os canais
float MEMORY_ISET_ALL[13]; // Memórias todos os canais

bool MEMORY_MODE;      // MODE=0 =>call memory / MODE=1 =>store memory
bool ENCODER_ADJ;      // Habilita/Desabilita ajuste por encoder
String string;         // Variável armazena dados digitados pelo keypad
double TEMP;           // Temperatura lida no sensor
byte TEMP_MIN;         // Temperatura para aceleracao do cooler
byte FAN_PWM;          // Valor PWM para o fan
byte FAN_PWM_MIN;      // PWM minimo quando estiver fria
byte OTP;              // Temperatura OTP
bool FLAG_OTP;         // FLAG OTP ACIONADO
bool FLAG_TRIGGER;     // FLAG trigger externo acionado
char key;              // tecla pressionada no teclado
signed char pos;       // controle posicao do encoder
bool FLAG_DIGIT;       // modo de digitacao dos numeros ativo

unsigned long previousMillis1 = 0;
byte TIMER;            // tempo de pressionamento do botao do encoder
byte MENU_ID = 1;      // Posicao do cursor nas telas de menu
byte ID_DISPLAY = 0;   // ID de tela atual
float X = 0.0001;      // Fatos de multiplicação para modo de calibração
bool flag_B_OUT;       // Flag Botão de saída
bool flag_encoderSW;   // Flag botão do encoder
bool FLAG_VSET = 1;    // ajuste VSET ativo
bool FLAG_ISET = 0;    // ajuste ISET ativo
byte FLAG_DIGIT_V = 0; // digito de ajuste VSET
byte FLAG_DIGIT_A = 0; // digito de ajuste ISET

int DELAY_CH1, DELAY_CH2, DELAY_CH3;           // delay para ligar os canais (nao implementado)
int COLOR_CH1, COLOR_CH2, COLOR_CH3;           // cor dos canais (nao implementado)
bool TRIGGER_CH1, TRIGGER_CH2, TRIGGER_CH3;    // trigger habilitado
bool FLAG_BOUT_CH1, FLAG_BOUT_CH2, FLAG_BOUT_CH3;
byte FLAG_SELECT = 0;                          // flag de canal selecionado 1/2/3
bool FLAG_SHIFT = 0;                           // flag shift
bool FLAG_BUZZER;                              // flag buzzer
bool FLAG_LOCK = 0;                            // flag lock - trava o teclado
bool FLAG_TRACK;                               // flag tracking
bool FLAG_ADJ_CH1, FLAG_ADJ_CH2, FLAG_ADJ_CH3; // flag de ajuste canal selecionado
bool FLAG_OUT_CH1, FLAG_OUT_CH2, FLAG_OUT_CH3; // flag saida ligada/desligada
bool FLAG_CC_CH1, FLAG_CC_CH2, FLAG_CC_CH3;    // flag CC ativo
bool FLAG_OVP_CH1, FLAG_OVP_CH2, FLAG_OVP_CH3; // flag OVP ativo
bool FLAG_OCP_CH1, FLAG_OCP_CH2, FLAG_OCP_CH3; // flag OCP ativo

float VMAX_CH1, VMAX_CH2, VMAX_CH3; // tensao maxima configuravel para os canais
float OVP_CH1, OVP_CH2, OVP_CH3;    // protecao OVP
float OCP_CH1, OCP_CH2, OCP_CH3;    // protecao OCP

float VOUT_CH1, IOUT_CH1, POUT_CH1, VSET_CH1, ISET_CH1; // ajustes e medições CH1
float VOUT_CH2, IOUT_CH2, POUT_CH2, VSET_CH2, ISET_CH2; // ajustes e medições CH2
float VOUT_CH3, IOUT_CH3, POUT_CH3, VSET_CH3, ISET_CH3; // ajustes e medições CH3

float V_DAC_CH1;   // DAC CH1
float I_DAC_CH1;   // DAC CH1
int SET_V_DAC_CH1; // DAC CH1
int SET_I_DAC_CH1; // DAC CH1

float V_DAC_CH2;   // DAC CH2
float I_DAC_CH2;   // DAC CH2
int SET_V_DAC_CH2; // DAC CH2
int SET_I_DAC_CH2; // DAC CH2

float V_DAC_CH3;   // DAC CH3
float I_DAC_CH3;   // DAC CH3
int SET_V_DAC_CH3; // DAC CH3
int SET_I_DAC_CH3; // DAC CH3

//--------------------------------VARIAVEIS DE CALIBRACAO---------------------------------------
float CH1_METER_V, CH1_METER_I;           // CALIBRACAO DOS MEDIDORES CH1
float CH1_OUT_V1, CH1_OUT_V2, CH1_OUT_V3; // CALIBRACAO DA SAIDA DE TENSAO CH1
float CH1_OUT_I1, CH1_OUT_I2, CH1_OUT_I3; // CALIBRACAO DA SAIDA DE CORRENTE CH1
float CH1_OFFSET;                         // OFFSET DE 0mA DE CORRENTE CH1
byte CH1_ID_CAL = 1;                      // posição do cursor na tela de calibração CH1

float CH2_METER_V, CH2_METER_I;           // CALIBRACAO DOS MEDIDORES CH2
float CH2_OUT_V1, CH2_OUT_V2, CH2_OUT_V3; // CALIBRACAO DA SAIDA DE TENSAO CH2
float CH2_OUT_I1, CH2_OUT_I2, CH2_OUT_I3; // CALIBRACAO DA SAIDA DE CORRENTE CH2
float CH2_OFFSET;                         // OFFSET DE 0mA DE CORRENTE CH2
byte CH2_ID_CAL = 1;                      // posição do cursor na tela de calibração CH2

float CH3_METER_V, CH3_METER_I; // CALIBRACAO DOS MEDIDORES CH3
float CH3_OUT_V1;               // CALIBRACAO DA SAIDA DE TENSAO CH3
float CH3_OUT_I1;               // CALIBRACAO DA SAIDA DE CORRENTE CH3
float CH3_OFFSET;               // OFFSET DE 0mA DE CORRENTE CH3
byte CH3_ID_CAL = 1;            // posição do cursor na tela de calibração CH3



//----------------------------------SETUP-----------------------------------------------
void setup()
{
  EEPROM.begin(2048);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  Wire.begin();
  Wire.setClock(400000);

  kpd.begin();

  pinMode(FAN_PIN, OUTPUT);        // Define pino como saida
  ledcAttachPin(FAN_PIN, PWM_FAN); // Atribui o pino FAN_PIN ao canal 0.
  ledcSetup(PWM_FAN, 25000, 8);    // Atribui ao canal 0 a frequencia de 25kHz com resolucao de 8bits.
  ledcWrite(PWM_FAN, 255);         // Liga FAN no PWM máximo

  pinMode(EXT_TRIG_PIN, INPUT);     // usa pull_up externo
  pinMode(B_OUT, INPUT);            // usa pull_up externo
  pinMode(encoderSW, INPUT_PULLUP); // encoder switch - pullup interno

  attachInterrupt(digitalPinToInterrupt(enc_DT), interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_CLK), interrupt, CHANGE);

  pinMode(RL1_CH1, OUTPUT);    // saida RL1_CH1
  pinMode(RL2_CH1, OUTPUT);    // saida RL2_CH1
  pinMode(RL1_CH2, OUTPUT);    // saida RL1_CH2
  pinMode(RL2_CH2, OUTPUT);    // saida RL2_CH2
  pinMode(BUZZER_PIN, OUTPUT); // saida buzzer
  pinMode(CC_CH1, INPUT);      // usa pull_up externo
  pinMode(CC_CH2, INPUT);      // usa pull_up externo
  pinMode(CC_CH3, INPUT);      // usa pull_up externo

  BOOT();     //executa sequencia de boot
  DAC_INIT(); //inicializa dos DACs

  if (EEPROM.read(2040) != 1) // gravar configuracoes e memoria no primeiro boot
  FIRST_BOOT();               // chama funcao FIRST_BOOT

  RESTORE_CALIBRATION();  // resupera os valores de calibracao dos medidores
  RESTORE_CONFIG();       // recupera os valores de configuracao
  CALL_MEMORY();          // recupera os valores salvos na memorias
  CALL_VALUES();          // recupera os valores ajustados da tela principal

  BUZZER(50);

} // setup

//----------------------------------LOOP------------------------------------------------
void loop(void)
{
    METER();
    FAN_CONTROL();
    KEYBOARD_GLOBAL();

    switch (ID_DISPLAY)
    {
    case 0:
      DISPLAY_PRINCIPAL();
      KEYBOARD_PRINCIPAL();
      break;

    case 1:
      DISPLAY_CALIBRACAO();
      KEYBOARD_CALIBRACAO();
      break;

    case 2:
      DISPLAY_MEMORIAS();
      KEYBOARD_MEMORIAS();
      break;

    case 3:
      DISPLAY_MENU();
      KEYBOARD_MENU();
      break;
    } // switch ID_DISPLAY


} // loop

void FAN_CONTROL()
{
  int RawADC = analogRead(NTC_PIN);
  long Resistance;

  // NTC 10k | Resistance = (4095/ADC)
	Resistance=((40950000/RawADC) - 11000);
  TEMP = log(Resistance);
  TEMP = 1 / (0.001129148 + (0.000234125 * TEMP) + (0.0000000876741 * TEMP * TEMP * TEMP));
  TEMP = TEMP - 273.15;  // Convert Kelvin to Celsius

  if (TEMP < TEMP_MIN)
  {
    ledcWrite(PWM_FAN, FAN_PWM_MIN);
  }
  else
  {
    FAN_PWM = constrain(map(TEMP, TEMP_MIN, OTP - 10, FAN_PWM_MIN, 255), 0, 255);
    ledcWrite(PWM_FAN, FAN_PWM);
  }

// --- OTP ---
  if (TEMP > OTP)
  {
    FLAG_OUT_CH1 = 0;
    FLAG_OUT_CH2 = 0;
    FLAG_OUT_CH3 = 0;    
    SET_OUT();
    FLAG_OTP = 1;
  }
  else if (TEMP < OTP - 10)
  {
    FLAG_OTP = 0;
  }

} // FAN_CONTROL

void BOOT()
{
  byte error, address;
  int nDevices;

  tft.drawFloat(VERSAO, 2, 400, 50, 4);

  while (nDevices < 10)
  {

    tft.setCursor(0, 7);
    tft.setTextFont(2);
    tft.println("Initializing...");
    delay(500);
    nDevices = 0;

    TCA9548A(CH1);
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0)
      {
        tft.print("CH1: 0x");
        if (address < 16)
          tft.print("0");
        tft.println(address, HEX);
        nDevices++;
      }
      delay(10);
    }

    delay(200);
    tft.println("");

    TCA9548A(CH2);
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0)
      {
        tft.print("CH2: 0x");
        if (address < 16)
          tft.print("0");
        tft.println(address, HEX);
        nDevices++;
      }
      delay(10);
    }

    delay(200);
    tft.println("");

    TCA9548A(CH3);
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0)
      {
        tft.print("CH3: 0x");
        if (address < 16)
          tft.print("0");
        tft.println(address, HEX);
        nDevices++;
      }
      delay(10);
    }

    delay(2000); // wait 2 seconds for next scan
    tft.fillScreen(TFT_BLACK);
  }

  /*
      0x20 Keypad
      0x40 ADS
      0x60 DAC_V
      0x61 DAC_I
      0x70 TCA
  */

} // boot

void DAC_INIT()
{
  TCA9548A(CH1);
  ADS_CH1.begin();
  ADS_CH1.setGain(0);
  DAC_Write(DAC_V_CH1, PWR_OFF, 0);
  DAC_Write(DAC_I_CH1, PWR_OFF, 0);

  TCA9548A(CH2);
  ADS_CH2.begin();
  ADS_CH2.setGain(0);
  DAC_Write(DAC_V_CH2, PWR_OFF, 0);
  DAC_Write(DAC_I_CH2, PWR_OFF, 0);

  TCA9548A(CH3);
  ADS_CH3.begin();
  ADS_CH3.setGain(0);
  DAC_Write(DAC_V_CH3, PWR_OFF, 0);
  DAC_Write(DAC_I_CH3, PWR_OFF, 0);
}

void interrupt()
{
  if (ENCODER_ADJ /*&& FLAG_SELECT > 0*/)
  {
    encoder.tick();
    pos = encoder.getPosition();
  }
}
 
float readEncoder(float value, float steps, float MIN, float MAX)
{
  value += (encoder.getPosition() * steps);
  if (value > MAX)
    value = MAX;
  if (value < MIN)
    value = MIN;
  encoder.reset();
  return value;
}

void DAC_Write(char DAC, char MODE, uint16_t VALUE)
{
  Wire.beginTransmission(DAC);
  Wire.write(MODE);               // 0b01000000 0x40 - ON | 0b01000010 0x42 - OFF
  Wire.write(VALUE >> 4);         // (D11.D10.D9.D8.D7.D6.D5.D4)
  Wire.write((VALUE & 0xF) << 4); // (D3.D2.D1.D0.0.0.0.0)
  Wire.endTransmission();
}

void TCA9548A(int bus)
{
  Wire.beginTransmission(0x70); // TCA9548A address is 0x70
  Wire.write(1 << bus);         // send byte to select bus
  Wire.endTransmission();
}

void BUZZER(int TEMPO)
{
  if (FLAG_BUZZER)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(TEMPO);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void RELES_CH1()
{
  if (VOUT_CH1 > RL1_SWITCH + 0.5)
    digitalWrite(RL1_CH1, HIGH);
  else if (VOUT_CH1 < RL1_SWITCH - 0.5)
    digitalWrite(RL1_CH1, LOW);
  if (VOUT_CH1 > RL2_SWITCH + 0.5)
    digitalWrite(RL2_CH1, HIGH);
  else if (VOUT_CH1 < RL2_SWITCH - 0.5)
    digitalWrite(RL2_CH1, LOW);
}

void RELES_CH2()
{
  if (VOUT_CH2 > RL1_SWITCH + 0.5)
    digitalWrite(RL1_CH2, HIGH);
  else if (VOUT_CH2 < RL1_SWITCH - 0.5)
    digitalWrite(RL1_CH2, LOW);
  if (VOUT_CH2 > RL2_SWITCH + 0.5)
    digitalWrite(RL2_CH2, HIGH);
  else if (VOUT_CH2 < RL2_SWITCH - 0.5)
    digitalWrite(RL2_CH2, LOW);
}

void METER()
{

  FLAG_CC_CH1 = (!digitalRead(CC_CH1));
  FLAG_CC_CH2 = (!digitalRead(CC_CH2));
  FLAG_CC_CH3 = (!digitalRead(CC_CH3));

  //------------------CH1------------------------------------------------------
  TCA9548A(CH1);
  VOUT_CH1 = (ADS_CH1.toVoltage(ADS_CH1.readADC(0)) * DIV_TENSAO) * CH1_METER_V;
  if (VOUT_CH1 < 0)
    VOUT_CH1 = 0;
  IOUT_CH1 = (ADS_CH1.toVoltage(ADS_CH1.readADC(1)) - CH1_OFFSET) * CH1_METER_I;
  if (IOUT_CH1 < 0)
    IOUT_CH1 = 0;
  POUT_CH1 = (VOUT_CH1 * IOUT_CH1);

  //--- OVP --- OCP ---
  if (VOUT_CH1 > OVP_CH1)
  {
    DAC_Write(DAC_V_CH1, PWR_OFF, 0);
    FLAG_OUT_CH1 = 0;
    //SET_OUT();
    FLAG_OVP_CH1 = 1;
    BUZZER(1000);
  }
  if (IOUT_CH1 > OCP_CH1)
  {
    DAC_Write(DAC_V_CH1, PWR_OFF, 0);
    FLAG_OUT_CH1 = 0;
    //SET_OUT();
    FLAG_OCP_CH1 = 1;
    BUZZER(1000);
  }  

  //------------------CH2------------------------------------------------------
  TCA9548A(CH2);
  VOUT_CH2 = (ADS_CH2.toVoltage(ADS_CH2.readADC(0)) * DIV_TENSAO) * CH2_METER_V;
  if (VOUT_CH2 < 0)
    VOUT_CH2 = 0;
  IOUT_CH2 = (ADS_CH2.toVoltage(ADS_CH2.readADC(1)) - CH2_OFFSET) * CH2_METER_I;
  if (IOUT_CH2 < 0)
    IOUT_CH2 = 0;
  POUT_CH2 = (VOUT_CH2 * IOUT_CH2);

  //--- OVP --- OCP ---
  if (VOUT_CH2 > OVP_CH2)
  {
    DAC_Write(DAC_V_CH2, PWR_OFF, 0);
    FLAG_OUT_CH2 = 0;
    //SET_OUT();
    FLAG_OVP_CH2 = 1;
    BUZZER(1000);
  }
  if (IOUT_CH2 > OCP_CH2)
  {
    DAC_Write(DAC_V_CH2, PWR_OFF, 0);
    FLAG_OUT_CH2 = 0;
    //SET_OUT();
    FLAG_OCP_CH2 = 1;
    BUZZER(1000);
  }

  //------------------CH3------------------------------------------------------
  TCA9548A(CH3);
  VOUT_CH3 = (ADS_CH3.toVoltage(ADS_CH3.readADC(0)) * DIV_TENSAO) * CH3_METER_V;
  if (VOUT_CH3 < 0)
    VOUT_CH3 = 0;
  IOUT_CH3 = (ADS_CH3.toVoltage(ADS_CH3.readADC(1)) - CH3_OFFSET) * CH3_METER_I;
  if (IOUT_CH3 < 0)
    IOUT_CH3 = 0;
  POUT_CH3 = (VOUT_CH3 * IOUT_CH3);

  //--- OVP --- OCP ---
  if (VOUT_CH3 > OVP_CH3)
  {
    DAC_Write(DAC_V_CH3, PWR_OFF, 0);
    FLAG_OUT_CH3 = 0;
    //SET_OUT();
    FLAG_OVP_CH3 = 1;
    BUZZER(1000);
  }
  if (IOUT_CH3 > OCP_CH3)
  {
    DAC_Write(DAC_V_CH3, PWR_OFF, 0);
    FLAG_OUT_CH3 = 0;
    //SET_OUT();
    FLAG_OCP_CH3 = 1;
    BUZZER(1000);
  }


  //------------CHAVEADOR DOS RELES----------------------------------------------

  if (FLAG_OUT_CH1)
    RELES_CH1();
  else
  {
    digitalWrite(RL1_CH1, LOW);
    digitalWrite(RL2_CH1, LOW);
  }
  if (FLAG_OUT_CH2)
    RELES_CH2();
  else
  {
    digitalWrite(RL1_CH2, LOW);
    digitalWrite(RL2_CH2, LOW);
  }

} // METER

void SET_OUT()
{

  if (FLAG_TRACK)
  {
    VSET_CH1 = VSET_CH2;
    ISET_CH1 = ISET_CH2;
  }

  //------------------CH1----------------------------------
  TCA9548A(CH1);
  //---------V_SET - AJUSTE DE TENSAO----------------------
  if (VSET_CH1 <= 10.00)
    V_DAC_CH1 = VSET_CH1 / (V_MAX / VREF) * CH1_OUT_V1;
  else if (VSET_CH1 > 10.00 && VSET_CH1 <= 25.00)
    V_DAC_CH1 = VSET_CH1 / (V_MAX / VREF) * CH1_OUT_V2;
  else if (VSET_CH1 > 25.00)
    V_DAC_CH1 = VSET_CH1 / (V_MAX / VREF) * CH1_OUT_V3;
  SET_V_DAC_CH1 = constrain(4095 * (V_DAC_CH1 / VREF), 0, 4095);
  if (FLAG_OUT_CH1 && VSET_CH1 > 0)
  {
    DAC_Write(DAC_V_CH1, PWR_ON, SET_V_DAC_CH1);
  }
  else
    DAC_Write(DAC_V_CH1, PWR_OFF, 0);
  //---------I_SET - AJUSTE DE CORRENTE--------------------
  if (ISET_CH1 <= 1.000)
    I_DAC_CH1 = ISET_CH1 / (I_MAX / VREF) * CH1_OUT_I1;
  else if (ISET_CH1 > 1.000 && ISET_CH1 <= 2.000)
    I_DAC_CH1 = ISET_CH1 / (I_MAX / VREF) * CH1_OUT_I2;
  else if (ISET_CH1 > 2.000)
    I_DAC_CH1 = ISET_CH1 / (I_MAX / VREF) * CH1_OUT_I3;
  SET_I_DAC_CH1 = constrain(4095 * (I_DAC_CH1 / VREF) + (CH1_OFFSET * 1000), 0, 4095);
  if (ISET_CH1 > 0)
  {
    DAC_Write(DAC_I_CH1, PWR_ON, SET_I_DAC_CH1);
  }
  else
    DAC_Write(DAC_I_CH1, PWR_OFF, 0);

  //------------------CH2----------------------------------
  TCA9548A(CH2);
  //---------V_SET - AJUSTE DE TENSAO----------------------
  if (VSET_CH2 <= 10.00)
    V_DAC_CH2 = VSET_CH2 / (V_MAX / VREF) * CH2_OUT_V1;
  else if (VSET_CH2 > 10.00 && VSET_CH2 <= 25.00)
    V_DAC_CH2 = VSET_CH2 / (V_MAX / VREF) * CH2_OUT_V2;
  else if (VSET_CH2 > 25.00)
    V_DAC_CH2 = VSET_CH2 / (V_MAX / VREF) * CH2_OUT_V3;
  SET_V_DAC_CH2 = constrain(4095 * (V_DAC_CH2 / VREF), 0, 4095);
  if (FLAG_OUT_CH2 && VSET_CH2 > 0)
  {
    DAC_Write(DAC_V_CH2, PWR_ON, SET_V_DAC_CH2);
  }
  else
    DAC_Write(DAC_V_CH2, PWR_OFF, 0);
  //---------I_SET - AJUSTE DE CORRENTE--------------------
  if (ISET_CH2 <= 1.000)
    I_DAC_CH2 = ISET_CH2 / (I_MAX / VREF) * CH2_OUT_I1;
  else if (ISET_CH2 > 1.000 && ISET_CH2 <= 2.000)
    I_DAC_CH2 = ISET_CH2 / (I_MAX / VREF) * CH2_OUT_I2;
  else if (ISET_CH2 > 2.000)
    I_DAC_CH2 = ISET_CH2 / (I_MAX / VREF) * CH2_OUT_I3;
  SET_I_DAC_CH2 = constrain(4095 * (I_DAC_CH2 / VREF) + (CH2_OFFSET * 1000), 0, 4095);
  if (ISET_CH2 > 0)
  {
    DAC_Write(DAC_I_CH2, PWR_ON, SET_I_DAC_CH2);
  }
  else
    DAC_Write(DAC_I_CH2, PWR_OFF, 0);

  //------------------CH3----------------------------------
  TCA9548A(CH3);
  //---------V_SET - AJUSTE DE TENSAO----------------------
  V_DAC_CH3 = VSET_CH3 / (6.2 / VREF) * CH3_OUT_V1;
  SET_V_DAC_CH3 = constrain(4095 * (V_DAC_CH3 / VREF), 0, 4095);
  if (FLAG_OUT_CH3 && VSET_CH3 > 0)
  {
    DAC_Write(DAC_V_CH3, PWR_ON, SET_V_DAC_CH3);
  }
  else
    DAC_Write(DAC_V_CH3, PWR_OFF, 0);
  //---------I_SET - AJUSTE DE CORRENTE--------------------
  I_DAC_CH3 = ISET_CH3 / (I_MAX / VREF) * CH3_OUT_I1;
  SET_I_DAC_CH3 = constrain(4095 * (I_DAC_CH3 / VREF) + (CH3_OFFSET * 1000), 0, 4095);
  if (ISET_CH3 > 0)
  {
    DAC_Write(DAC_I_CH3, PWR_ON, SET_I_DAC_CH3);
  }
  else
    DAC_Write(DAC_I_CH3, PWR_OFF, 0);

} // SET_OUT

void STORE_VALUES()
{
  EEPROM.put(50, VSET_CH1);
  EEPROM.put(55, ISET_CH1);
  EEPROM.put(60, VSET_CH2);
  EEPROM.put(65, ISET_CH2);
  EEPROM.put(70, VSET_CH3);
  EEPROM.put(75, ISET_CH3);

  EEPROM.commit();
}

void CALL_VALUES()
{

  EEPROM.get(50, VSET_CH1);
  EEPROM.get(55, ISET_CH1);
  EEPROM.get(60, VSET_CH2);
  EEPROM.get(65, ISET_CH2);
  EEPROM.get(70, VSET_CH3);
  EEPROM.get(75, ISET_CH3);

} // CALL VALUES

void STORE_CONFIG()
{

//  EEPROM.put(11, FLAG_TRACK);
  EEPROM.put(12, FLAG_BUZZER);
  EEPROM.put(13, OTP);
  EEPROM.put(14, TEMP_MIN);
  EEPROM.put(15, FAN_PWM_MIN);  

  // CH1
  EEPROM.put(1100, OVP_CH1);
  EEPROM.put(1105, OCP_CH1);
  EEPROM.put(1110, VMAX_CH1);
  EEPROM.put(1115, TRIGGER_CH1);
  EEPROM.put(1116, FLAG_BOUT_CH1);
  EEPROM.put(1117, DELAY_CH1);

  // CH2
  EEPROM.put(1140, OVP_CH2);
  EEPROM.put(1145, OCP_CH2);
  EEPROM.put(1150, VMAX_CH2);
  EEPROM.put(1155, TRIGGER_CH2);
  EEPROM.put(1156, FLAG_BOUT_CH2);
  EEPROM.put(1157, DELAY_CH2);  

  // CH3
  EEPROM.put(1170, OVP_CH3);
  EEPROM.put(1175, OCP_CH3);
  EEPROM.put(1180, VMAX_CH3);
  EEPROM.put(1185, TRIGGER_CH3);
  EEPROM.put(1186, FLAG_BOUT_CH3);  
  EEPROM.put(1187, DELAY_CH3);

  EEPROM.commit();

} // STORE CONFIG

void RESTORE_CONFIG()
{

//  EEPROM.get(11, FLAG_TRACK);
  EEPROM.get(12, FLAG_BUZZER);
  EEPROM.get(13, OTP);
  EEPROM.get(14, TEMP_MIN);
  EEPROM.get(15, FAN_PWM_MIN);

  EEPROM.get(1100, OVP_CH1);
  EEPROM.get(1105, OCP_CH1);
  EEPROM.get(1110, VMAX_CH1);
  EEPROM.get(1115, TRIGGER_CH1);
  EEPROM.get(1116, FLAG_BOUT_CH1);  
  EEPROM.get(1117, DELAY_CH1);   

  EEPROM.get(1140, OVP_CH2);
  EEPROM.get(1145, OCP_CH2);
  EEPROM.get(1150, VMAX_CH2);
  EEPROM.get(1155, TRIGGER_CH2);
  EEPROM.get(1156, FLAG_BOUT_CH2);  
  EEPROM.get(1157, DELAY_CH2);     

  EEPROM.get(1170, OVP_CH3);
  EEPROM.get(1175, OCP_CH3);
  EEPROM.get(1180, VMAX_CH3);
  EEPROM.get(1185, TRIGGER_CH3);
  EEPROM.get(1186, FLAG_BOUT_CH3); 
  EEPROM.get(1187, DELAY_CH3);   

} // RESTORE CONFIG

void CALL_MEMORY()
{

  EEPROM.get(400, MEMORY_VSET_CH1[1]);
  EEPROM.get(405, MEMORY_ISET_CH1[1]);
  EEPROM.get(410, MEMORY_VSET_CH1[2]);
  EEPROM.get(415, MEMORY_ISET_CH1[2]);
  EEPROM.get(420, MEMORY_VSET_CH1[3]);
  EEPROM.get(425, MEMORY_ISET_CH1[3]);
  EEPROM.get(430, MEMORY_VSET_CH1[4]);
  EEPROM.get(435, MEMORY_ISET_CH1[4]);
  EEPROM.get(440, MEMORY_VSET_CH1[5]);
  EEPROM.get(445, MEMORY_ISET_CH1[5]);
  EEPROM.get(450, MEMORY_VSET_CH1[6]);
  EEPROM.get(455, MEMORY_ISET_CH1[6]);
  EEPROM.get(460, MEMORY_VSET_CH1[7]);
  EEPROM.get(465, MEMORY_ISET_CH1[7]);
  EEPROM.get(470, MEMORY_VSET_CH1[8]);
  EEPROM.get(475, MEMORY_ISET_CH1[8]);
  EEPROM.get(480, MEMORY_VSET_CH1[9]);
  EEPROM.get(485, MEMORY_ISET_CH1[9]);

  EEPROM.get(500, MEMORY_VSET_CH2[1]);
  EEPROM.get(505, MEMORY_ISET_CH2[1]);
  EEPROM.get(510, MEMORY_VSET_CH2[2]);
  EEPROM.get(515, MEMORY_ISET_CH2[2]);
  EEPROM.get(520, MEMORY_VSET_CH2[3]);
  EEPROM.get(525, MEMORY_ISET_CH2[3]);
  EEPROM.get(530, MEMORY_VSET_CH2[4]);
  EEPROM.get(535, MEMORY_ISET_CH2[4]);
  EEPROM.get(540, MEMORY_VSET_CH2[5]);
  EEPROM.get(545, MEMORY_ISET_CH2[5]);
  EEPROM.get(550, MEMORY_VSET_CH2[6]);
  EEPROM.get(555, MEMORY_ISET_CH2[6]);
  EEPROM.get(560, MEMORY_VSET_CH2[7]);
  EEPROM.get(565, MEMORY_ISET_CH2[7]);
  EEPROM.get(570, MEMORY_VSET_CH2[8]);
  EEPROM.get(575, MEMORY_ISET_CH2[8]);
  EEPROM.get(580, MEMORY_VSET_CH2[9]);
  EEPROM.get(585, MEMORY_ISET_CH2[9]);

  EEPROM.get(600, MEMORY_VSET_CH3[1]);
  EEPROM.get(605, MEMORY_ISET_CH3[1]);
  EEPROM.get(610, MEMORY_VSET_CH3[2]);
  EEPROM.get(615, MEMORY_ISET_CH3[2]);
  EEPROM.get(620, MEMORY_VSET_CH3[3]);
  EEPROM.get(625, MEMORY_ISET_CH3[3]);
  EEPROM.get(630, MEMORY_VSET_CH3[4]);
  EEPROM.get(635, MEMORY_ISET_CH3[4]);
  EEPROM.get(640, MEMORY_VSET_CH3[5]);
  EEPROM.get(645, MEMORY_ISET_CH3[5]);
  EEPROM.get(650, MEMORY_VSET_CH3[6]);
  EEPROM.get(655, MEMORY_ISET_CH3[6]);

  EEPROM.get(700, MEMORY_VSET_ALL[1]);
  EEPROM.get(705, MEMORY_ISET_ALL[1]);
  EEPROM.get(710, MEMORY_VSET_ALL[2]);
  EEPROM.get(715, MEMORY_ISET_ALL[2]);
  EEPROM.get(720, MEMORY_VSET_ALL[3]);
  EEPROM.get(725, MEMORY_ISET_ALL[3]);
  EEPROM.get(730, MEMORY_VSET_ALL[4]);
  EEPROM.get(735, MEMORY_ISET_ALL[4]);
  EEPROM.get(740, MEMORY_VSET_ALL[5]);
  EEPROM.get(745, MEMORY_ISET_ALL[5]);
  EEPROM.get(750, MEMORY_VSET_ALL[6]);
  EEPROM.get(755, MEMORY_ISET_ALL[6]);
  EEPROM.get(760, MEMORY_VSET_ALL[7]);
  EEPROM.get(765, MEMORY_ISET_ALL[7]);
  EEPROM.get(770, MEMORY_VSET_ALL[8]);
  EEPROM.get(775, MEMORY_ISET_ALL[8]);
  EEPROM.get(780, MEMORY_VSET_ALL[9]);
  EEPROM.get(785, MEMORY_ISET_ALL[9]);
  EEPROM.get(790, MEMORY_VSET_ALL[10]);
  EEPROM.get(795, MEMORY_ISET_ALL[10]);
  EEPROM.get(800, MEMORY_VSET_ALL[11]);
  EEPROM.get(805, MEMORY_ISET_ALL[11]);
  EEPROM.get(810, MEMORY_VSET_ALL[12]);
  EEPROM.get(815, MEMORY_ISET_ALL[12]);

} // VOID CALL MEMORY

void STORE_CALIBRATION()
{

  //----------------CH1-------------------
  EEPROM.put(100, CH1_METER_V);
  EEPROM.put(105, CH1_METER_I);
  EEPROM.put(110, CH1_OUT_V1);
  EEPROM.put(115, CH1_OUT_V2);
  EEPROM.put(120, CH1_OUT_V3);
  EEPROM.put(125, CH1_OUT_I1);
  EEPROM.put(130, CH1_OUT_I2);
  EEPROM.put(135, CH1_OUT_I3);
  EEPROM.put(140, CH1_OFFSET);

  //----------------CH2-------------------
  EEPROM.put(200, CH2_METER_V);
  EEPROM.put(205, CH2_METER_I);
  EEPROM.put(210, CH2_OUT_V1);
  EEPROM.put(215, CH2_OUT_V2);
  EEPROM.put(220, CH2_OUT_V3);
  EEPROM.put(225, CH2_OUT_I1);
  EEPROM.put(230, CH2_OUT_I2);
  EEPROM.put(235, CH2_OUT_I3);
  EEPROM.put(240, CH2_OFFSET);

  //----------------CH3-------------------
  EEPROM.put(300, CH3_METER_V);
  EEPROM.put(305, CH3_METER_I);
  EEPROM.put(310, CH3_OUT_V1);
  EEPROM.put(325, CH3_OUT_I1);
  EEPROM.put(340, CH3_OFFSET);

  EEPROM.commit();

} // STORE_CALIBRATION

void RESTORE_CALIBRATION()
{

  //----------------CH1-------------------
  EEPROM.get(100, CH1_METER_V);
  EEPROM.get(105, CH1_METER_I);
  EEPROM.get(110, CH1_OUT_V1);
  EEPROM.get(115, CH1_OUT_V2);
  EEPROM.get(120, CH1_OUT_V3);
  EEPROM.get(125, CH1_OUT_I1);
  EEPROM.get(130, CH1_OUT_I2);
  EEPROM.get(135, CH1_OUT_I3);
  EEPROM.get(140, CH1_OFFSET);

  //----------------CH2-------------------
  EEPROM.get(200, CH2_METER_V);
  EEPROM.get(205, CH2_METER_I);
  EEPROM.get(210, CH2_OUT_V1);
  EEPROM.get(215, CH2_OUT_V2);
  EEPROM.get(220, CH2_OUT_V3);
  EEPROM.get(225, CH2_OUT_I1);
  EEPROM.get(230, CH2_OUT_I2);
  EEPROM.get(235, CH2_OUT_I3);
  EEPROM.get(240, CH2_OFFSET);

  //----------------CH3-------------------
  EEPROM.get(300, CH3_METER_V);
  EEPROM.get(305, CH3_METER_I);
  EEPROM.get(310, CH3_OUT_V1);
  EEPROM.get(325, CH3_OUT_I1);
  EEPROM.get(340, CH3_OFFSET);

} // RESTORE_CALIBRATION

void FIRST_BOOT()
{

  DEFALUT_VALUES();

  EEPROM.put(11, FLAG_TRACK);
  EEPROM.put(12, FLAG_BUZZER);
  EEPROM.put(13, OTP);
  EEPROM.put(14, TEMP_MIN);
  EEPROM.put(15, FAN_PWM_MIN);  

  // CH1
  EEPROM.put(1100, OVP_CH1);
  EEPROM.put(1105, OCP_CH1);
  EEPROM.put(1110, VMAX_CH1);
  EEPROM.put(1115, TRIGGER_CH1);

  // CH2
  EEPROM.put(1140, OVP_CH2);
  EEPROM.put(1145, OCP_CH2);
  EEPROM.put(1150, VMAX_CH2);
  EEPROM.put(1155, TRIGGER_CH2);

  // CH3
  EEPROM.put(1170, OVP_CH3);
  EEPROM.put(1175, OCP_CH3);
  EEPROM.put(1180, VMAX_CH3);
  EEPROM.put(1185, TRIGGER_CH3);

  STORE_CALIBRATION();

  EEPROM.put(2040, 1); // FIRST BOOT OK
  EEPROM.commit();

} // FIRST BOOT

void DEFALUT_VALUES()
{
  /*

      VSET_CH1 = 1.0;
      ISET_CH1 = 1.0;
      VSET_CH2 = 1.0;
      ISET_CH2 = 1.0;
      VSET_CH3 = 1.0;
      ISET_CH3 = 1.0;

      CH1_METER_V = 1.0;
      CH1_METER_I = 1.0;
      CH1_OUT_V1 = 1.0;
      CH1_OUT_V2 = 1.0;
      CH1_OUT_V3 = 1.0;
      CH1_OUT_I1 = 1.0;
      CH1_OUT_I2 = 1.0;
      CH1_OUT_I3 = 1.0;
      CH1_OFFSET = 0.0;

      CH2_METER_V = 1.0;
      CH2_METER_I = 1.0;
      CH2_OUT_V1 = 1.0;
      CH2_OUT_V2 = 1.0;
      CH2_OUT_V3 = 1.0;
      CH2_OUT_I1 = 1.0;
      CH2_OUT_I2 = 1.0;
      CH2_OUT_I3 = 1.0;
      CH2_OFFSET = 0.0;

      CH3_METER_V = 1.0;
      CH3_METER_I = 1.0;
      CH3_OUT_V1 = 1.0;
      CH3_OUT_I1 = 1.0;
      CH3_OFFSET = 0.0;
      */


  FLAG_TRACK = 0;
  FLAG_BUZZER = 1;
  OTP = 60;
  TEMP_MIN = 35;
  FAN_PWM_MIN = 80;

  OVP_CH1 = 33.00;
  OCP_CH1 = 3.3;
  VMAX_CH1 = 32.00;
  TRIGGER_CH1 = 0;

  OVP_CH2 = 33.00;
  OCP_CH2 = 3.3;
  VMAX_CH2 = 32.00;
  TRIGGER_CH2 = 0;

  OVP_CH3 = 6.50;
  OCP_CH3 = 3.3;
  VMAX_CH3 = 6.50;
  TRIGGER_CH3 = 0;

} // DEFALT VALUES

void DrawRectCH1()
{
  tft.drawRect(0, 0, 160, 285, COLOR1);
  tft.drawRect(1, 1, 158, 283, COLOR1);
  tft.drawRect(2, 2, 156, 281, COLOR1);
}
void DrawRectCH2()
{
  tft.drawRect(160, 0, 160, 285, COLOR2);
  tft.drawRect(161, 1, 158, 283, COLOR2);
  tft.drawRect(162, 2, 156, 281, COLOR2);
}
void DrawRectCH3()
{
  tft.drawRect(320, 0, 160, 285, COLOR3);
  tft.drawRect(321, 1, 158, 283, COLOR3);
  tft.drawRect(322, 2, 156, 281, COLOR3);
}
void ClearRectCH1()
{
  //  tft.drawRect( 0, 0, 160, 285,  TFT_BLACK);
  tft.drawRect(1, 1, 158, 283, TFT_BLACK);
  tft.drawRect(2, 2, 156, 281, TFT_BLACK);
}
void ClearRectCH2()
{
  //  tft.drawRect(160, 0, 160, 285,  TFT_BLACK);
  tft.drawRect(161, 1, 158, 283, TFT_BLACK);
  tft.drawRect(162, 2, 156, 281, TFT_BLACK);
}
void ClearRectCH3()
{
  //  tft.drawRect(320, 0, 160, 285,  TFT_BLACK);
  tft.drawRect(321, 1, 158, 283, TFT_BLACK);
  tft.drawRect(322, 2, 156, 281, TFT_BLACK);
}

void DISPLAY_PRINCIPAL()
{

  if (FLAG_SELECT == 1)
  {

    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
    //    tft.drawRect(  0, 0, 160, 285,  TFT_BLACK);     //ch1
    DrawRectCH1();
    ClearRectCH2();
    ClearRectCH3();

    if (ENCODER_ADJ)
    {
      if (FLAG_VSET)
      {

        if (FLAG_DIGIT_V == 3)
        {
          tft.drawLine(64, 246, 74, 246, TFT_BLACK);
          tft.drawLine(64, 247, 74, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 2)
        {
          tft.drawLine(78, 246, 88, 246, TFT_BLACK);
          tft.drawLine(78, 247, 88, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 1)
        {
          tft.drawLine(99, 246, 109, 246, TFT_BLACK);
          tft.drawLine(99, 247, 109, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 0)
        {
          tft.drawLine(113, 246, 123, 246, TFT_BLACK);
          tft.drawLine(113, 247, 123, 247, TFT_BLACK);
        }
      }
      else if (FLAG_ISET)
      {

        if (FLAG_DIGIT_A == 3)
        {
          tft.drawLine(64, 272, 74, 272, TFT_BLACK);
          tft.drawLine(64, 273, 74, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 2)
        {
          tft.drawLine(85, 272, 95, 272, TFT_BLACK);
          tft.drawLine(85, 273, 95, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 1)
        {
          tft.drawLine(99, 272, 109, 272, TFT_BLACK);
          tft.drawLine(99, 273, 109, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 0)
        {
          tft.drawLine(113, 272, 123, 272, TFT_BLACK);
          tft.drawLine(113, 273, 123, 273, TFT_BLACK);
        }
      }
      else
      {
      }
    } // ENCODER_ADJ
  }
  else if (FLAG_SELECT == 2)
  {
    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
    //    tft.drawRect(160, 0, 160, 285,  TFT_BLACK);     //ch2
    DrawRectCH2();
    ClearRectCH1();
    ClearRectCH3();

    if (ENCODER_ADJ)
    {
      if (FLAG_VSET)
      {

        if (FLAG_DIGIT_V == 3)
        {
          tft.drawLine(224, 246, 234, 246, TFT_BLACK);
          tft.drawLine(224, 247, 234, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 2)
        {
          tft.drawLine(238, 246, 248, 246, TFT_BLACK);
          tft.drawLine(238, 247, 248, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 1)
        {
          tft.drawLine(259, 246, 269, 246, TFT_BLACK);
          tft.drawLine(259, 247, 269, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 0)
        {
          tft.drawLine(273, 246, 283, 246, TFT_BLACK);
          tft.drawLine(273, 247, 283, 247, TFT_BLACK);
        }
      }
      else if (FLAG_ISET)
      {

        if (FLAG_DIGIT_A == 3)
        {
          tft.drawLine(224, 272, 234, 272, TFT_BLACK);
          tft.drawLine(224, 273, 234, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 2)
        {
          tft.drawLine(245, 272, 255, 272, TFT_BLACK);
          tft.drawLine(245, 273, 255, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 1)
        {
          tft.drawLine(259, 272, 269, 272, TFT_BLACK);
          tft.drawLine(259, 273, 269, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 0)
        {
          tft.drawLine(273, 272, 283, 272, TFT_BLACK);
          tft.drawLine(273, 273, 283, 273, TFT_BLACK);
        }
      }
      else
      {
      }
    } // ENCODER_ADJ
  }
  else if (FLAG_SELECT == 3)
  {
    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    DrawRectCH3();
    ClearRectCH1();
    ClearRectCH2();
    //    tft.drawRect( 10, 218, 140, 60, TFT_BLACK);
    //    tft.drawRect(170, 218, 140, 60, TFT_BLACK);

    if (ENCODER_ADJ)
    {
      if (FLAG_VSET)
      {
        //      tft.drawRect(330, 218, 140, 60, COLOR3);
        if (FLAG_DIGIT_V == 3)
        {
          tft.drawLine(398, 246, 408, 246, TFT_BLACK);
          tft.drawLine(398, 247, 408, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 2)
        {
          tft.drawLine(398, 246, 408, 246, TFT_BLACK);
          tft.drawLine(398, 247, 408, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 1)
        {
          tft.drawLine(419, 246, 429, 246, TFT_BLACK);
          tft.drawLine(419, 247, 429, 247, TFT_BLACK);
        }
        else if (FLAG_DIGIT_V == 0)
        {
          tft.drawLine(433, 246, 443, 246, TFT_BLACK);
          tft.drawLine(433, 247, 443, 247, TFT_BLACK);
        }
      }
      else if (FLAG_ISET)
      {
        //      tft.drawRect(330, 218, 140, 60, COLOR3);
        if (FLAG_DIGIT_A == 3)
        {
          tft.drawLine(384, 272, 394, 272, TFT_BLACK);
          tft.drawLine(384, 273, 394, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 2)
        {
          tft.drawLine(405, 272, 415, 272, TFT_BLACK);
          tft.drawLine(405, 273, 415, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 1)
        {
          tft.drawLine(419, 272, 429, 272, TFT_BLACK);
          tft.drawLine(419, 273, 429, 273, TFT_BLACK);
        }
        else if (FLAG_DIGIT_A == 0)
        {
          tft.drawLine(433, 272, 443, 272, TFT_BLACK);
          tft.drawLine(433, 273, 443, 273, TFT_BLACK);
        }
      }
      else
      {
      }
    } // ENCODER_ADJ
  }
  else
  {
    ClearRectCH1();
    ClearRectCH2();
    ClearRectCH3();

    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
  }

  // LINHAS HORIZONTAIS
  tft.drawLine(3, 36, 156, 36, TFT_DARKGREY);
  tft.drawLine(163, 36, 316, 36, TFT_DARKGREY);
  tft.drawLine(323, 36, 476, 36, TFT_DARKGREY);

  tft.drawLine(3, 215, 156, 215, TFT_DARKGREY);
  tft.drawLine(163, 215, 316, 215, TFT_DARKGREY);
  tft.drawLine(323, 215, 476, 215, TFT_DARKGREY);

  //----------------------TOPO DA TELA---------------------------

  //--------------------------------CH1-------------------------------------

  tft.setCursor(12, 7);
  tft.setTextFont(4);

  tft.fillRoundRect(9, 4, 20, 28, 4, COLOR1);
  //  tft.fillRect(9, 4, 20, 28, COLOR1);
  tft.setTextColor(TFT_BLACK);
  tft.println("1");

  tft.setCursor(53, 7);
  if (FLAG_OUT_CH1)
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.println("ON   ");
    tft.setCursor(110, 7);
    if (FLAG_CC_CH1)
      tft.println("CC ");
    else
      tft.println("CV ");
  }
  else if (FLAG_OVP_CH1)
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.println("OVP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else if (FLAG_OCP_CH1)
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.println("OCP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.println("OFF          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }

  if (FLAG_OUT_CH1)
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
  }
  else
  {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }

  tft.setCursor(134, 50);
  tft.println("V");
  tft.setCursor(134, 105);
  tft.println("A");
  tft.setCursor(131, 160);
  tft.println("W");

  tft.setTextDatum(BR_DATUM);

  if (VOUT_CH1 < 10.0)
  {
    tft.setCursor(8, 52);
    tft.setTextFont(6);
    tft.println("0");
    tft.drawFloat(VOUT_CH1, 2, 131, 100, 6);
  }
  else
  {
    tft.drawFloat(VOUT_CH1, 2, 131, 100, 6);
  }

  tft.drawFloat(IOUT_CH1, 3, 131, 155, 6);

  if (POUT_CH1 < 10.00)
  {
    tft.setCursor(8, 162);
    tft.setTextFont(6);
    tft.println("0");
    tft.drawFloat(POUT_CH1, 2, 131, 210, 6);
  }
  else if (POUT_CH1 >= 10.00 && POUT_CH1 < 100.00)
  {
    tft.drawFloat(POUT_CH1, 2, 131, 210, 6);
  }
  else
  {
    tft.drawFloat(POUT_CH1, 1, 131, 210, 6);
  }

  if (FLAG_VSET && FLAG_SELECT == 1)
    tft.setTextColor(TFT_BLACK, COLOR1);
  else
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.drawFloat(VSET_CH1, 2, 125, 250, 4);
  if (VSET_CH1 < 10.00)
  {
    tft.setCursor(62, 224);
    tft.setTextFont(4);
    tft.println("0");
  }

  if (FLAG_ISET && FLAG_SELECT == 1)
    tft.setTextColor(TFT_BLACK, COLOR1);
  else
    tft.setTextColor(COLOR1, TFT_BLACK);
  tft.drawFloat(ISET_CH1, 3, 125, 275, 4);

  tft.setTextFont(4);
  tft.setCursor(15, 236);
  if (FLAG_DIGIT && FLAG_SELECT == 1)
  {
    tft.setTextColor(TFT_BLACK, COLOR1);
    tft.println("Set");
  }
  else
  {
    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.println("Set");
  }
  tft.setTextColor(COLOR1, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(130, 225);
  tft.println("V");
  tft.setCursor(130, 250);
  tft.println("A");

  //--------------------------------CH2-------------------------------------
  tft.setCursor(172, 7);
  tft.setTextFont(4);

  tft.fillRoundRect(169, 4, 20, 28, 4, COLOR2);
  tft.setTextColor(TFT_BLACK);
  tft.println("2");

  tft.setCursor(213, 7);
  if (FLAG_OUT_CH2)
  {
    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.println("ON   ");
    tft.setCursor(270, 7);
    if (FLAG_CC_CH2)
      tft.println("CC ");
    else
      tft.println("CV ");
  }
  else if (FLAG_OVP_CH2)
  {
    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.println("OVP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else if (FLAG_OCP_CH2)
  {
    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.println("OCP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else
  {
    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.println("OFF          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }

  tft.setCursor(294, 50);
  tft.println("V");
  tft.setCursor(294, 105);
  tft.println("A");
  tft.setCursor(291, 160);
  tft.println("W");

  if (VOUT_CH2 < 10.0)
  {
    tft.setCursor(168, 52);
    tft.setTextFont(6);
    tft.println("0");
    tft.drawFloat(VOUT_CH2, 2, 291, 100, 6);
  }
  else
  {
    tft.drawFloat(VOUT_CH2, 2, 291, 100, 6);
  }

  tft.drawFloat(IOUT_CH2, 3, 291, 155, 6);

  if (POUT_CH2 < 10.00)
  {
    tft.setCursor(168, 162);
    tft.setTextFont(6);
    tft.println("0");
    tft.drawFloat(POUT_CH2, 2, 291, 210, 6);
  }
  else if (POUT_CH2 >= 10.00 && POUT_CH2 < 100.00)
  {
    tft.drawFloat(POUT_CH2, 2, 291, 210, 6);
  }
  else
  {
    tft.drawFloat(POUT_CH2, 1, 291, 210, 6);
  }

  if (FLAG_VSET && FLAG_SELECT == 2)
    tft.setTextColor(TFT_BLACK, COLOR2);
  else
    tft.setTextColor(COLOR2, TFT_BLACK);
  if (VSET_CH2 < 10.00)
  {
    tft.setCursor(222, 224);
    tft.setTextFont(4);
    tft.println("0");
  }
  tft.drawFloat(VSET_CH2, 2, 285, 250, 4);

  if (FLAG_ISET && FLAG_SELECT == 2)
    tft.setTextColor(TFT_BLACK, COLOR2);
  else
    tft.setTextColor(COLOR2, TFT_BLACK);
  tft.drawFloat(ISET_CH2, 3, 285, 275, 4);

  tft.setTextFont(4);
  tft.setCursor(175, 236);
  if (FLAG_DIGIT && FLAG_SELECT == 2)
  {
    tft.setTextColor(TFT_BLACK, COLOR2);
    tft.println("Set");
  }
  else
  {
    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.println("Set");
  }
  tft.setTextColor(COLOR2, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(290, 225);
  tft.println("V");
  tft.setCursor(290, 250);
  tft.println("A");

  //--------------------------------CH3-------------------------------------

  tft.setCursor(334, 7);
  tft.setTextFont(4);

  tft.fillRoundRect(331, 4, 20, 28, 4, COLOR3);
  tft.setTextColor(TFT_BLACK);
  tft.println("3");

  tft.setCursor(375, 7);
  if (FLAG_OUT_CH3)
  {
    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.println("ON   ");
    tft.setCursor(432, 7);
    if (FLAG_CC_CH3)
      tft.println("CC ");
    else
      tft.println("CV ");
  }
  else if (FLAG_OVP_CH3)
  {
    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.println("OVP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else if (FLAG_OCP_CH3)
  {
    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.println("OCP          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  else
  {
    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.println("OFF          ");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }

  tft.setCursor(454, 50);
  tft.println("V");
  tft.setCursor(454, 105);
  tft.println("A");
  tft.setCursor(451, 160);
  tft.println("W");

  tft.drawFloat(VOUT_CH3, 2, 451, 100, 6);
  tft.drawFloat(IOUT_CH3, 3, 451, 155, 6);
  tft.drawFloat(POUT_CH3, 2, 451, 210, 6);
  if (POUT_CH3 < 10.00)
  {
    tft.setCursor(328, 162);
    tft.setTextFont(6);
    tft.println("0");
  }

  if (FLAG_VSET && FLAG_SELECT == 3)
    tft.setTextColor(TFT_BLACK, COLOR3);
  else
    tft.setTextColor(COLOR3, TFT_BLACK);
  tft.drawFloat(VSET_CH3, 2, 445, 250, 4);

  if (FLAG_ISET && FLAG_SELECT == 3)
    tft.setTextColor(TFT_BLACK, COLOR3);
  else
    tft.setTextColor(COLOR3, TFT_BLACK);
  tft.drawFloat(ISET_CH3, 3, 445, 275, 4);

  tft.setTextFont(4);
  tft.setCursor(335, 236);
  if (FLAG_DIGIT && FLAG_SELECT == 3)
  {
    tft.setTextColor(TFT_BLACK, COLOR3);
    tft.println("Set");
  }
  else
  {
    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.println("Set");
  }
  tft.setTextColor(COLOR3, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(450, 225);
  tft.println("V");
  tft.setCursor(450, 250);
  tft.println("A");

  //---------------------RODAPE DA TELA--------------------------

  tft.setTextFont(4);
  tft.setTextColor(COLOR4, TFT_BLACK);

  tft.setCursor(10, 293);
  if (FLAG_SHIFT)
    tft.print("Shift");
  else
    tft.print("             ");

  if (FLAG_TRACK)
  {
    tft.setCursor(130, 293);
    tft.print("Track");
    tft.drawLine(127, 302, 97, 302, COLOR4);
    tft.drawLine(97, 302, 97, 290, COLOR4);

    tft.drawLine(195, 302, 225, 302, COLOR4);
    tft.drawLine(225, 302, 225, 290, COLOR4);
  }
  else
  {
    tft.fillRect(96, 290, 130, 30, TFT_BLACK);
  }

  tft.setCursor(230, 293);
  if (FLAG_TRIGGER)
    tft.print("Trigger");
  else
    tft.print("                ");

  tft.setCursor(320, 293);
  if (FLAG_OTP)
    tft.print("OTP");
  else
    tft.print("          ");

  tft.setCursor(400, 293);
  if (FLAG_LOCK)
    tft.print("Lock");
  else
    tft.print("            ");    

} // DISPLAY PRINCIPAL

void DISPLAY_MENU()
{

  if (FLAG_SELECT == 0)
  {
    tft.setTextColor(COLOR4, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(300, 16);
    tft.print("Menu  Config");
    tft.setCursor(0, 30);  tft.print("1-Buzzer");  tft.setCursor(120, 30);  tft.print(":");
    tft.setCursor(0, 60);  tft.print("2-OTP");     tft.setCursor(120, 60);  tft.print(":");
    tft.setCursor(0, 90);  tft.print("3-Temp");    tft.setCursor(120, 90);  tft.print(":");
    tft.setCursor(0, 120); tft.print("4-PWM");     tft.setCursor(120, 120); tft.print(":");

    tft.setCursor(185, MENU_ID * 30);
    tft.print("<");
    if (MENU_ID > 1)
      tft.setCursor(185, (MENU_ID * 30) - 30);
    tft.print("   ");
    tft.setCursor(185, (MENU_ID * 30) + 30);
    tft.print("   ");

    tft.setCursor(135, 30);
    if (FLAG_BUZZER)
      tft.print("on   ");
    else
      tft.print("off");
    // tft.drawNumber(FLAG_BUZZER, 150, 56, 4);
    tft.drawNumber(OTP, 170, 86, 4);
    tft.drawNumber(TEMP_MIN, 170, 116, 4);
    if(FAN_PWM_MIN < 100){
      tft.setCursor(130, 120); 
      tft.print("  ");
    } 
    tft.drawNumber(FAN_PWM_MIN, 170, 146, 4);
  }
  else if (FLAG_SELECT == 1)
  {

    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(300, 16);
    tft.print("CH1 Menu");
    tft.setCursor(0, 30);  tft.print("1-OVP");  tft.setCursor(120, 30);  tft.print(":"); tft.setCursor(225, 30); tft.print("V");    
    tft.setCursor(0, 60);  tft.print("2-OCP");  tft.setCursor(120, 60); tft.print(":"); tft.setCursor(225, 60);  tft.print("A");
    tft.setCursor(0, 90);  tft.print("3-V_Max");tft.setCursor(120, 90);  tft.print(":"); tft.setCursor(225, 90); tft.print("V");    
    tft.setCursor(0, 120); tft.print("4-Trigger");tft.setCursor(120, 120);tft.print(":");
    tft.setCursor(0, 150); tft.print("5-B_Out");tft.setCursor(120, 150);tft.print(":");
    tft.setCursor(0, 180); tft.print("6-Delay");tft.setCursor(120, 180);tft.print(":");

    tft.setCursor(255, MENU_ID * 30);
    tft.print("<");
    if (MENU_ID > 1)
      tft.setCursor(255, (MENU_ID * 30) - 30);
    tft.print("   ");
    tft.setCursor(255, (MENU_ID * 30) + 30);
    tft.print("   ");

    tft.drawFloat(OVP_CH1, 2, 220, 56, 4);
    tft.drawFloat(OCP_CH1, 2, 220, 86, 4);
    tft.drawFloat(VMAX_CH1, 2, 220, 116, 4);

    tft.setCursor(155, 120);
    if (TRIGGER_CH1)
      tft.print("on   ");
    else
      tft.print("off");
  
    tft.setCursor(155, 150);
    if (FLAG_BOUT_CH1)
      tft.print("on   ");
    else
      tft.print("off");

    tft.drawNumber(DELAY_CH1, 220, 206, 4);

  }
  else if (FLAG_SELECT == 2)
  {

    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(300, 16);
    tft.print("CH2 Menu");
    tft.setCursor(0, 30);  tft.print("1-OVP");  tft.setCursor(120, 30);  tft.print(":"); tft.setCursor(225, 30); tft.print("V");    
    tft.setCursor(0, 60);  tft.print("2-OCP");  tft.setCursor(120, 60); tft.print(":"); tft.setCursor(225, 60);  tft.print("A");
    tft.setCursor(0, 90);  tft.print("3-V_Max");tft.setCursor(120, 90);  tft.print(":"); tft.setCursor(225, 90); tft.print("V");    
    tft.setCursor(0, 120); tft.print("4-Trigger");tft.setCursor(120, 120);tft.print(":");
    tft.setCursor(0, 150); tft.print("5-B_Out");tft.setCursor(120, 150);tft.print(":");  
    tft.setCursor(0, 180); tft.print("6-Delay");tft.setCursor(120, 180);tft.print(":");      

    tft.setCursor(255, MENU_ID * 30);
    tft.print("<");
    if (MENU_ID > 1)
      tft.setCursor(255, (MENU_ID * 30) - 30);
    tft.print("   ");
    tft.setCursor(255, (MENU_ID * 30) + 30);
    tft.print("   ");

    tft.drawFloat(OVP_CH2, 2, 220, 56, 4);
    tft.drawFloat(OCP_CH2, 2, 220, 86, 4);
    tft.drawFloat(VMAX_CH2, 2, 220, 116, 4);
    tft.setCursor(155, 120);
    if (TRIGGER_CH2)
      tft.print("on   ");
    else
      tft.print("off");
    // tft.drawNumber(TRIGGER_CH2, 200, 146, 4);
    tft.setCursor(155, 150);
    if (FLAG_BOUT_CH2)
      tft.print("on   ");
    else
      tft.print("off");    
    
    tft.drawNumber(DELAY_CH2, 220, 206, 4);

  }
  else if (FLAG_SELECT == 3)
  {

    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(300, 16);
    tft.print("CH3 Menu");
    tft.setCursor(0, 30);  tft.print("1-OVP");  tft.setCursor(120, 30);  tft.print(":"); tft.setCursor(225, 30); tft.print("V");    
    tft.setCursor(0, 60);  tft.print("2-OCP");  tft.setCursor(120, 60); tft.print(":"); tft.setCursor(225, 60);  tft.print("A");
    tft.setCursor(0, 90);  tft.print("3-V_Max");tft.setCursor(120, 90);  tft.print(":"); tft.setCursor(225, 90); tft.print("V");    
    tft.setCursor(0, 120); tft.print("4-Trigger");tft.setCursor(120, 120);tft.print(":");
    tft.setCursor(0, 150); tft.print("5-B_Out");tft.setCursor(120, 150);tft.print(":");
    tft.setCursor(0, 180); tft.print("6-Delay");tft.setCursor(120, 180);tft.print(":");        

    tft.setCursor(255, MENU_ID * 30);
    tft.print("<");
    if (MENU_ID > 1)
      tft.setCursor(255, (MENU_ID * 30) - 30);
    tft.print("   ");
    tft.setCursor(255, (MENU_ID * 30) + 30);
    tft.print("   ");

    tft.drawFloat(OVP_CH3, 2, 220, 56, 4);
    tft.drawFloat(OCP_CH3, 2, 220, 86, 4);
    tft.drawFloat(VMAX_CH3, 2, 220, 116, 4);
    tft.setCursor(165, 120);
    if (TRIGGER_CH3)
      tft.print("on  ");
    else
      tft.print("off");
    tft.setCursor(165, 150);
    if (FLAG_BOUT_CH3)
      tft.print("on   ");
    else
      tft.print("off");      

    tft.drawNumber(DELAY_CH3, 220, 206, 4);

  }

  tft.setCursor(10, 293);
  tft.setTextColor(COLOR4, TFT_BLACK);
  if (FLAG_SHIFT)
    tft.print("Shift");
  else
    tft.print("             ");

  tft.drawNumber(TEMP, 450, 320, 4);
  tft.drawCircle(460, 300, 3, COLOR4);

} // DISPLAY_MENU

void DISPLAY_MEMORIAS()
{

  if (FLAG_SELECT == 1)
  {

    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(125, 15);
    tft.print("CH1");

    if (MEMORY_MODE)
    {
      tft.setCursor(210, 15);
      tft.print("STORE         ");
    }
    else
    {
      tft.setCursor(210, 15);
      tft.print("RECALL        ");
    }

    tft.setCursor(100, 50);
    tft.print("M1");
    tft.setCursor(225, 50);
    tft.print("V");
    tft.setCursor(325, 50);
    tft.print("A");
    tft.setCursor(100, 76);
    tft.print("M2");
    tft.setCursor(225, 76);
    tft.print("V");
    tft.setCursor(325, 76);
    tft.print("A");
    tft.setCursor(100, 102);
    tft.print("M3");
    tft.setCursor(225, 102);
    tft.print("V");
    tft.setCursor(325, 102);
    tft.print("A");
    tft.setCursor(100, 128);
    tft.print("M4");
    tft.setCursor(225, 128);
    tft.print("V");
    tft.setCursor(325, 128);
    tft.print("A");
    tft.setCursor(100, 154);
    tft.print("M5");
    tft.setCursor(225, 154);
    tft.print("V");
    tft.setCursor(325, 154);
    tft.print("A");
    tft.setCursor(100, 180);
    tft.print("M6");
    tft.setCursor(225, 180);
    tft.print("V");
    tft.setCursor(325, 180);
    tft.print("A");
    tft.setCursor(100, 206);
    tft.print("M7");
    tft.setCursor(225, 206);
    tft.print("V");
    tft.setCursor(325, 206);
    tft.print("A");
    tft.setCursor(100, 232);
    tft.print("M8");
    tft.setCursor(225, 232);
    tft.print("V");
    tft.setCursor(325, 232);
    tft.print("A");
    tft.setCursor(100, 258);
    tft.print("M9");
    tft.setCursor(225, 258);
    tft.print("V");
    tft.setCursor(325, 258);
    tft.print("A");

    tft.drawFloat(MEMORY_VSET_CH1[1], 2, 220, 76, 4);
    tft.drawFloat(MEMORY_ISET_CH1[1], 3, 320, 76, 4);
    tft.drawFloat(MEMORY_VSET_CH1[2], 2, 220, 102, 4);
    tft.drawFloat(MEMORY_ISET_CH1[2], 3, 320, 102, 4);
    tft.drawFloat(MEMORY_VSET_CH1[3], 2, 220, 128, 4);
    tft.drawFloat(MEMORY_ISET_CH1[3], 3, 320, 128, 4);
    tft.drawFloat(MEMORY_VSET_CH1[4], 2, 220, 154, 4);
    tft.drawFloat(MEMORY_ISET_CH1[4], 3, 320, 154, 4);
    tft.drawFloat(MEMORY_VSET_CH1[5], 2, 220, 180, 4);
    tft.drawFloat(MEMORY_ISET_CH1[5], 3, 320, 180, 4);
    tft.drawFloat(MEMORY_VSET_CH1[6], 2, 220, 206, 4);
    tft.drawFloat(MEMORY_ISET_CH1[6], 3, 320, 206, 4);
    tft.drawFloat(MEMORY_VSET_CH1[7], 2, 220, 232, 4);
    tft.drawFloat(MEMORY_ISET_CH1[7], 3, 320, 232, 4);
    tft.drawFloat(MEMORY_VSET_CH1[8], 2, 220, 258, 4);
    tft.drawFloat(MEMORY_ISET_CH1[8], 3, 320, 258, 4);
    tft.drawFloat(MEMORY_VSET_CH1[9], 2, 220, 284, 4);
    tft.drawFloat(MEMORY_ISET_CH1[9], 3, 320, 284, 4);
  }
  else if (FLAG_SELECT == 2)
  {

    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(125, 15);
    tft.print("CH2");

    if (MEMORY_MODE)
    {
      tft.setCursor(210, 15);
      tft.print("STORE         ");
    }
    else
    {
      tft.setCursor(210, 15);
      tft.print("RECALL        ");
    }

    tft.setCursor(100, 50);
    tft.print("M1");
    tft.setCursor(225, 50);
    tft.print("V");
    tft.setCursor(325, 50);
    tft.print("A");
    tft.setCursor(100, 76);
    tft.print("M2");
    tft.setCursor(225, 76);
    tft.print("V");
    tft.setCursor(325, 76);
    tft.print("A");
    tft.setCursor(100, 102);
    tft.print("M3");
    tft.setCursor(225, 102);
    tft.print("V");
    tft.setCursor(325, 102);
    tft.print("A");
    tft.setCursor(100, 128);
    tft.print("M4");
    tft.setCursor(225, 128);
    tft.print("V");
    tft.setCursor(325, 128);
    tft.print("A");
    tft.setCursor(100, 154);
    tft.print("M5");
    tft.setCursor(225, 154);
    tft.print("V");
    tft.setCursor(325, 154);
    tft.print("A");
    tft.setCursor(100, 180);
    tft.print("M6");
    tft.setCursor(225, 180);
    tft.print("V");
    tft.setCursor(325, 180);
    tft.print("A");
    tft.setCursor(100, 206);
    tft.print("M7");
    tft.setCursor(225, 206);
    tft.print("V");
    tft.setCursor(325, 206);
    tft.print("A");
    tft.setCursor(100, 232);
    tft.print("M8");
    tft.setCursor(225, 232);
    tft.print("V");
    tft.setCursor(325, 232);
    tft.print("A");
    tft.setCursor(100, 258);
    tft.print("M9");
    tft.setCursor(225, 258);
    tft.print("V");
    tft.setCursor(325, 258);
    tft.print("A");

    tft.drawFloat(MEMORY_VSET_CH2[1], 2, 220, 76, 4);
    tft.drawFloat(MEMORY_ISET_CH2[1], 3, 320, 76, 4);
    tft.drawFloat(MEMORY_VSET_CH2[2], 2, 220, 102, 4);
    tft.drawFloat(MEMORY_ISET_CH2[2], 3, 320, 102, 4);
    tft.drawFloat(MEMORY_VSET_CH2[3], 2, 220, 128, 4);
    tft.drawFloat(MEMORY_ISET_CH2[3], 3, 320, 128, 4);
    tft.drawFloat(MEMORY_VSET_CH2[4], 2, 220, 154, 4);
    tft.drawFloat(MEMORY_ISET_CH2[4], 3, 320, 154, 4);
    tft.drawFloat(MEMORY_VSET_CH2[5], 2, 220, 180, 4);
    tft.drawFloat(MEMORY_ISET_CH2[5], 3, 320, 180, 4);
    tft.drawFloat(MEMORY_VSET_CH2[6], 2, 220, 206, 4);
    tft.drawFloat(MEMORY_ISET_CH2[6], 3, 320, 206, 4);
    tft.drawFloat(MEMORY_VSET_CH2[7], 2, 220, 232, 4);
    tft.drawFloat(MEMORY_ISET_CH2[7], 3, 320, 232, 4);
    tft.drawFloat(MEMORY_VSET_CH2[8], 2, 220, 258, 4);
    tft.drawFloat(MEMORY_ISET_CH2[8], 3, 320, 258, 4);
    tft.drawFloat(MEMORY_VSET_CH2[9], 2, 220, 284, 4);
    tft.drawFloat(MEMORY_ISET_CH2[9], 3, 320, 284, 4);
  }
  else if (FLAG_SELECT == 3)
  {

    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.setTextFont(4);
    tft.setCursor(125, 15);
    tft.print("CH3");

    if (MEMORY_MODE)
    {
      tft.setCursor(220, 15);
      tft.print("STORE         ");
    }
    else
    {
      tft.setCursor(220, 15);
      tft.print("RECALL        ");
    }

    tft.setCursor(100, 50);
    tft.print("M1");
    tft.setCursor(225, 50);
    tft.print("V");
    tft.setCursor(325, 50);
    tft.print("A");
    tft.setCursor(100, 76);
    tft.print("M2");
    tft.setCursor(225, 76);
    tft.print("V");
    tft.setCursor(325, 76);
    tft.print("A");
    tft.setCursor(100, 102);
    tft.print("M3");
    tft.setCursor(225, 102);
    tft.print("V");
    tft.setCursor(325, 102);
    tft.print("A");
    tft.setCursor(100, 128);
    tft.print("M4");
    tft.setCursor(225, 128);
    tft.print("V");
    tft.setCursor(325, 128);
    tft.print("A");
    tft.setCursor(100, 154);
    tft.print("M5");
    tft.setCursor(225, 154);
    tft.print("V");
    tft.setCursor(325, 154);
    tft.print("A");
    tft.setCursor(100, 180);
    tft.print("M6");
    tft.setCursor(225, 180);
    tft.print("V");
    tft.setCursor(325, 180);
    tft.print("A");

    tft.drawFloat(MEMORY_VSET_CH3[1], 2, 220, 76, 4);
    tft.drawFloat(MEMORY_ISET_CH3[1], 3, 320, 76, 4);
    tft.drawFloat(MEMORY_VSET_CH3[2], 2, 220, 102, 4);
    tft.drawFloat(MEMORY_ISET_CH3[2], 3, 320, 102, 4);
    tft.drawFloat(MEMORY_VSET_CH3[3], 2, 220, 128, 4);
    tft.drawFloat(MEMORY_ISET_CH3[3], 3, 320, 128, 4);
    tft.drawFloat(MEMORY_VSET_CH3[4], 2, 220, 154, 4);
    tft.drawFloat(MEMORY_ISET_CH3[4], 3, 320, 154, 4);
    tft.drawFloat(MEMORY_VSET_CH3[5], 2, 220, 180, 4);
    tft.drawFloat(MEMORY_ISET_CH3[5], 3, 320, 180, 4);
    tft.drawFloat(MEMORY_VSET_CH3[6], 2, 220, 206, 4);
    tft.drawFloat(MEMORY_ISET_CH3[6], 3, 320, 206, 4);
  }
  else
  {

    tft.setTextColor(COLOR4, TFT_BLACK);
    tft.setTextFont(4);

    if (MEMORY_MODE)
    {
      tft.setCursor(0, 15);
      tft.print("STORE    ");
    }
    else
    {
      tft.setCursor(0, 15);
      tft.print("RECALL   ");
    }

    tft.setCursor(40, 70);
    tft.print("M1");
    //    tft.drawLine(0, 108, 480, 108, TFT_DARKGREY);
    tft.setCursor(40, 130);
    tft.print("M2");
    //    tft.drawLine(0, 168, 480, 168, TFT_DARKGREY);
    tft.setCursor(40, 190);
    tft.print("M3");
    //    tft.drawLine(0, 228, 480, 228, TFT_DARKGREY);
    tft.setCursor(40, 250);
    tft.print("M4");

    tft.setTextColor(COLOR1, TFT_BLACK);
    tft.setCursor(150, 15);
    tft.print("CH1");
    tft.drawFloat(MEMORY_VSET_ALL[1], 2, 195, 81, 4);
    tft.setCursor(200, 57);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[1], 3, 195, 107, 4);
    tft.setCursor(200, 83);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[4], 2, 195, 141, 4);
    tft.setCursor(200, 117);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[4], 3, 195, 167, 4);
    tft.setCursor(200, 143);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[7], 2, 195, 201, 4);
    tft.setCursor(200, 177);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[7], 3, 195, 227, 4);
    tft.setCursor(200, 203);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[10], 2, 195, 261, 4);
    tft.setCursor(200, 237);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[10], 3, 195, 287, 4);
    tft.setCursor(200, 263);
    tft.print("A");

    tft.setTextColor(COLOR2, TFT_BLACK);
    tft.setCursor(270, 15);
    tft.print("CH2");
    tft.drawFloat(MEMORY_VSET_ALL[2], 2, 315, 81, 4);
    tft.setCursor(320, 57);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[2], 3, 315, 107, 4);
    tft.setCursor(320, 83);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[5], 2, 315, 141, 4);
    tft.setCursor(320, 117);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[5], 3, 315, 167, 4);
    tft.setCursor(320, 143);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[8], 2, 315, 201, 4);
    tft.setCursor(320, 177);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[8], 3, 315, 227, 4);
    tft.setCursor(320, 203);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[11], 2, 315, 261, 4);
    tft.setCursor(320, 237);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[11], 3, 315, 287, 4);
    tft.setCursor(320, 263);
    tft.print("A");

    tft.setTextColor(COLOR3, TFT_BLACK);
    tft.setCursor(390, 15);
    tft.print("CH3");
    tft.drawFloat(MEMORY_VSET_ALL[3], 2, 435, 81, 4);
    tft.setCursor(440, 57);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[3], 3, 435, 107, 4);
    tft.setCursor(440, 83);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[6], 2, 435, 141, 4);
    tft.setCursor(440, 117);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[6], 3, 435, 167, 4);
    tft.setCursor(440, 143);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[9], 2, 435, 201, 4);
    tft.setCursor(440, 177);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[9], 3, 435, 227, 4);
    tft.setCursor(440, 203);
    tft.print("A");
    tft.drawFloat(MEMORY_VSET_ALL[12], 2, 435, 261, 4);
    tft.setCursor(440, 237);
    tft.print("V");
    tft.drawFloat(MEMORY_ISET_ALL[12], 3, 435, 287, 4);
    tft.setCursor(440, 263);
    tft.print("A");
  }

  tft.setCursor(10, 293);
  tft.setTextColor(COLOR4, TFT_BLACK);
  if (FLAG_SHIFT)
    tft.print("Shift");
  else
    tft.print("             ");

} // MEMORY_DISPLAY

void DISPLAY_CALIBRACAO()
{

  if (FLAG_SELECT == 1)
  {
    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
    DrawRectCH1();
    ClearRectCH2();
    ClearRectCH3();
  }
  else if (FLAG_SELECT == 2)
  {
    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
    DrawRectCH2();
    ClearRectCH1();
    ClearRectCH3();
  }
  else if (FLAG_SELECT == 3)
  {
    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    DrawRectCH3();
    ClearRectCH1();
    ClearRectCH2();
  }
  else
  {
    ClearRectCH1();
    ClearRectCH2();
    ClearRectCH3();
    tft.drawRect(0, 0, 160, 285, TFT_DARKGREY);   // ch1
    tft.drawRect(160, 0, 160, 285, TFT_DARKGREY); // ch2
    tft.drawRect(320, 0, 160, 285, TFT_DARKGREY); // ch3
  }

  //--------------------------------CH1-------------------------------------

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextFont(2);
  tft.setCursor(10, 20);
  tft.print("METER_V :");
  tft.drawFloat(CH1_METER_V, 4, 130, 20, 2);
  tft.setCursor(10, 40);
  tft.print("METER_I :");
  tft.drawFloat(CH1_METER_I, 4, 130, 40, 2);
  tft.setCursor(10, 60);
  tft.print("OUT_V1  :");
  tft.drawFloat(CH1_OUT_V1, 4, 130, 60, 2);
  tft.setCursor(10, 80);
  tft.print("OUT_V2  :");
  tft.drawFloat(CH1_OUT_V2, 4, 130, 80, 2);
  tft.setCursor(10, 100);
  tft.print("OUT_V3  :");
  tft.drawFloat(CH1_OUT_V3, 4, 130, 100, 2);
  tft.setCursor(10, 120);
  tft.print("OUT_I1  :");
  tft.drawFloat(CH1_OUT_I1, 4, 130, 120, 2);
  tft.setCursor(10, 140);
  tft.print("OUT_I2  :");
  tft.drawFloat(CH1_OUT_I2, 4, 130, 140, 2);
  tft.setCursor(10, 160);
  tft.print("OUT_I3  :");
  tft.drawFloat(CH1_OUT_I3, 4, 130, 160, 2);
  tft.setCursor(10, 180);
  tft.print("OFFSET  :");
  tft.drawFloat(CH1_OFFSET, 4, 130, 180, 2);

  tft.setCursor(138, CH1_ID_CAL * 20);
  tft.print("<");
  if (CH1_ID_CAL > 1)
    tft.setCursor(138, (CH1_ID_CAL * 20) - 20);
  tft.print("  ");
  tft.setCursor(138, (CH1_ID_CAL * 20) + 20);
  tft.print("  ");

  if (FLAG_OUT_CH1)
    tft.setTextColor(COLOR1, TFT_BLACK);
  else
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextDatum(BR_DATUM);
  tft.drawFloat(VOUT_CH1, 3, 125, 250, 4);
  if (VOUT_CH1 < 10.000)
  {
    tft.setCursor(48, 224);
    tft.setTextFont(4);
    tft.println("0");
  }
  tft.drawFloat(IOUT_CH1, 4, 125, 275, 4);
  tft.setTextFont(2);
  tft.setCursor(130, 225);
  tft.println("V");
  tft.setCursor(130, 250);
  tft.println("A");

  //--------------------------------CH2-------------------------------------

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextFont(2);
  tft.setCursor(170, 20);
  tft.print("METER_V :");
  tft.drawFloat(CH2_METER_V, 4, 290, 20, 2);
  tft.setCursor(170, 40);
  tft.print("METER_I :");
  tft.drawFloat(CH2_METER_I, 4, 290, 40, 2);
  tft.setCursor(170, 60);
  tft.print("OUT_V1  :");
  tft.drawFloat(CH2_OUT_V1, 4, 290, 60, 2);
  tft.setCursor(170, 80);
  tft.print("OUT_V2  :");
  tft.drawFloat(CH2_OUT_V2, 4, 290, 80, 2);
  tft.setCursor(170, 100);
  tft.print("OUT_V3  :");
  tft.drawFloat(CH2_OUT_V3, 4, 290, 100, 2);
  tft.setCursor(170, 120);
  tft.print("OUT_I1  :");
  tft.drawFloat(CH2_OUT_I1, 4, 290, 120, 2);
  tft.setCursor(170, 140);
  tft.print("OUT_I2  :");
  tft.drawFloat(CH2_OUT_I2, 4, 290, 140, 2);
  tft.setCursor(170, 160);
  tft.print("OUT_I3  :");
  tft.drawFloat(CH2_OUT_I3, 4, 290, 160, 2);
  tft.setCursor(170, 180);
  tft.print("OFFSET  :");
  tft.drawFloat(CH2_OFFSET, 4, 290, 180, 2);

  tft.setCursor(298, CH2_ID_CAL * 20);
  tft.print("<");
  if (CH2_ID_CAL > 1)
    tft.setCursor(298, (CH2_ID_CAL * 20) - 20);
  tft.print("  ");
  tft.setCursor(298, (CH2_ID_CAL * 20) + 20);
  tft.print("  ");

  if (FLAG_OUT_CH2)
    tft.setTextColor(COLOR2, TFT_BLACK);
  else
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextDatum(BR_DATUM);
  if (VOUT_CH2 < 10.000)
  {
    tft.setCursor(208, 224);
    tft.setTextFont(4);
    tft.println("0");
  }
  tft.drawFloat(VOUT_CH2, 3, 285, 250, 4);
  tft.drawFloat(IOUT_CH2, 4, 285, 275, 4);
  tft.setTextFont(2);
  tft.setCursor(290, 225);
  tft.println("V");
  tft.setCursor(290, 250);
  tft.println("A");

  //--------------------------------CH3-------------------------------------

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextFont(2);
  tft.setCursor(330, 20);
  tft.print("METER_V :");
  tft.drawFloat(CH3_METER_V, 4, 450, 20, 2);
  tft.setCursor(330, 40);
  tft.print("METER_I :");
  tft.drawFloat(CH3_METER_I, 4, 450, 40, 2);
  tft.setCursor(330, 60);
  tft.print("OUT_V1  :");
  tft.drawFloat(CH3_OUT_V1, 4, 450, 60, 2);
  tft.setCursor(330, 80);
  tft.print("OUT_I1  :");
  tft.drawFloat(CH3_OUT_I1, 4, 450, 80, 2);
  tft.setCursor(330, 100);
  tft.print("OFFSET  :");
  tft.drawFloat(CH3_OFFSET, 4, 450, 100, 2);

  tft.setCursor(458, CH3_ID_CAL * 20);
  tft.print("<");
  if (CH3_ID_CAL > 1)
    tft.setCursor(458, (CH3_ID_CAL * 20) - 20);
  tft.print("  ");
  tft.setCursor(458, (CH3_ID_CAL * 20) + 20);
  tft.print("  ");

  if (FLAG_OUT_CH3)
    tft.setTextColor(COLOR3, TFT_BLACK);
  else
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextDatum(BR_DATUM);
  tft.drawFloat(VOUT_CH3, 3, 445, 250, 4);
  tft.drawFloat(IOUT_CH3, 4, 445, 275, 4);
  tft.setTextFont(2);
  tft.setCursor(450, 225);
  tft.println("V");
  tft.setCursor(450, 250);
  tft.println("A");

  //--------------------------------RODAPE--------------------------------

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);

  tft.setCursor(10, 293);
  if (FLAG_SHIFT)
    tft.print("Shift");
  else
    tft.print("             ");

  tft.drawFloat(X, 4, 440, 320, 4);

} // DISPLAY CALIBRACAO

void KEYBOARD_GLOBAL()
{

//  pos = encoder.getPosition();

  if (!FLAG_LOCK)
  {
    key = kpd.getKey();
    if (key)
      BUZZER(50);
  }

    // botao OUT
  if (!digitalRead(B_OUT) && flag_B_OUT == 0)
  {
    BUZZER(50);
                                        
      if(FLAG_BOUT_CH1){
        delay(DELAY_CH1);
        FLAG_OUT_CH1 = !FLAG_OUT_CH1;
        FLAG_OVP_CH1 = 0;
        FLAG_OCP_CH1 = 0;
        SET_OUT();
      }
      if(FLAG_BOUT_CH2){
        delay(DELAY_CH2);
        FLAG_OUT_CH2 = !FLAG_OUT_CH2;
        FLAG_OVP_CH2 = 0;
        FLAG_OCP_CH2 = 0;
        SET_OUT();
      }
      if(FLAG_BOUT_CH3){
        delay(DELAY_CH3);
        FLAG_OUT_CH3 = !FLAG_OUT_CH3;    
        FLAG_OVP_CH3 = 0;
        FLAG_OCP_CH3 = 0;
        SET_OUT();
      }
      STORE_VALUES();
      flag_B_OUT = 1;
  }
  else if (digitalRead(B_OUT) && flag_B_OUT == 1)
  {
    flag_B_OUT = 0;
  } // botao OUT

//EXT_TRIGGER
  if (!digitalRead(EXT_TRIG_PIN) && FLAG_TRIGGER == 0)
  {
    BUZZER(50);
    FLAG_TRIGGER = 1;
    if (FLAG_TRIGGER && TRIGGER_CH1)
      delay(DELAY_CH1);
      FLAG_OVP_CH1 = 0;
      FLAG_OCP_CH1 = 0;
      FLAG_OUT_CH1 = 1;
      SET_OUT();
    if (FLAG_TRIGGER && TRIGGER_CH2)
      delay(DELAY_CH2);
      FLAG_OVP_CH2 = 0;
      FLAG_OCP_CH2 = 0;
      FLAG_OUT_CH2 = 1;
      SET_OUT();
    if (FLAG_TRIGGER && TRIGGER_CH3)
      delay(DELAY_CH3);
      FLAG_OVP_CH3 = 0;
      FLAG_OCP_CH3 = 0;      
      FLAG_OUT_CH3 = 1;
      SET_OUT();
  }
  else if (digitalRead(EXT_TRIG_PIN) && FLAG_TRIGGER == 1)
  {
    if (FLAG_TRIGGER && TRIGGER_CH1)
      FLAG_OUT_CH1 = 0;
    if (FLAG_TRIGGER && TRIGGER_CH2)
      FLAG_OUT_CH2 = 0;
    if (FLAG_TRIGGER && TRIGGER_CH3)
      FLAG_OUT_CH3 = 0;
    SET_OUT();
    FLAG_TRIGGER = 0;
  } //EXT_TRIGGER


  if (key == 'A')
  {
    FLAG_SHIFT = !FLAG_SHIFT;
  }

  // liga / desliga Tracking
  if (key == '7' && FLAG_SHIFT && FLAG_SELECT == 0)
  {
    FLAG_TRACK = !FLAG_TRACK;
    FLAG_SHIFT = 0;
    SET_OUT();
  }

  if (!ENCODER_ADJ)
  {
    if (key == '*')
    {
      FLAG_SELECT--;
      if (FLAG_SELECT < 0)
        FLAG_SELECT = 3;
    }
    if (key == '#')
    {
      FLAG_SELECT++;
      if (FLAG_SELECT > 3)
        FLAG_SELECT = 0;
    }
  }

  // se o botao do encoder for pressionado
  if (!digitalRead(encoderSW))
  {
    // aciona lock mantendo pressionado botao do encoder
    unsigned long currentMillis1 = millis();
    if (currentMillis1 - previousMillis1 >= 300)
    {
      previousMillis1 = currentMillis1;
      TIMER++;
      if (TIMER >= 9)
      {
        FLAG_LOCK = !FLAG_LOCK;
        TIMER = 0;
        FLAG_SELECT = 0;
        ENCODER_ADJ = 0;
        FLAG_SHIFT = 0;
        BUZZER(100);
      }
    }
  }
  else
    TIMER = 0;

} // VOID KEYBOARD_GLOBAL

void KEYBOARD_PRINCIPAL()
{
  // liga / desliga as saidas
  if (key == '1' && FLAG_SHIFT)
  {
    FLAG_OUT_CH1 = !FLAG_OUT_CH1;
    FLAG_OVP_CH1 = 0;
    FLAG_OCP_CH1 = 0;
    SET_OUT();
    STORE_VALUES();
    FLAG_SHIFT = 0;
    key = 0;
  }
  if (key == '2' && FLAG_SHIFT)
  {
    FLAG_OUT_CH2 = !FLAG_OUT_CH2;
    FLAG_OVP_CH2 = 0;
    FLAG_OCP_CH2 = 0;
    SET_OUT();
    STORE_VALUES();
    FLAG_SHIFT = 0;
    key = 0;
  }
  if (key == '3' && FLAG_SHIFT)
  {
    FLAG_OUT_CH3 = !FLAG_OUT_CH3;
    FLAG_OVP_CH3 = 0;
    FLAG_OCP_CH3 = 0;
    SET_OUT();
    STORE_VALUES();
    FLAG_SHIFT = 0;
    key = 0;
  }

  // seleciona os canais
  if (key == '4' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 1)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 1;
    FLAG_SHIFT = 0;
    key = 0;
  }
  if (key == '5' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 2)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 2;
    FLAG_SHIFT = 0;
    key = 0;
  }
  if (key == '6' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 3)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 3;
    FLAG_SHIFT = 0;
    key = 0;
  }

  // entra menu de calibracao
  if (key == '0' && FLAG_SHIFT)
  {
    ID_DISPLAY = 1;
    ENCODER_ADJ = 1;
    FLAG_SHIFT = 0;
    FLAG_SELECT = 0;
    tft.fillScreen(TFT_BLACK);
  }
  // entra menu
  if (key == 'D' && FLAG_SHIFT)
  {
    ID_DISPLAY = 3;
    ENCODER_ADJ = 1;
    FLAG_SHIFT = 0;
    tft.fillScreen(TFT_BLACK);
  }

  // entra na tela das memorias
  if (key == 'B')
  {
    if (FLAG_SHIFT)
    {
      MEMORY_MODE = 1;
      FLAG_SHIFT = 0;
    }
    tft.fillScreen(TFT_BLACK);
    FLAG_SHIFT = 0;
    CALL_MEMORY();
    ID_DISPLAY = 2;
  }

  if (key == 'C')
  {
    FLAG_VSET = !FLAG_VSET;
    FLAG_ISET = !FLAG_ISET;
  }

  if (key == 'D')
  {
    SET_OUT();
    string = "";
    FLAG_DIGIT = 0;
  }

  if (!digitalRead(encoderSW) && flag_encoderSW == 0)
  { // botao do encoder
    BUZZER(50);
    ENCODER_ADJ = !ENCODER_ADJ;
    flag_encoderSW = 1;
  }
  else if (digitalRead(encoderSW) && flag_encoderSW == 1)
  {
    flag_encoderSW = 0;
  }

  switch (FLAG_SELECT)
  {

  case 1: // AJUSTES CANAL 1

    if (!FLAG_TRACK)
    {
      if (FLAG_VSET)
      {
        if (isDigit(key) && !FLAG_SHIFT)
        {
          FLAG_DIGIT = 1;
          string.concat(key);
          VSET_CH1 = string.toFloat() / 100;
          if (VSET_CH1 > VMAX_CH1)
            VSET_CH1 = 0;
          if (string.length() > 4)
          {
            string = "";
          }
        }

        if (ENCODER_ADJ)
        {
          if (key == '#')
          {
            if (FLAG_DIGIT_V > 0)
              FLAG_DIGIT_V--;
          }
          if (key == '*')
          {
            if (FLAG_DIGIT_V < 3)
              FLAG_DIGIT_V++;
          }
          if (pos != 0)
          {
            if (FLAG_DIGIT_V == 0)
              VSET_CH1 = readEncoder(VSET_CH1, 0.01, 0, VMAX_CH1);
            else if (FLAG_DIGIT_V == 1)
              VSET_CH1 = readEncoder(VSET_CH1, 0.1, 0, VMAX_CH1);
            else if (FLAG_DIGIT_V == 2)
              VSET_CH1 = readEncoder(VSET_CH1, 1, 0, VMAX_CH1);
            else if (FLAG_DIGIT_V == 3)
              VSET_CH1 = readEncoder(VSET_CH1, 10, 0, VMAX_CH1 - 2);
            SET_OUT();
          }
        } // ENCODER_ADJ
      }
      else if (FLAG_ISET)
      {
        if (isDigit(key) && !FLAG_SHIFT)
        {
          FLAG_DIGIT = 1;
          string.concat(key);
          ISET_CH1 = string.toFloat() / 1000;
          if (ISET_CH1 > I_MAX)
            ISET_CH1 = 0;
          if (string.length() > 4)
          {
            string = "";
          }
        }

        if (ENCODER_ADJ)
        {
          if (key == '#')
          {
            if (FLAG_DIGIT_A > 0)
              FLAG_DIGIT_A--;
          }
          if (key == '*')
          {
            if (FLAG_DIGIT_A < 3)
              FLAG_DIGIT_A++;
          }
          if (pos != 0)
          {
            if (FLAG_DIGIT_A == 0)
              ISET_CH1 = readEncoder(ISET_CH1, 0.001, 0, I_MAX);
            else if (FLAG_DIGIT_A == 1)
              ISET_CH1 = readEncoder(ISET_CH1, 0.01, 0, I_MAX);
            else if (FLAG_DIGIT_A == 2)
              ISET_CH1 = readEncoder(ISET_CH1, 0.1, 0, I_MAX);
            else if (FLAG_DIGIT_A == 3)
              ISET_CH1 = readEncoder(ISET_CH1, 1, 0, I_MAX - 0.2);
            SET_OUT();
          }
        } // ENCODER_ADJ

      }    // FLAG_ISET
    }      // FLAG_TRACK
    break; // FLAG_SELECT CASE 1

  case 2: // AJUSTES CANAL 2
    if (FLAG_VSET)
    {
      if (isDigit(key) && !FLAG_SHIFT)
      {
        FLAG_DIGIT = 1;
        string.concat(key);
        VSET_CH2 = string.toFloat() / 100;
        if (VSET_CH2 > VMAX_CH2)
          VSET_CH2 = 0;
        if (string.length() > 4)
        {
          string = "";
        }
      }

      if (ENCODER_ADJ)
      {
        if (key == '#')
        {
          if (FLAG_DIGIT_V > 0)
            FLAG_DIGIT_V--;
        }
        if (key == '*')
        {
          if (FLAG_DIGIT_V < 3)
            FLAG_DIGIT_V++;
        }
        if (pos != 0)
        {
          if (FLAG_DIGIT_V == 0)
            VSET_CH2 = readEncoder(VSET_CH2, 0.01, 0, VMAX_CH2);
          else if (FLAG_DIGIT_V == 1)
            VSET_CH2 = readEncoder(VSET_CH2, 0.1, 0, VMAX_CH2);
          else if (FLAG_DIGIT_V == 2)
            VSET_CH2 = readEncoder(VSET_CH2, 1, 0, VMAX_CH2);
          else if (FLAG_DIGIT_V == 3)
            VSET_CH2 = readEncoder(VSET_CH2, 10, 0, VMAX_CH2 - 2);
          SET_OUT();
        }
      } // ENCODER_ADJ
    }
    else if (FLAG_ISET)
    {
      if (isDigit(key) && !FLAG_SHIFT)
      {
        FLAG_DIGIT = 1;
        string.concat(key);
        ISET_CH2 = string.toFloat() / 1000;
        if (ISET_CH2 > I_MAX)
          ISET_CH2 = 0;
        if (string.length() > 4)
        {
          string = "";
        }
      }

      if (ENCODER_ADJ)
      {
        if (key == '#')
        {
          if (FLAG_DIGIT_A > 0)
            FLAG_DIGIT_A--;
        }
        if (key == '*')
        {
          if (FLAG_DIGIT_A < 3)
            FLAG_DIGIT_A++;
        }
        if (pos != 0)
        {
          if (FLAG_DIGIT_A == 0)
            ISET_CH2 = readEncoder(ISET_CH2, 0.001, 0, I_MAX);
          else if (FLAG_DIGIT_A == 1)
            ISET_CH2 = readEncoder(ISET_CH2, 0.01, 0, I_MAX);
          else if (FLAG_DIGIT_A == 2)
            ISET_CH2 = readEncoder(ISET_CH2, 0.1, 0, I_MAX);
          else if (FLAG_DIGIT_A == 3)
            ISET_CH2 = readEncoder(ISET_CH2, 1, 0, I_MAX - 0.2);
          SET_OUT();
        }
      } // ENCODER_ADJ
    }   // FLAG_ISET

    break; // FLAG_SELECT CASE 2

  case 3: // AJUSTES CANAL 3

    if (FLAG_VSET)
    {
      if (isDigit(key) && !FLAG_SHIFT)
      {
        FLAG_DIGIT = 1;
        string.concat(key);
        VSET_CH3 = string.toFloat() / 100;
        if (VSET_CH3 > VMAX_CH3)
          VSET_CH3 = 0;
        if (string.length() > 3)
        {
          string = "";
        }
      }
      if (ENCODER_ADJ)
      {
        if (key == '#')
        {
          if (FLAG_DIGIT_V > 0)
            FLAG_DIGIT_V--;
        }
        if (key == '*')
        {
          if (FLAG_DIGIT_V < 3)
            FLAG_DIGIT_V++;
        }

        if (pos != 0)
        {
          if (FLAG_DIGIT_V == 0)
            VSET_CH3 = readEncoder(VSET_CH3, 0.01, 0, VMAX_CH3);
          else if (FLAG_DIGIT_V == 1)
            VSET_CH3 = readEncoder(VSET_CH3, 0.1, 0, VMAX_CH3);
          else if (FLAG_DIGIT_V == 2)
            VSET_CH3 = readEncoder(VSET_CH3, 1, 0, VMAX_CH3 - 0.2);
          SET_OUT();
        }
      } // ENCODER_ADJ
    }
    else if (FLAG_ISET)
    {
      if (isDigit(key) && !FLAG_SHIFT)
      {
        FLAG_DIGIT = 1;
        string.concat(key);
        ISET_CH3 = string.toFloat() / 1000;
        if (ISET_CH3 > I_MAX)
          ISET_CH3 = 0;
        if (string.length() > 4)
        {
          string = "";
        }
      }
      if (ENCODER_ADJ)
      {
        if (key == '#')
        {
          if (FLAG_DIGIT_A > 0)
            FLAG_DIGIT_A--;
        }
        if (key == '*')
        {
          if (FLAG_DIGIT_A < 3)
            FLAG_DIGIT_A++;
        }

        if (pos != 0)
        {
          if (FLAG_DIGIT_A == 0)
            ISET_CH3 = readEncoder(ISET_CH3, 0.001, 0, I_MAX);
          else if (FLAG_DIGIT_A == 1)
            ISET_CH3 = readEncoder(ISET_CH3, 0.01, 0, I_MAX);
          else if (FLAG_DIGIT_A == 2)
            ISET_CH3 = readEncoder(ISET_CH3, 0.1, 0, I_MAX);
          else if (FLAG_DIGIT_A == 3)
            ISET_CH3 = readEncoder(ISET_CH3, 1, 0, I_MAX - 0.2);
          SET_OUT();
        }
      } // ENCODER_ADJ

      break; // FLAG_SELECT CASE 3
    }

  } // switch FLAG_SELECT

} // void KEYBOAR_PRINCIPAL

void KEYBOARD_MENU()
{

  // sai do menu
  if (key == 'D' && FLAG_SHIFT)
  {
    ID_DISPLAY = 0;
    ENCODER_ADJ = 0;
    FLAG_SHIFT = 0;
  //  FLAG_SELECT = 0;
    STORE_CONFIG();
    tft.fillScreen(TFT_BLACK);
  }


  if (FLAG_SELECT == 0)
  {

    if (key == '*')
    {
      if (MENU_ID > 1)
        MENU_ID--;
    }
    if (key == '#')
    {
      if (MENU_ID < ITENS_MENU_CONFIG)
        MENU_ID++;
    }


    if (pos != 0)
    {
      if (MENU_ID == 1)
        FLAG_BUZZER = readEncoder(FLAG_BUZZER, 1, 0, 1);
      else if (MENU_ID == 2)
        OTP = readEncoder(OTP, 5, 50, 90);
      else if (MENU_ID == 3)
        TEMP_MIN = readEncoder(TEMP_MIN, 5, 30, 60);
      else if (MENU_ID == 4)
        FAN_PWM_MIN = readEncoder(FAN_PWM_MIN, 5, 50, 250);
    }
  }
  else if (FLAG_SELECT == 1)
  {

    if (key == '*')
    {
      if (MENU_ID > 1)
        MENU_ID--;
    }
    if (key == '#')
    {
      if (MENU_ID < ITENS_MENU_CH1_2)
        MENU_ID++;
    }

    if (pos != 0)
    {
      if (MENU_ID == 1)
      {
        OVP_CH1 = readEncoder(OVP_CH1, 0.1, 0, V_MAX + 1);    
      }
      else if (MENU_ID == 2)
      {
        OCP_CH1 = readEncoder(OCP_CH1, 0.1, 0, I_MAX + 0.1);
      }
      else if (MENU_ID == 3)
      {
        VMAX_CH1 = readEncoder(VMAX_CH1, 1, 0, V_MAX);
      }
      else if (MENU_ID == 4)
      {
        TRIGGER_CH1 = readEncoder(TRIGGER_CH1, 1, 0, 1);
      }
      else if (MENU_ID == 5)
      {
        FLAG_BOUT_CH1 = readEncoder(FLAG_BOUT_CH1, 1, 0, 1);   
      }
      else if (MENU_ID == 6)
      {
      DELAY_CH1 = readEncoder(DELAY_CH1, 10, 0, 10000);               
      }
    }
  }
  else if (FLAG_SELECT == 2)
  {

    if (key == '*')
    {
      if (MENU_ID > 1)
        MENU_ID--;
    }
    if (key == '#')
    {
      if (MENU_ID < ITENS_MENU_CH1_2)
        MENU_ID++;
    }

    if (pos != 0)
    {
      if (MENU_ID == 1)
        OVP_CH2 = readEncoder(OVP_CH2, 0.1, 0, V_MAX + 1);
      else if (MENU_ID == 2)
        OCP_CH2 = readEncoder(OCP_CH2, 0.1, 0, I_MAX + 0.1);
      else if (MENU_ID == 3)
        VMAX_CH2 = readEncoder(VMAX_CH2, 1, 0, V_MAX);
      else if (MENU_ID == 4)
        TRIGGER_CH2 = readEncoder(TRIGGER_CH2, 1, 0, 1);
      else if (MENU_ID == 5)
        FLAG_BOUT_CH2 = readEncoder(FLAG_BOUT_CH2, 1, 0, 1);   
      else if (MENU_ID == 6)
        DELAY_CH2 = readEncoder(DELAY_CH2, 10, 0, 10000);                    
    }
  }
  else if (FLAG_SELECT == 3)
  {

    if (key == '*')
    {
      if (MENU_ID > 1)
        MENU_ID--;
    }
    if (key == '#')
    {
      if (MENU_ID < ITENS_MENU_CH3)
        MENU_ID++;
    }

    if (pos != 0)
    {
      if (MENU_ID == 1)
        OVP_CH3 = readEncoder(OVP_CH3, 0.1, 0, V_MAX_CH3 + 0.3);
      else if (MENU_ID == 2)
        OCP_CH3 = readEncoder(OCP_CH3, 0.1, 0, I_MAX + 0.1);
      else if (MENU_ID == 3)
        VMAX_CH3 = readEncoder(VMAX_CH3, 0.1, 0, V_MAX_CH3);
      else if (MENU_ID == 4)
        TRIGGER_CH3 = readEncoder(TRIGGER_CH3, 1, 0, 1);
      else if (MENU_ID == 5)
        FLAG_BOUT_CH3 = readEncoder(FLAG_BOUT_CH3, 1, 0, 1);  
      else if (MENU_ID == 6)
        DELAY_CH3 = readEncoder(DELAY_CH3, 10, 0, 10000);                     
    }
  }

} // void KEYBOARD_MENU

void KEYBOARD_MEMORIAS()
{

  switch (MEMORY_MODE)
  {
  case 0: // CALL_MEMORY

    if (key == 'B')
    {
      tft.fillScreen(TFT_BLACK);
      ID_DISPLAY = 0;
    }

    if (FLAG_SELECT == 1)
    {
      if (key == '1')
      {
        EEPROM.get(400, VSET_CH1);
        EEPROM.get(405, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '2')
      {
        EEPROM.get(410, VSET_CH1);
        EEPROM.get(415, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '3')
      {
        EEPROM.get(420, VSET_CH1);
        EEPROM.get(425, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '4')
      {
        EEPROM.get(430, VSET_CH1);
        EEPROM.get(435, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '5')
      {
        EEPROM.get(440, VSET_CH1);
        EEPROM.get(445, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '6')
      {
        EEPROM.get(450, VSET_CH1);
        EEPROM.get(455, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
      }
      else if (key == '7')
      {
        EEPROM.get(460, VSET_CH1);
        EEPROM.get(465, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '8')
      {
        EEPROM.get(470, VSET_CH1);
        EEPROM.get(475, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '9')
      {
        EEPROM.get(480, VSET_CH1);
        EEPROM.get(485, ISET_CH1);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
    }
    else if (FLAG_SELECT == 2)
    {
      if (key == '1')
      {
        EEPROM.get(500, VSET_CH2);
        EEPROM.get(505, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '2')
      {
        EEPROM.get(510, VSET_CH2);
        EEPROM.get(515, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '3')
      {
        EEPROM.get(520, VSET_CH2);
        EEPROM.get(525, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '4')
      {
        EEPROM.get(530, VSET_CH2);
        EEPROM.get(535, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '5')
      {
        EEPROM.get(540, VSET_CH2);
        EEPROM.get(545, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '6')
      {
        EEPROM.get(550, VSET_CH2);
        EEPROM.get(555, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '7')
      {
        EEPROM.get(560, VSET_CH2);
        EEPROM.get(565, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '8')
      {
        EEPROM.get(570, VSET_CH2);
        EEPROM.get(575, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '9')
      {
        EEPROM.get(580, VSET_CH2);
        EEPROM.get(585, ISET_CH2);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
    }
    else if (FLAG_SELECT == 3)
    {
      if (key == '1')
      {
        EEPROM.get(600, VSET_CH3);
        EEPROM.get(605, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '2')
      {
        EEPROM.get(610, VSET_CH3);
        EEPROM.get(615, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '3')
      {
        EEPROM.get(620, VSET_CH3);
        EEPROM.get(625, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '4')
      {
        EEPROM.get(630, VSET_CH3);
        EEPROM.get(635, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '5')
      {
        EEPROM.get(640, VSET_CH3);
        EEPROM.get(645, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
        ;
      }
      else if (key == '6')
      {
        EEPROM.get(650, VSET_CH3);
        EEPROM.get(655, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
    }
    else
    {
      if (key == '1')
      {
        EEPROM.get(700, VSET_CH1);
        EEPROM.get(705, ISET_CH1);
        EEPROM.get(710, VSET_CH2);
        EEPROM.get(715, ISET_CH2);
        EEPROM.get(720, VSET_CH3);
        EEPROM.get(725, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '2')
      {
        EEPROM.get(730, VSET_CH1);
        EEPROM.get(735, ISET_CH1);
        EEPROM.get(740, VSET_CH2);
        EEPROM.get(745, ISET_CH2);
        EEPROM.get(750, VSET_CH3);
        EEPROM.get(755, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '3')
      {
        EEPROM.get(760, VSET_CH1);
        EEPROM.get(765, ISET_CH1);
        EEPROM.get(770, VSET_CH2);
        EEPROM.get(775, ISET_CH2);
        EEPROM.get(780, VSET_CH3);
        EEPROM.get(785, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '4')
      {
        EEPROM.get(790, VSET_CH1);
        EEPROM.get(795, ISET_CH1);
        EEPROM.get(800, VSET_CH2);
        EEPROM.get(805, ISET_CH2);
        EEPROM.get(810, VSET_CH3);
        EEPROM.get(815, ISET_CH3);
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
    }
    break;

  case 1: // STORE_MEMORY

    if (key == 'B')
    {
      tft.fillScreen(TFT_BLACK);
      MEMORY_MODE = 0;
      ID_DISPLAY = 0;
    }
    if (FLAG_SELECT == 1)
    {
      if (key == '1')
      {
        BUZZER(500);
        EEPROM.put(400, VSET_CH1);
        EEPROM.put(405, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        SET_OUT();
      }
      else if (key == '2')
      {
        BUZZER(500);
        EEPROM.put(410, VSET_CH1);
        EEPROM.put(415, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '3')
      {
        BUZZER(500);
        EEPROM.put(420, VSET_CH1);
        EEPROM.put(425, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '4')
      {
        BUZZER(500);
        EEPROM.put(430, VSET_CH1);
        EEPROM.put(435, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '5')
      {
        BUZZER(500);
        EEPROM.put(440, VSET_CH1);
        EEPROM.put(445, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '6')
      {
        BUZZER(500);
        EEPROM.put(450, VSET_CH1);
        EEPROM.put(455, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '7')
      {
        BUZZER(500);
        EEPROM.put(460, VSET_CH1);
        EEPROM.put(465, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '8')
      {
        BUZZER(500);
        EEPROM.put(470, VSET_CH1);
        EEPROM.put(475, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '9')
      {
        BUZZER(500);
        EEPROM.put(480, VSET_CH1);
        EEPROM.put(485, ISET_CH1);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
    }
    else if (FLAG_SELECT == 2)
    {
      if (key == '1')
      {
        BUZZER(500);
        EEPROM.put(500, VSET_CH2);
        EEPROM.put(505, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '2')
      {
        BUZZER(500);
        EEPROM.put(510, VSET_CH2);
        EEPROM.put(515, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '3')
      {
        BUZZER(500);
        EEPROM.put(520, VSET_CH2);
        EEPROM.put(525, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '4')
      {
        BUZZER(500);
        EEPROM.put(530, VSET_CH2);
        EEPROM.put(535, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '5')
      {
        BUZZER(500);
        EEPROM.put(540, VSET_CH2);
        EEPROM.put(545, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '6')
      {
        BUZZER(500);
        EEPROM.put(550, VSET_CH2);
        EEPROM.put(555, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '7')
      {
        BUZZER(500);
        EEPROM.put(560, VSET_CH2);
        EEPROM.put(565, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '8')
      {
        BUZZER(500);
        EEPROM.put(570, VSET_CH2);
        EEPROM.put(575, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '9')
      {
        BUZZER(500);
        EEPROM.put(580, VSET_CH2);
        EEPROM.put(585, ISET_CH2);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
    }
    else if (FLAG_SELECT == 3)
    {
      if (key == '1')
      {
        BUZZER(500);
        EEPROM.put(600, VSET_CH3);
        EEPROM.put(605, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '2')
      {
        BUZZER(500);
        EEPROM.put(610, VSET_CH3);
        EEPROM.put(615, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '3')
      {
        BUZZER(500);
        EEPROM.put(620, VSET_CH3);
        EEPROM.put(625, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '4')
      {
        BUZZER(500);
        EEPROM.put(630, VSET_CH3);
        EEPROM.put(635, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '5')
      {
        BUZZER(500);
        EEPROM.put(640, VSET_CH3);
        EEPROM.put(645, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '6')
      {
        BUZZER(500);
        EEPROM.put(650, VSET_CH3);
        EEPROM.put(655, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
    }
    else
    {
      if (key == '1')
      {
        BUZZER(500);
        EEPROM.put(700, VSET_CH1);
        EEPROM.put(705, ISET_CH1);
        EEPROM.put(710, VSET_CH2);
        EEPROM.put(715, ISET_CH2);
        EEPROM.put(720, VSET_CH3);
        EEPROM.put(725, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '2')
      {
        BUZZER(500);
        EEPROM.put(730, VSET_CH1);
        EEPROM.put(735, ISET_CH1);
        EEPROM.put(740, VSET_CH2);
        EEPROM.put(745, ISET_CH2);
        EEPROM.put(750, VSET_CH3);
        EEPROM.put(755, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '3')
      {
        BUZZER(500);
        EEPROM.put(760, VSET_CH1);
        EEPROM.put(765, ISET_CH1);
        EEPROM.put(770, VSET_CH2);
        EEPROM.put(775, ISET_CH2);
        EEPROM.put(780, VSET_CH3);
        EEPROM.put(785, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
      else if (key == '4')
      {
        BUZZER(500);
        EEPROM.put(790, VSET_CH1);
        EEPROM.put(795, ISET_CH1);
        EEPROM.put(800, VSET_CH2);
        EEPROM.put(805, ISET_CH2);
        EEPROM.put(810, VSET_CH3);
        EEPROM.put(815, ISET_CH3);
        EEPROM.commit();
        tft.fillScreen(TFT_BLACK);
        ID_DISPLAY = 0;
        MEMORY_MODE = 0;
      }
    }
  } // MEMORY_MODE
} // void KEYBOAR_MEMORIAS

void KEYBOARD_CALIBRACAO()
{

  // liga / desliga as saidas
  if (key == '1' && FLAG_SHIFT)
  {
    FLAG_OUT_CH1 = !FLAG_OUT_CH1;
    FLAG_SHIFT = 0;
    SET_OUT();
  }
  if (key == '2' && FLAG_SHIFT)
  {
    FLAG_OUT_CH2 = !FLAG_OUT_CH2;
    FLAG_SHIFT = 0;
    SET_OUT();
  }
  if (key == '3' && FLAG_SHIFT)
  {
    FLAG_OUT_CH3 = !FLAG_OUT_CH3;
    FLAG_SHIFT = 0;
    SET_OUT();
  }

  // seleciona os canais
  if (key == '4' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 1)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 1;
    FLAG_SHIFT = 0;
  }
  if (key == '5' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 2)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 2;
    FLAG_SHIFT = 0;
  }
  if (key == '6' && FLAG_SHIFT)
  {
    if (FLAG_SELECT == 3)
      FLAG_SELECT = 0;
    else
      FLAG_SELECT = 3;
    FLAG_SHIFT = 0;
  }

  if (key == '0' && FLAG_SHIFT)
  {
    ID_DISPLAY = 0;
    ENCODER_ADJ = 0;
    FLAG_SHIFT = 0;
    tft.fillScreen(TFT_BLACK);
    STORE_CALIBRATION();
  }

  if (!digitalRead(encoderSW) && flag_encoderSW == 0)
  { // botao do encoder
    BUZZER(50);
    X = X * 10;
    if (X > 0.1)
      X = 0.0001;
    flag_encoderSW = 1;
  }
  else if (digitalRead(encoderSW) && flag_encoderSW == 1)
  {
    flag_encoderSW = 0;
  }

  if (FLAG_SELECT == 1)
  {

    if (key == '*')
    {
      if (CH1_ID_CAL > 1)
        CH1_ID_CAL--;
    }
    if (key == '#')
    {
      if (CH1_ID_CAL < 9)
        CH1_ID_CAL++;
    }

    if (pos != 0)
    {
      if (CH1_ID_CAL == 1)
        CH1_METER_V = readEncoder(CH1_METER_V, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 2)
        CH1_METER_I = readEncoder(CH1_METER_I, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 3)
        CH1_OUT_V1 = readEncoder(CH1_OUT_V1, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 4)
        CH1_OUT_V2 = readEncoder(CH1_OUT_V2, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 5)
        CH1_OUT_V3 = readEncoder(CH1_OUT_V3, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 6)
        CH1_OUT_I1 = readEncoder(CH1_OUT_I1, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 7)
        CH1_OUT_I2 = readEncoder(CH1_OUT_I2, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 8)
        CH1_OUT_I3 = readEncoder(CH1_OUT_I3, X, -2.0, 2.0);
      else if (CH1_ID_CAL == 9)
        CH1_OFFSET = readEncoder(CH1_OFFSET, X, -2.0, 2.0);
      SET_OUT();
    }
  } // FLAG_SELECT==1

  else if (FLAG_SELECT == 2)
  {

    if (key == '*')
    {
      if (CH2_ID_CAL > 1)
        CH2_ID_CAL--;
    }
    if (key == '#')
    {
      if (CH2_ID_CAL < 9)
        CH2_ID_CAL++;
    }

    if (pos != 0)
    {
      if (CH2_ID_CAL == 1)
        CH2_METER_V = readEncoder(CH2_METER_V, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 2)
        CH2_METER_I = readEncoder(CH2_METER_I, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 3)
        CH2_OUT_V1 = readEncoder(CH2_OUT_V1, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 4)
        CH2_OUT_V2 = readEncoder(CH2_OUT_V2, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 5)
        CH2_OUT_V3 = readEncoder(CH2_OUT_V3, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 6)
        CH2_OUT_I1 = readEncoder(CH2_OUT_I1, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 7)
        CH2_OUT_I2 = readEncoder(CH2_OUT_I2, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 8)
        CH2_OUT_I3 = readEncoder(CH2_OUT_I3, X, -2.0, 2.0);
      else if (CH2_ID_CAL == 9)
        CH2_OFFSET = readEncoder(CH2_OFFSET, X, -2.0, 2.0);
      SET_OUT();
    }

  } // FLAG_SELECT==2

  else if (FLAG_SELECT == 3)
  {

    if (key == '*')
    {
      if (CH3_ID_CAL > 1)
        CH3_ID_CAL--;
    }
    if (key == '#')
    {
      if (CH3_ID_CAL < 5)
        CH3_ID_CAL++;
    }

    if (pos != 0)
    {
      if (CH3_ID_CAL == 1)
        CH3_METER_V = readEncoder(CH3_METER_V, X, -2.0, 2.0);
      else if (CH3_ID_CAL == 2)
        CH3_METER_I = readEncoder(CH3_METER_I, X, -2.0, 2.0);
      else if (CH3_ID_CAL == 3)
        CH3_OUT_V1 = readEncoder(CH3_OUT_V1, X, -2.0, 2.0);
      else if (CH3_ID_CAL == 4)
        CH3_OUT_I1 = readEncoder(CH3_OUT_I1, X, -2.0, 2.0);
      else if (CH3_ID_CAL == 5)
        CH3_OFFSET = readEncoder(CH3_OFFSET, X, -2.0, 2.0);
      SET_OUT();
    }

  } // FLAG_SELECT==3

} // void KEYBOARD_CALIBRACAO
