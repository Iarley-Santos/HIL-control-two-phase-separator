#include <Arduino.h>
#include <pid_controller.h>

// Union para converter float <-> bytes
typedef union{
  float numbers[2];
  uint8_t bytes[8];
} DATA_VECTOR;


// Funções
DATA_VECTOR serial_read_vector();
void serial_write_vector(float value1, float value2);
float pid_calculation(float error);

// Variaveis
DATA_VECTOR recive_data;
DATA_VECTOR control;

// Inicializa o PID de nível com ganhos específicos
pid_controller pid_level(1.0, 0.0, 0.0);

// Inicializa o PID de pressão com ganhos específicos
pid_controller pid_pressure(1.0, 0.0, 0.0);

// Controle de tempo (100 ms)
const float dt = 0.1;         // segundos
const unsigned long Ts = 100; // ms
unsigned long last_time = 0;
unsigned long now = 0;

void setup()
{
  pinMode(2, OUTPUT);

  // Inicia a comunicação serial a 115200 bps
  Serial.begin(115200);

  // Aguarda a serial estar pronta (importante em alguns ambientes)
  while (!Serial) {}

  // Liga LED
  digitalWrite(2, HIGH);

  // Pega o tempo
  last_time = millis();
}

void loop() 
{
  recive_data = serial_read_vector();

  now = millis();

  // Executa a cada Ts ms
  if (now - last_time >= Ts) 
  {
    last_time = now;
    control.numbers[0] = pid_level.pid_calculation(recive_data.numbers[0], dt);
    control.numbers[1] = pid_pressure.pid_calculation(recive_data.numbers[1], dt);
  }

  serial_write_vector(control.numbers[0], control.numbers[1]);
  delay(100);
}

// Função para enviar um dado do tipo float para a serial
void serial_write_vector(float value1, float value2)
{
  DATA_VECTOR a;
  a.numbers[0] = value1;
  a.numbers[1] = value2;

  // Envia Caractere de sincronização
  Serial.write('A');

  // Envia 8 bytes
  for (int i=0; i<8; i++)
  { 
    Serial.write(a.bytes[i]);
  }

  Serial.print('\n');
}

// Funcao para receber dado da serial
DATA_VECTOR serial_read_vector()
{
    // Crie uma variável para armazenar os dados recebidos
    DATA_VECTOR f;

    while (Serial.read() != 'B')
    {
      ;
    }

    // Ler os 8 bytes do vetor
    int cont = 0;
    while(cont < 8)
    {
        if(Serial.available() > 0)
        {
            f.bytes[cont] = Serial.read();
            cont += 1;
        }
    }
  
    // Aguarde um '\n'
    while(Serial.available() > 0 && Serial.read() != '\n')
    {
      ;// Descarta dados até encontrar o '\n'
    }
    
    return f;
}
