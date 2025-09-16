#include <Arduino.h>


typedef union
{
  float number;
  uint8_t bytes[4];
} DATA;

void serial_write(float value);
float get_float();

DATA data;
DATA dado;

void setup()
{
  pinMode(2, OUTPUT);
  // Inicia a comunicação serial a 115200 bps
  Serial.begin(115200);

  // Aguarda a serial estar pronta (importante em alguns ambientes)
  while (!Serial) {}
  digitalWrite(2, 1);
}

void loop() 
{
  dado.number = get_float();
  if (dado.number == 1)
  {
    digitalWrite(2, 0);
  }

  delay(100);

  serial_write(dado.number);
}

// função para enviar um dado do tipo float para a serial
void serial_write(float value)
{
  DATA a;
  a.number = value;
  Serial.write('A'); // para liberar a comunicação

  for (int i=0; i<4; i++)
  {
    Serial.write(a.bytes[i]);
  }

  Serial.print('\n');
}

// funcao para receber dado da serial
float get_float()
{
  int cont = 0;
  DATA f;
  while(cont<4)
  {
    f.bytes[cont] = Serial.read();
    cont += 1;
  }
  return f.number;
}