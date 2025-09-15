#include <Arduino.h>

int cont = 0;

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
  cont++;
  // Publica uma mensagem a cada segundo
  Serial.println(cont);

  delay(100);
}
