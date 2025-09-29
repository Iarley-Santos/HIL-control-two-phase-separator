#include "serial_bridge.h"

void serial_write_vector(float value1, float value2) 
{
    DATA_VECTOR_OUT a;
    a.numbers[0] = value1;
    a.numbers[1] = value2;

    Serial.write('A');
    for (int i = 0; i < 8; i++) 
    {
        Serial.write(a.bytes[i]);
    }
    Serial.print('\n');
}

DATA_VECTOR_IN serial_read_vector() 
{
    DATA_VECTOR_IN f;

    while (Serial.read() != 'B') {;}

    int cont = 0;
    while (cont < 24) 
    {
        if (Serial.available() > 0) 
        {
            f.bytes[cont] = Serial.read();
            cont++;
        }
    }

    while (Serial.available() > 0 && Serial.read() != '\n') {;}

    return f;
}
