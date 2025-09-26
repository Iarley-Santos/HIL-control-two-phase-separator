/********************************************************************
 * File: serial_bridge.h
 * Description: Utility functions and global definitions for handling 
 *              serial communication (float <-> byte conversion). 
 * 
 *              Designed for use with Arduino, ESP32, and similar 
 *              embedded platforms. 
 * 
 * Created by: Iarley Santos - September 25, 2025
 ********************************************************************/

#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <Arduino.h>

/**
 * @brief Union to convert between two floats and 8 bytes.
 * 
 * Provides a convenient way to send and receive two floats 
 * over serial communication as a raw byte array.
 */
typedef union {
    float numbers[2];   // Stores two float values
    uint8_t bytes[8];   // Raw byte representation of the two floats
} DATA_VECTOR;

// ===================== FUNCTION PROTOTYPES ====================

/**
 * @brief Sends two float values over serial.
 * 
 * @param value1 First float value
 * @param value2 Second float value
 */
void serial_write_vector(float value1, float value2);

/**
 * @brief Receives two float values from serial.
 * 
 * Waits for synchronization character 'B', reads two floats 
 * (8 bytes), and discards data until a newline is found.
 * 
 * @return DATA_VECTOR containing the two floats received
 */
DATA_VECTOR serial_read_vector();

#endif // SERIAL_BRIDGE_H