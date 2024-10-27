//////////////////////////////////////////////////////////////////////////////
/*
 *  Serializer.h
 *  
 *  Module serializes and deserializes data packets.
 * 
 *  Created on: Jun 29, 2024
 *      Author: Žan Hertiš
 */
 ////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Includes 
//////////////////////////////////////////////////////////////////////////////

#include "Serializer.h"

//////////////////////////////////////////////////////////////////////////////
// Global Variables 
//////////////////////////////////////////////////////////////////////////////

// Lookup table for converting a nibble (4 bits) to its ASCII character representation
static const char gNibbleToAscii[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

//////////////////////////////////////////////////////////////////////////////
// Function prototypes 
//////////////////////////////////////////////////////////////////////////////

static bool Serializer_ByteToAscii(const uint8_t data, uint8_t* const lsbAscii, uint8_t* const msbAscii);
static bool Serializer_AsciiToByte(const uint8_t lsbAscii, const uint8_t msbAscii, uint8_t* const byteOut);

//////////////////////////////////////////////////////////////////////////////
// Function Definitions 
//////////////////////////////////////////////////////////////////////////////

static bool Serializer_ByteToAscii(const uint8_t data, uint8_t* const lsbAscii, uint8_t* const msbAscii)
{
    if( data > 0xFF )
    {
        return false;  // Invalid data
    }

    *lsbAscii = gNibbleToAscii[data & 0xF];
    *msbAscii = gNibbleToAscii[(data >> 4) & 0xF];

    return true;
}

static bool Serializer_AsciiToByte(const uint8_t lsbAscii, const uint8_t msbAscii, uint8_t* const byteOut) 
{
    uint8_t lsb, msb;

    // Convert ASCII to nibble for msb
    if( msbAscii >= '0' && msbAscii <= '9' ) 
    {
        msb = msbAscii - '0';
    } 
    else if( msbAscii >= 'A' && msbAscii <= 'F' ) 
    {
        msb = msbAscii - 'A' + 10;
    } 
    else 
    {
        return false;  // Invalid ASCII character
    }

    // Convert ASCII to nibble for lsb
    if( lsbAscii >= '0' && lsbAscii <= '9' ) 
    {
        lsb = lsbAscii - '0';
    } 
    else if( lsbAscii >= 'A' && lsbAscii <= 'F' ) 
    {
        lsb = lsbAscii - 'A' + 10;
    } 
    else
    {
        return false;  // Invalid ASCII character
    }

    // Combine msb and lsb nibbles into a byte
    *byteOut = (msb << 4) | lsb;
    return true;
}

bool Serializer_SerializeForESP(const uint8_t id, const uint32_t data, uint8_t* const serializedData)
{
    uint8_t lsbAscii    = 0;
    uint8_t msbAscii    = 0;
    uint8_t index       = 3; // start of the data field

    serializedData[0] = 'Z';

    // Serialize ID
    Serializer_ByteToAscii(id, &lsbAscii, &msbAscii);
    serializedData[1] = msbAscii;
    serializedData[2] = lsbAscii;

    // Serialize data field 
    for( uint8_t i = 0; i < ESP_DATA_FIELD_SIZE; i++ )
    { 
        uint8_t dataByte        = (data >> (8 * i)) & 0xFF;
        Serializer_ByteToAscii(dataByte, &lsbAscii, &msbAscii);

        serializedData[index++] = lsbAscii;
        serializedData[index++] = msbAscii;
    }

    return true;

}

bool Seriazlizer_Deserialize(const uint8_t* const serializedData, uint8_t* const id, uint32_t* const data)
{
    uint8_t lsbAscii, msbAscii;
    uint8_t index = 2; // Start of the data field
    uint8_t tempByte;

    // Deserialize ID
    msbAscii = serializedData[0];
    lsbAscii = serializedData[1];

    if (!Serializer_AsciiToByte(lsbAscii, msbAscii, id)) {
        return false;
    }

    // Deserialize data field
    *data = 0;

    for (uint8_t i = 0; i < ESP_DATA_FIELD_SIZE; i++) {
        lsbAscii = serializedData[index++];
        msbAscii = serializedData[index++];
        
        if (!Serializer_AsciiToByte(lsbAscii, msbAscii, &tempByte)) {
            return false;
        }

        *data |= (uint32_t)tempByte << (8 * i);
    }

    return true;
}