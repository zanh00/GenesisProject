#include "Serializer.h"

#define ESP_PACKET_SIZE         11
#define ESP_DATA_FIELD_SIZE     4

// Lookup table for converting a nibble (4 bits) to its ASCII character representation
static const char gNibbleToAscii[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

static void Serializer_ByteToAscii(const uint8_t data, uint8_t* const lsbAscii, uint8_t* const msbAscii)
{
    *lsbAscii = gNibbleToAscii[data & 0xF];
    *msbAscii = gNibbleToAscii[(data >> 4) & 0xF];
}

bool Serializer_DataForESP(const uint8_t id, const uint32_t data, uint8_t* const serializedData)
{
    uint8_t lsbAscii    = 0;
    uint8_t msbAscii    = 0;
    uint8_t index       = 3; // start of the data field
    // check if array is large enough
    if( sizeof(*serializedData) < ESP_PACKET_SIZE )
    {
        return false;
    }

    serializedData[0] = 'Z';

    // Serialize ID
    Serializer_ByteToAscii(id, &lsbAscii, &msbAscii);
    serializedData[1] = lsbAscii;
    serializedData[2] = msbAscii;

    // Serialize data field 
    for(uint8_t i = 0; i < ESP_DATA_FIELD_SIZE; i++)
    { 
        uint8_t dataByte        = data << (8 * i);
        Serializer_ByteToAscii(dataByte, lsbAscii, msbAscii);

        serializedData[index++] = lsbAscii;
        serializedData[index++] = msbAscii;
    }

    return true;

}