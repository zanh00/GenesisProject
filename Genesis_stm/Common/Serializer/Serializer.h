#include <stdint.h>
#include <stdbool.h>

#define SERIALIZER_PACKET_SIZE         11
#define SERIALIZER_DATA_FIELD_SIZE     4

bool Serializer_SerializeUint32 (const uint8_t id, const uint32_t data, uint8_t* const serializedData);
bool Serializer_SerializeFloat  (const uint8_t id, const float data, uint8_t* const serializedData);
bool Seriazlizer_Deserialize    (const uint8_t* const serializedData, uint8_t* const id, uint32_t* const data);