#include <stdint.h>
#include <stdbool.h>

#define ESP_PACKET_SIZE         11
#define ESP_DATA_FIELD_SIZE     4

bool Serializer_SerializeForESP (const uint8_t id, const uint32_t data, uint8_t* const serializedData);
bool Seriazlizer_Deserialize    (const uint8_t* const serializedData, uint8_t* const id, uint32_t* const data);