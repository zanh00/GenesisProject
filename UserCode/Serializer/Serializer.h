#include <stdint.h>
#include <stdbool.h>

#define ESP_PACKET_SIZE         11
#define ESP_DATA_FIELD_SIZE     4

bool Serializer_DataForESP(const uint8_t id, const uint32_t data, uint8_t* const serializedData);