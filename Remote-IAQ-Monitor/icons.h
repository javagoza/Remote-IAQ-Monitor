// icons
const unsigned short termo6x16[96] PROGMEM={
0x0000, 0x31A6, 0xFFFF, 0xFFFF, 0x31A6, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000,   // 0x0010 (16) pixels
0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF,   // 0x0020 (32) pixels
0x31A6, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0x0000, 0xFFFF, 0x0000,   // 0x0030 (48) pixels
0x0000, 0xFFFF, 0xFFFF, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0x0000,   // 0x0040 (64) pixels
0xFFFF, 0x0000, 0x31A6, 0xD69A, 0x31A6, 0x0000, 0xD69A, 0x31A6, 0xFFFF, 0x31A6, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x31A6,   // 0x0050 (80) pixels
0x31A6, 0x0000, 0x0000, 0xFFFF, 0x31A6, 0xFFFF, 0x31A6, 0x0000, 0xFFFF, 0x31A6, 0x0000, 0x31A6, 0xFFFF, 0xFFFF, 0x31A6, 0x0000,   // 0x0060 (96) pixels
};

const unsigned short humidity6x16[96] PROGMEM={
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xE71C, 0xE71C,   // 0x0010 (16) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0xC638, 0xE71C, 0xE71C, 0xC638, 0x0000, 0x0000, 0xFFFF,   // 0x0020 (32) pixels
0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0x0000, 0xE71C, 0xC638, 0x0000, 0x0000, 0xC638, 0xE71C,   // 0x0030 (48) pixels
0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000,   // 0x0040 (64) pixels
0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xE71C, 0xC638, 0x0000, 0x0000, 0xC638, 0xE71C, 0x0000, 0xE71C,   // 0x0050 (80) pixels
0xFFFF, 0xFFFF, 0xE71C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0060 (96) pixels
};

typedef struct {
  const uint16_t  *data;
  uint16_t width;
  uint16_t height;
  uint8_t dataSize;
} tImage;
