// Define Functions
#define IsHigh(BIT, PORT) ((PORT & (1 << BIT)) != 0)
#define IsLow(BIT, PORT) ((PORT & (1 << BIT)) == 0)
#define SetBit(BIT, PORT) PORT |= (1 << BIT)
#define ClearBit(BIT, PORT) PORT &= ~(1 << BIT)

// Specific to the comms protocol
#define PackBits(V, P) (((V << 5) & 0xFF00) | (V & 0x07) | 0x8000 | (P << 3))