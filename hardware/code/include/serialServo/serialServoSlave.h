#ifndef serialServoSlave
#define serialServoSlave

#include <stdint.h>


void serialServoSlave_setup(uint16_t *cmdpos_, uint16_t *cachepos_, uint8_t *listen_, uint8_t numServos_, const uint8_t *servoConfigAddr);

unsigned int serialServoSlave_dobyte(uint8_t data);

unsigned int serialServoSlave_servoCmd(unsigned int command, unsigned int argument);





#endif