//motors connection
//A0 ---> M1
//A1 ---> M2
//A2 ---> M3
//A3 ---> M4

#pragma once
void setupMotors(GPIO_TypeDef *port, uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4, uint16_t minThrottle, uint16_t maxThrottle);
void sendDSHOT(uint16_t values[]);
void setMotors(float m[]);
void stopMotors(void);
