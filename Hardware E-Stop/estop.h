#ifndef ESTOP_H
#define ESTOP_H

extern volatile int ledFlag;
void ConfigureESTOP(void);
void ESTOPISR(void);


#endif /* ESTOP_H */
