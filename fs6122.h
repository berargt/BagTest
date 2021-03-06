#ifndef FS6122_H
#define FS6122_H
#include <Encoder.h>    // for byte

//////////////////////////////////////////////////////
// Siargo Ltd fs6122 sensor
// see httpfps://www.servoflo.com/download-archive/data-sheets/254-mass-flow-vacuum-sensors/1220-fs6122-datasheet
// and https://www.servoflo.com/download-archive/application-notes/212-mass-flow/1256-fs-i2c-communication
// and https://www.digikey.com/product-detail/en/siargo-ltd/FS6122-250F250-0P0-0/2257-FS6122-250F250-0P0-0-ND/11657804?utm_adgroup=Siargo%20Ltd&utm_source=google&utm_medium=cpc&utm_campaign=Shopping_DK%2BSupplier_Other&utm_term=&utm_content=Siargo%20Ltd&gclid=EAIaIQobChMIkJmP45_c6QIVCtvACh1LvwBoEAYYASABEgL9X_D_BwE
#define FS6122_ADDRESS 0x01 //Default
#define FS6122_FILTER_DEPTH 32 //0-128
#define FS6122_CMD_WRITE_FILTER_DEPTH byte(0x0B)
#define FS6122_CMD_CAL_FLOW_RATE byte(0x1C)
#define FS6122_CMD_CAL_PRESSURE byte(0x24)
#define FS6122_CMD_READ_FLOW_RATE byte(0x83)
#define FS6122_CMD_READ_FLOW_RATE_AND_PRESSURE byte(0x84)
#define FS6122_CMD_READ_PRESSURE byte(0xA3)
#define FS6122_CMD_READ_TEMPERATURE byte(0xB2)
#define FS6122_CMD_READ_HUMIDITY byte(0xB3)

typedef struct{
   long umH2O;
   long mSLPM;
} pressure_flow_type;

void fs6122_init();
void fs6122_zeroFlowCal();
void fs6122_readSmlpM_umH2O(pressure_flow_type *pf);
float fs6122_readSmlpM();

#endif // FS6122_H
