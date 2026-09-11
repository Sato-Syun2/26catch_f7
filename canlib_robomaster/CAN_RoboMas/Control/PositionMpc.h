#ifndef ID4_POSITION_MPC_H
#define ID4_POSITION_MPC_H
#include <stdbool.h>
#include <stdint.h>
uint32_t Id4PositionMpc_ClockMs(void);
#define ID4_MPC_HORIZON 50
#define ID4_MPC_PERIOD_S 0.02f
/* M2006/C610: 24V時の出力軸無負荷500rpm、ID4は99mm/出力軸1回転。 */
#define ID4_MPC_SPEED_MAX (500.0f * 99.0f / 60.0f)
typedef struct {
    float u[ID4_MPC_HORIZON];
    float x[ID4_MPC_HORIZON+1], v[ID4_MPC_HORIZON+1];
    float reference, elapsed, target, bias;
    bool initialized;
    unsigned iterations;
} Id4PositionMpc;
void Id4PositionMpc_Reset(Id4PositionMpc *s);
float PositionMpc_UpdateBounded(Id4PositionMpc *s, float x, float v,
    float target, float alpha, float dt, float minimum, float maximum, float speed);
float Id4PositionMpc_Update(Id4PositionMpc *s, float x, float v,
                           float target, float alpha, float dt);
#endif
