#ifndef ID4_POSITION_MPC_H
#define ID4_POSITION_MPC_H
#include <stdbool.h>
#include <stdint.h>
uint32_t Id4PositionMpc_ClockMs(void);
#define ID4_MPC_HORIZON 50
#define ID4_MPC_PERIOD_S 0.02f
/* 0: 実機確認済み二次評価、1: 到達時間優先試験。電流/端保護は共通。 */
#define ID4_MPC_TIME_PRIORITY 0
/* M2006/C610: 24V時の出力軸無負荷500rpm、ID4は99mm/出力軸1回転。 */
#define ID4_MPC_SPEED_MAX (500.0f * 99.0f / 60.0f)
typedef struct {
    float u[ID4_MPC_HORIZON];
    float x[ID4_MPC_HORIZON+1], v[ID4_MPC_HORIZON+1];
    float reference, elapsed, target, bias;
    bool initialized;
    unsigned iterations;
    float switch_time, arrival_time, phase_time, first_direction;
    bool time_priority_active;
} Id4PositionMpc;
bool Id4MinimumTimePlan(float x,float v,float target,float alpha,float limit,
                       float *switch_time,float *arrival_time,float *direction);
void Id4PositionMpc_Reset(Id4PositionMpc *s);
float Id4PositionMpc_Update(Id4PositionMpc *s, float x, float v,
                           float target, float alpha, float dt);
#endif
