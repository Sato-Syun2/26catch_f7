#ifndef ARM_POSITION_MPC_H
#define ARM_POSITION_MPC_H
#include <stdbool.h>
#include <stdint.h>
#define ARM_MPC_N 40
#define ARM_MPC_PERIOD .02f
#define ARM_MPC_SPEED 400.0f /* 両軸個別の速度上限比較試験 [deg/s] */
typedef struct {
    float u[ARM_MPC_N], x[ARM_MPC_N+1], v[ARM_MPC_N+1];
    float elapsed, reference, target, bias;
    float observer_position, observer_velocity;
    bool initialized;
} ArmPositionMpc;
void ArmPositionMpc_Reset(ArmPositionMpc *s);
bool ArmPositionMpc_TargetAllowed(float target);
float ArmPositionMpc_Update(ArmPositionMpc *s,float x,float v,float target,float alpha,float dt);
bool ArmPositionMpc_TargetAllowedForDevice(uint8_t id, float target);
float ArmPositionMpc_UpdateForDevice(uint8_t id, ArmPositionMpc *s,float x,float v,float target,float alpha,float dt);
#endif
