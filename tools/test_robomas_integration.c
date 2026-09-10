/* CAN2のC610 ID4/ID1とC620 ID3を同一フレームで動かすホスト回帰試験。 */
#include "C620Commission.h"
#include "CAN_RoboMas_System.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_test_config.h"
#include <string.h>

GPIO_TypeDef test_gpio;
CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart3;
static uint32_t tick;
static bool pressed, fresh[9];
static RoboMas_FeedbackData feedback[9];
static int16_t frame[4];
uint32_t HAL_GetTick(void) { return tick; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) { (void)p;(void)pin;return pressed; }
bool RoboMas_FeedbackFresh(uint8_t id) { return id<9 && fresh[id]; }
bool RoboMas_Id4FeedbackFresh(void) { return fresh[4]; }
RoboMas_FeedbackData Get_RoboMas_FeedbackData(RoboMas_DeviceInfo *d) { return feedback[d->device_id]; }
void _change_internal_offset_for_calib(RoboMas_DeviceInfo *d) { feedback[d->device_id].position=0; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *a,uint8_t *b,uint16_t c,uint32_t d)
{ (void)a;(void)b;(void)c;(void)d;return HAL_OK; }
HAL_StatusTypeDef RoboMas_SendBytes(CAN_HandleTypeDef *can,uint32_t id,uint8_t *bytes,uint32_t size)
{
    assert(can==&hcan2 && id==0x200 && size==8);
    for(unsigned i=0;i<4;i++) frame[i]=(int16_t)((bytes[2*i]<<8)|bytes[2*i+1]);
    return HAL_OK;
}
static void step(RoboMas_DeviceInfo *devices)
{ tick+=2;RoboMas_SendRequest(devices,3,500,&hcan2);assert(frame[1]==0); }
static bool service(RoboMas_DeviceInfo *d,const char *cmd,float x)
{ char text[33];return C620_Service(d,cmd,x,text,sizeof(text)); }
int main(void)
{
    RoboMas_DeviceInfo devices[3]={
        {.device_id=4,.device_type=ROBOMASTER_C610},
        {.device_id=1,.device_type=ROBOMASTER_C610},
        {.device_id=3,.device_type=ROBOMASTER_C620}};
    RoboMas_Init(devices,3);
    for(unsigned i=0;i<3;i++) {
        RoboMas_DeviceInfo *d=&devices[i];
        feedback[d->device_id]=(RoboMas_FeedbackData){.device_id=d->device_id,.get_flag=1};
        fresh[d->device_id]=true;
        d->ctrl_param.use_internal_offset=ROBOMAS_USE_OFFSET_POS_CALIB;
        d->ctrl_param.velocity_dob.J=.001f;
        d->ctrl_param.velocity_dob.d=.01f;
        d->ctrl_param.velocity_dob.K_tau=1;
        d->ctrl_param.velocity_dob.reference_alpha=30;
        d->ctrl_param.velocity_dob.velocity_kp=.1f;
        d->ctrl_param.velocity_dob.velocity_unit_to_rad_s=1;
        d->ctrl_param.velocity_dob.current_limit=10;
        d->ctrl_param.velocity_dob.current_limit_enable=true;
    }
    step(devices);assert(frame[0]==0 && frame[2]==0 && frame[3]==0);
    assert(!service(&devices[0],"c620_status",0));
    assert(!service(&devices[1],"c620_calibrate",5));
    RoboMas_ChangeControl(&devices[0],ROBOMAS_CTRL_POS_MPC);
    RoboMas_ChangeControl(&devices[2],ROBOMAS_CTRL_POS_MPC);
    assert(service(&devices[2],"c620_calibrate",5));
    RoboMas_Calibration(&devices[0],-40,ROBOMAS_SWITCH_NO,&test_gpio,16,&hcan2);
    step(devices);pressed=true;step(devices);pressed=false;
    assert(devices[0].ctrl_param.ctrl_type==ROBOMAS_CTRL_POS_MPC);
    assert(devices[2].ctrl_param.ctrl_type==ROBOMAS_CTRL_POS_MPC);
    if (INTEGRATED_STARTUP_HOLD) {
        assert(devices[0].ctrl_param._enable_flag && devices[2].ctrl_param._enable_flag);
        assert(devices[0].ctrl_param._startup_hold && devices[2].ctrl_param._startup_hold);
        assert(devices[0].ctrl_param._target_value==0 && devices[2].ctrl_param._target_value==0);
        pressed=true;step(devices);pressed=false;
        assert(devices[2].ctrl_param._enable_flag && C620_EnableAllowed());
        RoboMas_ControlDisable(&devices[0]);
        RoboMas_ControlDisable(&devices[2]);
        assert(!devices[0].ctrl_param._startup_hold && !devices[2].ctrl_param._startup_hold);
    } else {
        assert(!devices[0].ctrl_param._enable_flag && !devices[2].ctrl_param._enable_flag);
        assert(service(&devices[2],"c620_restore",175));
    }
    assert(service(&devices[2],"c620_current",20));
    for(unsigned i=0;i<3;i++) {
        feedback[devices[i].device_id].position=50;
        RoboMas_ChangeControl(&devices[i],ROBOMAS_CTRL_CURRENT);
        RoboMas_SetTarget(&devices[i],1);
        RoboMas_ControlEnable(&devices[i]);
    }
    step(devices);
    assert(abs(frame[0])==1000 && abs(frame[3])==1000 && abs(frame[2])==819);
    RoboMas_ControlDisable(&devices[2]);step(devices);
    assert(abs(frame[0])==1000 && abs(frame[3])==1000 && frame[2]==0);
    RoboMas_ChangeControl(&devices[0],ROBOMAS_CTRL_POS_MPC);
    RoboMas_ChangeControl(&devices[2],ROBOMAS_CTRL_POS_MPC);
    RoboMas_SetTarget(&devices[0],100);RoboMas_SetTarget(&devices[2],100);
    RoboMas_ControlEnable(&devices[0]);RoboMas_ControlEnable(&devices[2]);
    for(unsigned i=0;i<25;i++) step(devices);
    assert(frame[3]!=0 && frame[2]!=0 && abs(frame[0])==1000);
    /* 範囲外目標は拒否し、実位置が上下限外でも内向きMPCはラッチしない。 */
    RoboMas_SetTarget(&devices[0],-1);RoboMas_SetTarget(&devices[2],1000);
    assert(devices[0].ctrl_param._target_value==100 && devices[2].ctrl_param._target_value==100);
    feedback[4].position=-3;feedback[3].position=-3;
    for(unsigned i=0;i<50;i++) step(devices);
    assert(devices[0].ctrl_param._enable_flag && devices[2].ctrl_param._enable_flag);
    assert(frame[3]>0 && frame[2]>0);
    feedback[4].position=530;feedback[3].position=182;
    for(unsigned i=0;i<50;i++) step(devices);
    assert(devices[0].ctrl_param._enable_flag && devices[2].ctrl_param._enable_flag);
    assert(frame[3]<0 && frame[2]<0);
    fresh[3]=false;step(devices);
    assert(frame[2]==0 && frame[3]!=0 && abs(frame[0])==1000);
    for(unsigned i=0;i<3;i++) RoboMas_ControlDisable(&devices[i]);
    step(devices);assert(frame[0]==0 && frame[2]==0 && frame[3]==0);
    puts("PASS: mixed CAN2 frame, independent Disable/stale, simultaneous ID4/C620 MPC");
}
