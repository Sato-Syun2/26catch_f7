#include "C620Commission.h"
#include "CAN_RoboMas_System.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static uint32_t tick;
static bool fresh = true, limit_switch;
static RoboMas_FeedbackData feedback = {.device_id=3, .get_flag=1};
static int16_t sent;
GPIO_TypeDef test_gpio;
CAN_HandleTypeDef hcan2;
UART_HandleTypeDef huart3;
uint32_t HAL_GetTick(void) { return tick; }
bool RoboMas_FeedbackFresh(uint8_t id) { return id == 3 && fresh; }
bool RoboMas_Id4FeedbackFresh(void) { return false; }
RoboMas_FeedbackData Get_RoboMas_FeedbackData(RoboMas_DeviceInfo *dev) { (void)dev; return feedback; }
void _change_internal_offset_for_calib(RoboMas_DeviceInfo *dev) { (void)dev; feedback.position=0; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    assert(port == &test_gpio && pin == 16); return limit_switch;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *a, uint8_t *b, uint16_t c, uint32_t d) {
    (void)a;(void)b;(void)c;(void)d;return HAL_OK;
}
HAL_StatusTypeDef RoboMas_SendBytes(CAN_HandleTypeDef *can, uint32_t id, uint8_t *bytes, uint32_t size) {
    (void)can; assert(id == 0x200 && size == 8);
    assert(bytes[0] == 0 && bytes[1] == 0 && bytes[2] == 0 && bytes[3] == 0 && bytes[6] == 0 && bytes[7] == 0);
    sent=(int16_t)((bytes[4]<<8)|bytes[5]); return HAL_OK;
}
static void step(RoboMas_DeviceInfo *dev, uint32_t dt) {
    tick += dt; RoboMas_SendRequest(dev,1,500,&hcan2);
    const int cap = dev->ctrl_param._is_calibrating
        ? (int)(C620_CALIBRATION_CURRENT_A*16384/20) : C620_IsSurvey() ? 4096 : 2048;
    assert(abs(sent) <= cap);
}
static bool service(RoboMas_DeviceInfo *dev, const char *cmd, float value) {
    char msg[33]; return C620_Service(dev,cmd,value,msg,sizeof(msg));
}
int main(int argc, char **argv) {
    RoboMas_DeviceInfo dev = {.device_id=3,.device_type=ROBOMASTER_C620};
    RoboMas_Init(&dev,1);
    dev.ctrl_param.use_internal_offset=ROBOMAS_USE_OFFSET_POS_CALIB;
    dev.ctrl_param.rotation=ROBOMAS_ROT_CW;
    dev.ctrl_param.pid_vel.kp=.2f;
    dev.ctrl_param.current_limit_size=2.5f;
    RoboMas_SetTarget(&dev,100); RoboMas_ControlEnable(&dev);
    step(&dev,2); assert(sent==0 && !dev.ctrl_param._enable_flag);
    if (argc > 1 && strncmp(argv[1],"boot",4)==0) {
        assert(C620_AUTO_CALIBRATE_ON_BOOT);
        if (strcmp(argv[1],"boot_cancel")==0) {
            C620_CancelRequest(); step(&dev,1000);
            assert(sent==0 && !dev.ctrl_param._is_calibrating); return 0;
        }
        fresh=false;step(&dev,1000);assert(sent==0);
        fresh=true;step(&dev,2);
        assert(sent==0);step(&dev,C620_CALIBRATION_RAMP_MS);
        assert(sent>0 && sent<=(int)(C620_CALIBRATION_CURRENT_A*16384/20) && dev.ctrl_param._is_calibrating);
        limit_switch=true;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag && !dev.ctrl_param._is_calibrating);
        limit_switch=false;step(&dev,2000);assert(sent==0);
        puts("C620 automatic boot calibration checks passed");return 0;
    }
    assert(!service(&dev,"c620_current",3));
    assert(!service(&dev,"c620_range",100));
    assert(service(&dev,"c620_calibrate",0));
    if (argc > 1 && strcmp(argv[1],"cancel")==0) {
        C620_CancelRequest(); step(&dev,2); assert(sent==0); return 0;
    }
    step(&dev,2);assert(sent==0 && dev.ctrl_param._is_calibrating);
    step(&dev,C620_CALIBRATION_RAMP_MS/2);assert(sent==819);
    step(&dev,C620_CALIBRATION_RAMP_MS/2);
    assert(sent==1638 && dev.ctrl_param._is_calibrating);
    assert(dev.ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL);
    dev.ctrl_param.pid_vel.kp=2;step(&dev,2);assert(sent==8192);
    feedback.velocity=-10;step(&dev,2);assert(sent==0);
    feedback.velocity=0;dev.ctrl_param.pid_vel.kp=.2f;
    if (argc > 1 && strcmp(argv[1],"overspeed")==0) {
        feedback.velocity=-31;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        RoboMas_ControlEnable(&dev);assert(!dev.ctrl_param._enable_flag);return 0;
    }
    if (argc > 1 && strcmp(argv[1],"no_stall")==0) {
        feedback.velocity=0;step(&dev,5000);
        assert(sent==1638 && dev.ctrl_param._is_calibrating);
        limit_switch=true;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._is_calibrating && !dev.ctrl_param._enable_flag);
        puts("C620 calibration: 10A only during homing, no stall cutoff, switch stops current");return 0;
    }
    if (argc > 1 && (strcmp(argv[1],"stale")==0 || strcmp(argv[1],"timeout")==0)) {
        if (strcmp(argv[1],"stale")==0) {
            fresh=false; step(&dev,2);
        } else {
            feedback.position=-10;feedback.velocity=-10;step(&dev,C620_CALIBRATION_TIMEOUT_MS+1);
        }
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        RoboMas_ControlEnable(&dev);assert(!dev.ctrl_param._enable_flag);return 0;
    }
    limit_switch=true; step(&dev,2);
    assert(sent==0 && !dev.ctrl_param._enable_flag && !dev.ctrl_param._is_calibrating);
    const bool survey5 = argc > 1 && strcmp(argv[1],"survey5")==0;
    assert(!service(&dev,"c620_survey",6));
    assert(service(&dev,"c620_survey",survey5 ? 5 : 0));
    step(&dev,2); assert(sent==0);
    step(&dev,500); assert(sent==(survey5 ? -4096 : -819));
    dev.ctrl_param.pid_vel.kp=2;step(&dev,2);
    assert(sent==(survey5 ? -4096 : -819));
    feedback.velocity=40;step(&dev,2);assert(sent==0);
    dev.ctrl_param.pid_vel.kp=.2f;
    if (argc > 1 && strncmp(argv[1],"survey_",7)==0) {
        if (strcmp(argv[1],"survey_speed")==0) feedback.velocity=151;
        if (strcmp(argv[1],"survey_stale")==0) fresh=false;
        step(&dev,strcmp(argv[1],"survey_timeout")==0 ? 60000 : 2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        RoboMas_ControlEnable(&dev);assert(!dev.ctrl_param._enable_flag);return 0;
    }
    feedback.position=100;feedback.velocity=20;step(&dev,100);
    feedback.velocity=0;feedback.current=0;
    step(&dev,600);assert(dev.ctrl_param._enable_flag);
    feedback.current=survey5 ? -5 : -1;
    step(&dev,600);assert(sent==0 && !dev.ctrl_param._enable_flag);
    assert(service(&dev,"c620_survey",survey5 ? 5 : 0));
    assert(!service(&dev,"c620_range",95));
    step(&dev,2);assert(sent==0 && dev.ctrl_param._enable_flag);
    step(&dev,600);assert(dev.ctrl_param._enable_flag);
    step(&dev,600);assert(sent==0 && !dev.ctrl_param._enable_flag);
    assert(!service(&dev,"c620_range",96));
    assert(service(&dev,"c620_range",95));
    RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_CURRENT);
    RoboMas_SetTarget(&dev,-100);RoboMas_ControlEnable(&dev);step(&dev,2);assert(sent==2048);
    RoboMas_ControlDisable(&dev);step(&dev,2);assert(sent==0);
    RoboMas_send_current(&dev,20,&hcan2);assert(sent==0);
    feedback.position=50;RoboMas_SetTarget(&dev,NAN);RoboMas_ControlEnable(&dev);step(&dev,2);assert(sent==0);
    puts("C620 commissioning/current-frame checks passed");
}
