#include "C620Commission.h"
#include "arm_test_config.h"
#include "CAN_RoboMas_System.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static uint32_t tick;
static unsigned origin_resets;
static bool fresh = true, limit_switch;
static RoboMas_FeedbackData feedback = {.device_id=3, .get_flag=1};
static int16_t sent;
GPIO_TypeDef test_gpio;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart3;
uint32_t HAL_GetTick(void) { return tick; }
bool RoboMas_FeedbackFresh(uint8_t id) { return id == 3 && fresh; }
bool RoboMas_Id4FeedbackFresh(void) { return false; }
RoboMas_FeedbackData Get_RoboMas_FeedbackData(RoboMas_DeviceInfo *dev) { (void)dev; return feedback; }
void _change_internal_offset_for_calib(RoboMas_DeviceInfo *dev) { (void)dev; feedback.position=0; ++origin_resets; }
int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    assert(port == &test_gpio && pin == 32); return limit_switch;
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
        ? (int)(C620_CALIBRATION_CURRENT_A*16384/20) : C620_IsSurvey() ? (int)(C620_SURVEY_CURRENT_A*16384/20) : (int)(C620_RUN_CURRENT_A*16384/20);
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
    if (argc>1 && strncmp(argv[1],"timeout",7)==0)
        RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_POS_MPC);
    RoboMas_SetTarget(&dev,100); RoboMas_ControlEnable(&dev);
    step(&dev,2); assert(sent==0 && !dev.ctrl_param._enable_flag);
    if (argc > 1 && strncmp(argv[1],"boot",4)==0) {
        assert(C620_AUTO_CALIBRATE_ON_BOOT);
        RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_POS_MPC);
        if (strcmp(argv[1],"boot_pressed")==0) {
            limit_switch=true; feedback.position=12;
            fresh=false;step(&dev,2);
            assert(sent==0 && origin_resets==0 && !dev.ctrl_param._is_calibrating);
            fresh=true;feedback.get_flag=0;step(&dev,2);
            assert(sent==0 && origin_resets==0);
            feedback.get_flag=1;step(&dev,2);
            assert(tick<1000 && sent==0 && origin_resets==1);
            assert(feedback.position==0 && !dev.ctrl_param._is_calibrating);
            assert(dev.ctrl_param.ctrl_type==ROBOMAS_CTRL_POS_MPC);
            assert(dev.ctrl_param._enable_flag==(bool)INTEGRATED_STARTUP_HOLD);
            assert(dev.ctrl_param._startup_hold==(bool)INTEGRATED_STARTUP_HOLD);
            step(&dev,2);assert(origin_resets==1);
            puts("PASS: pressed-at-boot origin without calibration current, fresh-FB gate, no repeat");
            return 0;
        }
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
        assert(dev.ctrl_param.ctrl_type==ROBOMAS_CTRL_POS_MPC);
        limit_switch=false;step(&dev,2000);assert(sent==0);
        puts("C620 automatic boot calibration checks passed");return 0;
    }
    assert(!service(&dev,"c620_current",C620_RUN_CURRENT_A+1));
    assert(!service(&dev,"c620_alpha",61));
    assert(service(&dev,"c620_alpha",30));
    assert(dev.ctrl_param.velocity_dob.reference_alpha==30);
    assert(!service(&dev,"c620_speed",401));
    assert(!service(&dev,"c620_speed",0));
    assert(service(&dev,"c620_speed",400));
    assert(dev.ctrl_param.velocity_limit_size==400);
    assert(service(&dev,"c620_speed",20));
    assert(!service(&dev,"c620_restore",175));
    assert(!service(&dev,"c620_range",100));
    const float calibration_speed = argc>1 && strcmp(argv[1],"slow_calibration")==0 ? 5 : 10;
    assert(!service(&dev,"c620_calibrate",11));
    assert(service(&dev,"c620_calibrate",calibration_speed));
    if (argc > 1 && strcmp(argv[1],"cancel")==0) {
        C620_CancelRequest(); step(&dev,2); assert(sent==0); return 0;
    }
    step(&dev,2);assert(sent==0 && dev.ctrl_param._is_calibrating);
    step(&dev,C620_CALIBRATION_RAMP_MS/2);assert(sent==(int)(.1f*calibration_speed*16384/20));
    step(&dev,C620_CALIBRATION_RAMP_MS/2);
    assert(sent==(int)(.2f*calibration_speed*16384/20) && dev.ctrl_param._is_calibrating);
    assert(dev.ctrl_param.ctrl_type == ROBOMAS_CTRL_VEL);
    dev.ctrl_param.pid_vel.kp=2;step(&dev,2);
    assert(sent==(int)(C620_CALIBRATION_CURRENT_A*16384/20));
    feedback.velocity=-calibration_speed;step(&dev,2);assert(sent==0);
    feedback.velocity=0;dev.ctrl_param.pid_vel.kp=.2f;
    if (argc > 1 && strcmp(argv[1],"calibration_fast_feedback")==0) {
        const float velocities[] = {-31.0f,31.0f,-100.0f,100.0f};
        for (unsigned i=0;i<sizeof(velocities)/sizeof(velocities[0]);++i) {
            feedback.velocity=velocities[i];step(&dev,2);
            assert(dev.ctrl_param._enable_flag && dev.ctrl_param._is_calibrating);
            char reason[33];assert(C620_Service(&dev,"c620_fault",0,reason,sizeof(reason)));
            assert(strcmp(reason,"fault=0")==0);
        }
        feedback.velocity=0;limit_switch=true;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._is_calibrating && origin_resets==1);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1],"calibration_wrong_mode")==0) {
        dev.ctrl_param.ctrl_type=ROBOMAS_CTRL_POS;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        char reason[33];assert(C620_Service(&dev,"c620_fault",0,reason,sizeof(reason)));
        assert(strcmp(reason,"fault=4")==0);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1],"overspeed")==0) {
        feedback.velocity=-151;step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        RoboMas_ControlEnable(&dev);assert(!dev.ctrl_param._enable_flag);return 0;
    }
    if (argc > 1 && strcmp(argv[1],"no_stall")==0) {
        feedback.velocity=-10;step(&dev,5000);
        assert(dev.ctrl_param._is_calibrating);
        feedback.velocity=0;step(&dev,C620_CALIBRATION_STALL_MS-1);
        assert(dev.ctrl_param._is_calibrating && sent!=0);
        step(&dev,1);assert(sent==0 && origin_resets==0);
        step(&dev,C620_TIMEOUT_STOP_SETTLE_MS);
        assert(sent==0 && !dev.ctrl_param._is_calibrating && !dev.ctrl_param._enable_flag);
        char source[33]; assert(C620_Service(&dev,"c620_origin",0,source,sizeof(source)));
        assert(strcmp(source,"stall_unverified")==0 && origin_resets==1);
        puts("C620 calibration: moving continues; 3s stall plus 500ms zero-current settle");return 0;
    }
    if (argc > 1 && strncmp(argv[1],"timeout",7)==0) {
        feedback.position=-10;feedback.velocity=0;
        step(&dev,C620_CALIBRATION_TIMEOUT_MS+1);
        assert(sent==0 && dev.ctrl_param._is_calibrating && origin_resets==0);
        if (strcmp(argv[1],"timeout_moving")==0) feedback.velocity=-10;
        if (strcmp(argv[1],"timeout_stale")==0) fresh=false;
        if (strcmp(argv[1],"timeout_nan")==0) feedback.current=NAN;
        if (strcmp(argv[1],"timeout_cancel")==0) { C620_CancelRequest(); RoboMas_ControlDisable(&dev); }
        if (strcmp(argv[1],"timeout")==0) {
            step(&dev,C620_TIMEOUT_STOP_SETTLE_MS-1);assert(sent==0 && origin_resets==0);
            step(&dev,1);
            assert(sent==0 && origin_resets==1 && feedback.position==0);
            assert(!dev.ctrl_param._is_calibrating && dev.ctrl_param.ctrl_type==ROBOMAS_CTRL_POS_MPC);
            assert(dev.ctrl_param._enable_flag==(bool)INTEGRATED_STARTUP_HOLD);
            char source[33]; assert(C620_Service(&dev,"c620_origin",0,source,sizeof(source)));
            assert(strcmp(source,"timeout_unverified")==0);
            step(&dev,2);assert(origin_resets==1);
        } else {
            step(&dev,C620_TIMEOUT_STOP_LIMIT_MS);
            assert(sent==0 && origin_resets==0 && !dev.ctrl_param._enable_flag);
            RoboMas_ControlEnable(&dev);assert(!dev.ctrl_param._enable_flag);
        }
        puts("PASS: timeout origin or fault separation");return 0;
    }
    if (argc > 1 && strcmp(argv[1],"stale")==0) {
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
    if (argc > 1 && strncmp(argv[1],"pulse",5)==0) {
        assert(service(&dev,"c620_restore",175));
        limit_switch=false;feedback.position=40;feedback.velocity=0;
        assert(!service(&dev,"c620_pulse",801));
        const float speed=strcmp(argv[1],"pulse800")==0 ? 800 : 400;
        assert(service(&dev,"c620_pulse",speed));
        step(&dev,2);assert(dev.ctrl_param._enable_flag);
        assert(dev.ctrl_param._target_value==speed);
        if (strcmp(argv[1],"pulse_stale")==0) {
            fresh=false;step(&dev,2);assert(!dev.ctrl_param._enable_flag);return 0;
        }
        if (strcmp(argv[1],"pulse_cancel")==0) {
            RoboMas_ControlDisable(&dev);step(&dev,2);assert(!dev.ctrl_param._enable_flag);return 0;
        }
        if (strcmp(argv[1],"pulse_bounds")==0) {
            feedback.position=156;step(&dev,2);assert(!dev.ctrl_param._enable_flag);return 0;
        }
        if (strcmp(argv[1],"pulse_time")==0) step(&dev,400);
        else {feedback.position=86;feedback.velocity=speed;step(&dev,2);}
        assert(dev.ctrl_param._enable_flag && dev.ctrl_param._target_value==0);
        if (strcmp(argv[1],"pulse_timeout")==0) {
            step(&dev,1000);assert(!dev.ctrl_param._enable_flag);return 0;
        }
        feedback.velocity=0;step(&dev,202);assert(!dev.ctrl_param._enable_flag);
        char msg[33];assert(C620_Service(&dev,"c620_pulse_fb",0,msg,sizeof(msg)));
        assert(strstr(msg,"done=1"));puts(msg);return 0;
    }
    if (argc > 1 && strcmp(argv[1],"mpc_zero")==0) {
        assert(service(&dev,"c620_restore",175));
        limit_switch=false;feedback.position=10;feedback.velocity=0;
        RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_POS_MPC);
        RoboMas_SetTarget(&dev,0);RoboMas_ControlEnable(&dev);step(&dev,2);
        assert(dev.ctrl_param._enable_flag);
        limit_switch=true;feedback.position=.1f;step(&dev,2);
        assert(sent==0 && dev.ctrl_param._enable_flag && C620_EnableAllowed());
        assert(dev.ctrl_param._target_valid && dev.ctrl_param._target_value==0);
        step(&dev,100);
        assert(sent==0 && dev.ctrl_param._enable_flag);
        /* スイッチがまだ押されていても、位置指令だけで外向きへ復帰する。 */
        RoboMas_SetTarget(&dev,170);
        assert(!C620_GuardZero(&dev,&feedback));
        assert(dev.ctrl_param._enable_flag && dev.ctrl_param._target_value==170);
        /* 明示Disableを後続の位置指令で解除しない。 */
        RoboMas_ControlDisable(&dev);
        RoboMas_SetTarget(&dev,170);step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1],"restore")==0) {
        assert(!service(&dev,"c620_restore",176));
        assert(service(&dev,"c620_restore",175));
        assert(!service(&dev,"c620_restore",175));
        feedback.position=90; feedback.velocity=31;
        RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_VEL_DOB);
        RoboMas_SetTarget(&dev,0);RoboMas_ControlEnable(&dev);step(&dev,2);
        assert(sent==0 && !dev.ctrl_param._enable_flag);
        char reason[33];assert(C620_Service(&dev,"c620_fault",0,reason,sizeof(reason)));
        assert(strcmp(reason,"fault=9")==0);return 0;
    }
    const bool survey5 = argc > 1 && strcmp(argv[1],"survey5")==0;
    assert(!service(&dev,"c620_survey",C620_SURVEY_CURRENT_A+1));
    assert(service(&dev,"c620_survey",survey5 ? 1 : 0));
    step(&dev,2); assert(sent==0);
    step(&dev,500); assert(sent==(survey5 ? -819 : -1638));
    dev.ctrl_param.pid_vel.kp=2;step(&dev,2);
    assert(sent==(survey5 ? -819 : -(int)(C620_SURVEY_CURRENT_A*16384/20)));
    feedback.velocity=10;step(&dev,2);assert(sent==0);
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
    feedback.current=survey5 ? -1 : -C620_SURVEY_CURRENT_A;
    step(&dev,600);assert(sent==0 && !dev.ctrl_param._enable_flag);
    assert(service(&dev,"c620_survey",survey5 ? 1 : 0));
    assert(!service(&dev,"c620_range",95));
    step(&dev,2);assert(sent==0 && dev.ctrl_param._enable_flag);
    step(&dev,600);assert(dev.ctrl_param._enable_flag);
    step(&dev,600);assert(sent==0 && !dev.ctrl_param._enable_flag);
    assert(!service(&dev,"c620_range",96));
    assert(service(&dev,"c620_range",95));
    RoboMas_ChangeControl(&dev,ROBOMAS_CTRL_CURRENT);
    assert(service(&dev,"c620_current",C620_RUN_CURRENT_A));
    RoboMas_SetTarget(&dev,-100);RoboMas_ControlEnable(&dev);step(&dev,2);
    assert(sent==(int)(C620_RUN_CURRENT_A*16384/20));
    RoboMas_ControlDisable(&dev);step(&dev,2);assert(sent==0);
    RoboMas_send_current(&dev,20,&hcan2);assert(sent==0);
    feedback.position=50;RoboMas_SetTarget(&dev,NAN);RoboMas_ControlEnable(&dev);step(&dev,2);assert(sent==0);
    puts("C620 commissioning/current-frame checks passed");
}
