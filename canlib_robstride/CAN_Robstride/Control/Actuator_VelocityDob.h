#ifndef ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H
#define ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H

#include <stdbool.h>

/* Robstride用の速度DOB制御器パラメータ。速度は内部でrad/sに変換して扱う。 */
typedef struct {
    float J;
    float d;
    float K_tau;
    float dob_bandwidth;
    float velocity_kp;
    float velocity_ki;
    float velocity_kd;
    float reference_alpha;

    /* 速度指令上限はrad/s、電流上限はA、トルク上限はNm。 */
    float velocity_reference_limit;
    float current_limit;
    float torque_limit;
    /* falseは無効。trueなら0.0も有効値として扱う（トルク上限は負値不可）。 */
    bool velocity_reference_limit_enable;
    bool current_limit_enable;
    bool torque_limit_enable;
    float velocity_unit_to_rad_s;

    /*
     * 設定上の公称周期[s]。実際の計算周期はUpdate()に渡すdtを使用する。
     * 呼び出し側の周期設定と診断用の基準値として保持する。
     */
    float control_period;
} Robstride_Actuator_VelocityDob_Parameters;

typedef struct {
    /* 一次遅れ基準モデルの状態[rad/s]。 */
    float omega_model;

    /* DOBのQフィルタ状態（トルク[Nm]、速度[rad/s]）。 */
    float dob_q_torque;
    float dob_q_velocity;

    /* PI/PID状態。積分値の単位は速度偏差[rad/s]・s。 */
    float integral;
    /* 前周期の速度偏差[rad/s]。 */
    float previous_error;
    /* 前周期に実際に適用した電流指令[A]。 */
    float previous_current;

    /* 外乱トルク推定値[Nm]。 */
    float disturbance_estimate;
    /* 基準モデル由来のフィードフォワードトルク[Nm]。 */
    float feedforward_torque;
    /* PIDのフィードバックトルク[Nm]。 */
    float feedback_torque;
    /* disturbance_estimateと同じ値を保持する互換用フィールド[Nm]。 */
    float dob_torque;
    /* 出力上限適用前の合成トルク[Nm]。 */
    float unsaturated_torque;
    /* 出力上限適用後のトルク[Nm]。 */
    float final_torque;
    /* 実際に返す電流指令[A]。 */
    float final_current;
    /* 一次遅れQフィルタの係数。DOB無効時はdob_beta=1.0。 */
    float dob_beta;
    /* 一次遅れ基準モデルの係数。モデル無効時はreference_beta=0.0。 */
    float reference_beta;

    /* 初回更新済みフラグ。 */
    bool initialized;
    /* unsaturated_torqueが上限でクリップされたか。 */
    bool output_saturated;
    /* 飽和をさらに進める積分を停止したか。 */
    bool anti_windup_active;
} Robstride_Actuator_VelocityDob_State;

void Robstride_Actuator_VelocityDob_Reset(
    Robstride_Actuator_VelocityDob_State *state);

/* 戻り値はF7から送る電流指令[A]。 */
float Robstride_Actuator_VelocityDob_Update(
    const Robstride_Actuator_VelocityDob_Parameters *parameters,
    Robstride_Actuator_VelocityDob_State *state,
    float velocity_ref,
    float velocity_measured,
    float dt);

#endif /* ROBSTRIDE_ACTUATOR_VELOCITY_DOB_H */
