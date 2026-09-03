// Includes --------------------------------

#include <main.h> // HALライブラリの定義が含まれているヘッダ

#include <CAN_Robstride.h>
#include <CAN_Robstride_Def.h>
#include <CAN_Robstride_System.h>
#include <robstride_constant.h>
#include <Robstride_Control.h>

#include <math.h>  // 数学関数 (fabsf, fmaxf, fminf など) を使用するためにインクルード
#include <stdio.h> // 標準入出力関数 (printf など) を使用するためにインクルード

// Private Function Prototypes --------------------------------

#define ROBSTRIDE_CONTROL_RETRY_LIMIT 1000U

static void Robstride_PID_Ctrl_init(Robstride_PID_StructTypedef *const params);
static void Robstride_Ctrl_Struct_init(Robstride_Ctrl_StructTypedef *ctrl_struct);
// static void Check_CAN_Error(CAN_HandleTypeDef *phcan);
// static void Print_CAN_BitTiming_Params(CAN_HandleTypeDef *hcan);
static uint8_t Robstride_get_switch_state(GPIO_TypeDef *limit_port, uint32_t limit_pin, ROBSTRIDE_SWITCH_TYPE sw_type);
static void Robstride_SetMechposToZero(Robstride_DeviceInfo *dev_info, DelayFunction_t f_delay);

// Functions --------------------------------

/**
 * @brief Robstride PID制御パラメータ構造体を初期化します。
 * @param params 初期化対象のRobstride_PID_StructTypedef構造体へのポインタ
 * @retval なし
 */
static void Robstride_PID_Ctrl_init(Robstride_PID_StructTypedef *const params) {
    params->kp_pos = 0.5f;      // 位置制御Pゲインのデフォルト値
    params->kp_vel = 10.0f;     // 速度制御Pゲインのデフォルト値
    params->ki_vel = 5.0f;      // 速度制御Iゲインのデフォルト値
    params->filter_vel = 1.0f;  // 速度制御フィルタゲインのデフォルト値
    params->kp_cur = 0.05f;     // 電流制御Pゲインのデフォルト値
    params->ki_cur = 0.05f;     // 電流制御Iゲインのデフォルト値
    params->filter_cur = 0.06f; // 電流制御フィルタゲインのデフォルト値
    params->_integral = 0.0f;   // 積分項の初期値
    params->_prev_value = 0.0f; // 前回値の初期値
}

/**
 * @brief Robstride制御構造体を初期化します。
 * @param ctrl_struct 初期化対象のRobstride_Ctrl_StructTypedef構造体へのポインタ
 * @retval なし
 */
static void Robstride_Ctrl_Struct_init(Robstride_Ctrl_StructTypedef *const ctrl_struct) {
    ctrl_struct->_target_value = 0.0f;            // 目標値の初期値
    ctrl_struct->_enable_flag = 0;                // 有効フラグの初期値 (無効)
    Robstride_PID_Ctrl_init(&(ctrl_struct->pid)); // PIDパラメータ構造体を初期化
}

/**
 * @brief Robstrideデバイス情報配列を初期化します。
 * @param dev_info_array Robstrideデバイス情報構造体の配列
 * @param size 配列のサイズ
 * @retval なし
 */
void Robstride_Init(Robstride_DeviceInfo dev_info_array[], const uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        Robstride_Ctrl_Struct_init(&(dev_info_array[i].ctrl_param)); // 各デバイスの制御構造体を初期化
    }
}

void Robstride_WaitForConnect(Robstride_DeviceInfo dev_info_array[], const uint8_t size, DelayFunction_t f_delay) {
    uint8_t flag = 0;
    printf("[Robstride] Wait for Connection...\r\n");
    while (!flag) {
        flag = 1;
        for (uint8_t i = 0; i < size; i++) {
            /*
             * 電源投入前は応答が来ないため、応答待ちの Robstride_ControlDisable()
             * をここで使うと以後の再送が止まる。リセット要求を1回だけ送信し、
             * 次の周回で再試行する。
             */
            const uint8_t data[8] = {0};
            Robstride_SendBytes(dev_info_array[i].phcan,
                                dev_info_array[i].device_id,
                                CMD_RESET,
                                dev_info_array[i].master_id,
                                data,
                                sizeof(data));
            // printf("Checking Active Report Status for motor %d...\n\r", i + 1);
            // Robstride_CheckActiveReportStatus(&dev_info_array[i]);
            f_delay(50); // 応答を待つためのディレイ
            // Check_CAN_Error(dev_info_array[i].phcan); // CANエラーをチェック
            // Print_CAN_BitTiming_Params(dev_info_array[i].phcan); // CANビットタイミングパラメータを表示
            if (!Read_Robstride_FeedbackData(&dev_info_array[i]).get_flag) {
                printf("[Robstride] Device id : %d Not Connected...\r\n", dev_info_array[i].device_id);
                // 1台でも接続されていなければ (get_flagが0なら)、フラグを0にしてループを継続
                flag = 0;
            } else {
                printf("[Robstride] Device id : %d Connected!\r\n", dev_info_array[i].device_id);
            }
        }
        f_delay(100);
    }

    printf("[Robstride] All Connected!\r\n");
    f_delay(500);
}

#if 0 // 使われていないので一旦コメントアウト
/**
 * @brief CANのエラーステータスを確認し、内容をシリアルコンソールに出力します。
 * @param hcan 確認対象のCANハンドル
 */
static void Check_CAN_Error(CAN_HandleTypeDef *const phcan) {
    // HAL_CAN_GetError() を呼び出して現在のエラーコードを取得
    const uint32_t can_error = HAL_CAN_GetError(phcan);

    // エラーが存在するかどうかを確認
    if (can_error == HAL_CAN_ERROR_NONE) {
        // エラーなし
        // printf("CAN No Error.\r\n"); // 正常時に毎回表示するとログが煩雑になるため、コメントアウトを推奨
        return; // エラーがなければここで処理終了
    }

    // --- 何らかのエラーが検出された場合、以下で詳細を判別 ---

    printf("--- CAN Error Detected! Code: 0x%08lX ---\r\n", can_error);

    // === バス状態とプロトコルに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_BOF) != 0) {
        printf("  - Bus-off error: バスから切り離されました。重大なエラーです。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_EPV) != 0) {
        printf("  - Error Passive: エラーパッシブ状態です (RECまたはTEC > 127)。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_EWG) != 0) {
        printf("  - Protocol Error Warning: エラーワーニング状態です (RECまたはTEC > 95)。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_STF) != 0) {
        printf("  - Stuff error: スタッフビットのルール違反を検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_FOR) != 0) {
        printf("  - Form error: フレームフォーマットの固定ビットが不正です。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_ACK) != 0) {
        printf("  - Acknowledgment error: 送信メッセージに対しACKが返されませんでした。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_BR) != 0) {
        printf("  - Bit recessive error: ドミナントビットを送信中にリセッシブを検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_BD) != 0) {
        printf("  - Bit dominant error: リセッシブビットを送信中にドミナントを検出しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_CRC) != 0) {
        printf("  - CRC error: 受信メッセージのCRCが一致しませんでした。\r\n");
    }

    // === 受信FIFOに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_RX_FOV0) != 0) {
        printf("  - Rx FIFO0 overrun error: 受信FIFO0がオーバーランしました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_RX_FOV1) != 0) {
        printf("  - Rx FIFO1 overrun error: 受信FIFO1がオーバーランしました。\r\n");
    }

    // === 送信メールボックスに関するエラー ===
    if ((can_error & HAL_CAN_ERROR_TX_ALST0) != 0) {
        printf("  - TxMailbox 0 arbitration lost: 送信メールボックス0が調停に失敗しました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR0) != 0) {
        printf("  - TxMailbox 0 transmit error: 送信メールボックス0で送信エラーが発生しました。\r\n");
    }
    // メールボックス1と2も同様にチェック
    if ((can_error & HAL_CAN_ERROR_TX_ALST1) != 0) {
        printf("  - TxMailbox 1 arbitration lost\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR1) != 0) {
        printf("  - TxMailbox 1 transmit error\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_ALST2) != 0) {
        printf("  - TxMailbox 2 arbitration lost\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_TX_TERR2) != 0) {
        printf("  - TxMailbox 2 transmit error\r\n");
    }

    // === HALドライバ内部のエラー ===
    if ((can_error & HAL_CAN_ERROR_TIMEOUT) != 0) {
        printf("  - Timeout error: 処理がタイムアウトしました。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_INITIALIZED) != 0) {
        printf("  - Peripheral not initialized: ペリフェラルが初期化されていません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_READY) != 0) {
        printf("  - Peripheral not ready: ペリフェラルが準備完了状態ではありません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_NOT_STARTED) != 0) {
        printf("  - Peripheral not started: ペリフェラルが開始されていません。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_PARAM) != 0) {
        printf("  - Parameter error: 関数の引数が不正です。\r\n");
    }
    if ((can_error & HAL_CAN_ERROR_INTERNAL) != 0) {
        printf("  - Internal error: HALドライバの内部エラーです。\r\n");
    }

    printf("-----------------------------------------\r\n\r\n");
}

/**
 * @brief  現在動作中のCANペリフェラルのビットタイミング設定をレジスタから直接読み出し、
 * 計算されたボーレートなどと共にシリアルコンソールに出力します。
 * @param  hcan 確認対象のCANハンドル
 */
static void Print_CAN_BitTiming_Params(CAN_HandleTypeDef *const hcan) {
    uint32_t can_btr_reg;
    uint32_t pclk1_freq;

    // CAN_BTRレジスタの現在の値を読み出す
    can_btr_reg = hcan->Instance->BTR;

    // --- レジスタ値から各パラメータを抽出 ---
    // CAN_BTRレジスタのビット配置
    // | 31 | 30 | 29..26 | 25..24 | 23 | 22..20 | 19..16 | 15..10 | 9..0  |
    // | SILM | LBKM | -      | SJW    | -  | TS2    | TS1    | -      | BRP   |

    // ボーレートプリスケーラ (BRP)
    const uint32_t brp = (can_btr_reg & CAN_BTR_BRP);
    const uint32_t prescaler = brp + 1;

    // タイムセグメント1 (TS1)
    const uint32_t ts1 = (can_btr_reg & CAN_BTR_TS1) >> 16;
    const uint32_t time_seg1 = ts1 + 1;

    // タイムセグメント2 (TS2)
    const uint32_t ts2 = (can_btr_reg & CAN_BTR_TS2) >> 20;
    const uint32_t time_seg2 = ts2 + 1;

    // 再同期ジャンプ幅 (SJW)
    const uint32_t sjw = (can_btr_reg & CAN_BTR_SJW) >> 24;
    const uint32_t sync_jump_width = sjw + 1;

    // --- 計算 ---
    // APB1クロック周波数を取得 (CANペリフェラルのクロックソース)
    pclk1_freq = HAL_RCC_GetPCLK1Freq();

    // 1ビットあたりの合計タイムクアンタ数 (SYNC_SEGは常に1 Tq)
    const uint32_t total_tq = 1 + time_seg1 + time_seg2;

    // ボーレートの計算
    // BaudRate = PCLK1 / (Prescaler * (1 + TS1 + TS2))
    const uint32_t baud_rate = pclk1_freq / (prescaler * total_tq);

    // サンプルポイントの計算
    // SamplePoint = (1 + TS1) / (1 + TS1 + TS2) * 100
    const float sample_point = (float)(1 + time_seg1) * 100.0f / (float)total_tq;

    // --- 結果の表示 ---
    printf("--- Current CAN Bit Timing Parameters ---\r\n");
    printf("  [System]\r\n");
    printf("    APB1 Clock (PCLK1) : %lu Hz\r\n", pclk1_freq);
    printf("\r\n");
    printf("  [Register Raw Values (from CAN_BTR)]\r\n");
    printf("    BRP: %lu, TS1: %lu, TS2: %lu, SJW: %lu\r\n", brp, ts1, ts2, sjw);
    printf("\r\n");
    printf("  [Calculated Parameters]\r\n");
    printf("    Prescaler          : %lu\r\n", prescaler);
    printf("    Time Segment 1     : %lu Tq\r\n", time_seg1);
    printf("    Time Segment 2     : %lu Tq\r\n", time_seg2);
    printf("    Sync Jump Width    : %lu Tq\r\n", sync_jump_width);
    printf("    Total Time Quanta  : %lu Tq/bit\r\n", total_tq);
    printf("\r\n");
    printf("  [Result]\r\n");
    printf("    Calculated Baud Rate: %lu bps (%lu kbps)\r\n", baud_rate, baud_rate / 1000);
    printf("    Sample Point        : %.2f %%\r\n", sample_point);
    printf("-----------------------------------------\r\n\r\n");
}
#endif

/**
 * @brief リミットスイッチの状態を取得します。
 * @param limit_port リミットスイッチが接続されているGPIOポート
 * @param limit_pin リミットスイッチが接続されているGPIOピン
 * @param sw_type スイッチのタイプ (ノーマリオープンまたはノーマリクローズ)
 * @retval uint8_t スイッチの状態 (アクティブなら1, 非アクティブなら0)
 */
static uint8_t Robstride_get_switch_state(GPIO_TypeDef *const limit_port, const uint32_t limit_pin, const ROBSTRIDE_SWITCH_TYPE sw_type) {
    if (sw_type == ROBSTRIDE_SWITCH_NO) {                // ノーマリオープン (Normally Open) の場合
        return HAL_GPIO_ReadPin(limit_port, limit_pin);  // ピンがHighなら1 (アクティブ), Lowなら0 (非アクティブ)
    } else {                                             // ノーマリクローズ (Normally Close) の場合
        return !HAL_GPIO_ReadPin(limit_port, limit_pin); // ピンがLowなら1 (アクティブ), Highなら0 (非アクティブ)
    }
}

/**
 * @brief Robstrideの機械的位置をゼロに設定します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
static void Robstride_SetMechposToZero(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    uint8_t data[8] = { 0x00 }; // 送信するデータ配列を初期化
    data[0] = 0x01;             // コマンドデータ (機械的位置をゼロにするための特定データ)
    // CANメッセージを送信 (CMD_SET_MECH_POSITION_TO_ZERO コマンド)
    while (1) {
        Robstride_SendBytes(dev_info->phcan, dev_info->device_id, CMD_SET_MECH_POSITION_TO_ZERO, dev_info->master_id, (uint8_t *)data, sizeof(data));
        f_delay(1); // 送信後に少し待機
        // フィードバックデータを読み取る (機械的位置がゼロに設定されたか確認)
        if (fabs(Read_Robstride_FeedbackData(dev_info).position - dev_info->ctrl_param.offset_pos) < 0.1f) {
            break; // 機械的位置がゼロに設定されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideのパラメータをプリセットします（PIDパラメータ設定、制御モード設定、フィードバック初期化）。Control_Disable状態でのみ設定可能です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_PresetParameters(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    Robstride_SetPIDParams(dev_info, f_delay);                               // PIDパラメータを設定
    Robstride_SetControl(dev_info, dev_info->ctrl_param.ctrl_type, f_delay); // 制御モードを設定
    Robstride_fb_init(dev_info);                                             // フィードバックを初期化
}

/**
 * @brief RobstrideのPIDパラメータを設定します。Control_Disable状態でのみ設定可能です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetPIDParams(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    Robstride_WriteFloatData(dev_info, ADDR_LOC_KP, dev_info->ctrl_param.pid.kp_pos);                  // 位置制御Pゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_KP, dev_info->ctrl_param.pid.kp_vel);                  // 速度制御Pゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_KI, dev_info->ctrl_param.pid.ki_vel);                  // 速度制御Iゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_SPD_FILTER_GAIN, dev_info->ctrl_param.pid.filter_vel);     // 速度制御フィルタゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_CURRENT_KP, dev_info->ctrl_param.pid.kp_cur);              // 電流制御Pゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_CURRENT_KI, dev_info->ctrl_param.pid.ki_cur);              // 電流制御Iゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
    Robstride_WriteFloatData(dev_info, ADDR_CURRENT_FILTER_GAIN, dev_info->ctrl_param.pid.filter_cur); // 電流制御フィルタゲインを設定
    f_delay(10);                                                                                       // 書き込み後に少し待機
}

/**
 * @brief Robstrideの速度制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetVelocityLimit(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    switch (dev_info->device) {                                                            // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                                 // Robstride_02 の場合
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) { // 速度制限が無効の場合
                dev_info->ctrl_param.velocity_limit_size = 44.0f;                          // デフォルトの最大制限値を設定
            } else {                                                                       // 速度制限が有効の場合
                // 設定値を 0.0f と 44.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 44.0f), 0.0f);
            }
            break;
        case Robstride_04:                                                                 // Robstride_04 の場合
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) { // 速度制限が無効の場合
                dev_info->ctrl_param.velocity_limit_size = 15.0f;                          // デフォルトの最大制限値を設定
            } else {                                                                       // 速度制限が有効の場合
                // 設定値を 0.0f と 15.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 15.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.velocity_limit == ROBSTRIDE_VELOCITY_LIMIT_DISABLE) {
                dev_info->ctrl_param.velocity_limit_size = 50.0f; // Max Speed: 50 rad/s
            } else {
                // 設定値を 0.0f と 50.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.velocity_limit_size, 50.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    while (1) {
        Robstride_WriteFloatData(dev_info, ADDR_LIMIT_SPEED, dev_info->ctrl_param.velocity_limit_size);                  // 速度制限値を書き込み
        Robstride_RequestReadParameter(dev_info, ADDR_LIMIT_SPEED);                                                      // 書き込み後に読み出し要求を送信して、設定が反映されたか確認
        f_delay(1);                                                                                                      // 書き込み後に少し待機
        if (fabsf(Read_Robstride_FeedbackData(dev_info).limit_spd - dev_info->ctrl_param.velocity_limit_size) < 0.01f) { // 設定が反映されたか確認
            break;                                                                                                       // 反映されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideの電流制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetCurrentLimit(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    switch (dev_info->device) {                                                          // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                               // Robstride_02 の場合
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) { // 電流制限が無効の場合
                dev_info->ctrl_param.current_limit_size = 23.0f;                         // デフォルトの最大制限値を設定
            } else {                                                                     // 電流制限が有効の場合
                // 設定値を 0.0f と 23.0f の間にクリッピング
                dev_info->ctrl_param.current_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 23.0f), 0.0f);
            }
            break;
        case Robstride_04: // Robstride_04 の場合
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) {
                // 電流制限が無効の場合
                // TODO: 注意: ここでは velocity_limit_size が使われているが、current_limit_size の誤りと思われる。
                dev_info->ctrl_param.velocity_limit_size = 90.0f; // デフォルトの最大制限値を設定
            } else {
                // 電流制限が有効の場合
                // TODO: 注意: ここでは velocity_limit_size が使われているが、current_limit_size の誤りと思われる。
                // 設定値を 0.0f と 90.0f の間にクリッピング
                dev_info->ctrl_param.velocity_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 90.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.current_limit == ROBSTRIDE_CURRENT_LIMIT_DISABLE) {
                dev_info->ctrl_param.current_limit_size = 11.0f; // Max Current: 11 A
            } else {
                dev_info->ctrl_param.current_limit_size = fmaxf(fminf(dev_info->ctrl_param.current_limit_size, 11.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    while (1) {
        Robstride_WriteFloatData(dev_info, ADDR_LIMIT_CURRENT, dev_info->ctrl_param.current_limit_size); // 電流制限値を書き込み
        Robstride_RequestReadParameter(dev_info, ADDR_LIMIT_CURRENT);                                    // 書き込み後に読み出し要求を送信して、設定が反映されたか確認
        f_delay(1);                                                                                      // 書き込み後に少し待機
        if (fabsf(Read_Robstride_FeedbackData(dev_info).limit_cur - dev_info->ctrl_param.current_limit_size) < 0.01f) {
            // 設定が反映されたか確認
            break; // 反映されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideのトルク制限を設定します。Control_Disable時にのみ有効です。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_SetTorqueLimit(Robstride_DeviceInfo *const dev_info) {
    switch (dev_info->device) {                                                        // デバイスの種類によって制限値の上限が異なる
        case Robstride_02:                                                             // Robstride_02 の場合
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) { // トルク制限が無効の場合
                dev_info->ctrl_param.torque_limit_size = 17.0f;                        // デフォルトの最大制限値を設定
            } else {                                                                   // トルク制限が有効の場合
                // 設定値を 0.0f と 17.0f の間にクリッピング
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 17.0f), 0.0f);
            }
            break;
        case Robstride_04:                                                             // Robstride_04 の場合
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) { // トルク制限が無効の場合
                dev_info->ctrl_param.torque_limit_size = 120.0f;                       // デフォルトの最大制限値を設定
            } else {                                                                   // トルク制限が有効の場合
                // 設定値を 0.0f と 120.0f の間にクリッピング
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 120.0f), 0.0f);
            }
            break;
        case Robstride_05_Edu:
            if (dev_info->ctrl_param.torque_limit == ROBSTRIDE_TORQUE_LIMIT_DISABLE) {
                dev_info->ctrl_param.torque_limit_size = 6.0f; // Max Torque: 6 Nm
            } else {
                dev_info->ctrl_param.torque_limit_size = fmaxf(fminf(dev_info->ctrl_param.torque_limit_size, 6.0f), 0.0f);
            }
            break;
            // defaultケースが抜けているため、Robstride_04以外のデバイスタイプの場合、処理がスキップされる。
        default:
            break;
    }
    Robstride_WriteFloatData(dev_info, ADDR_LIMIT_TORQUE, dev_info->ctrl_param.torque_limit_size); // トルク制限値を書き込み
    // TODO: Robstride_SetVelocityLimit(), SetCurrentLimit() のように書き込みを確認しなくてもよいのか？
}

/**
 * @brief Robstrideのキャリブレーション（原点出し）を実行します。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 * @param calib_velocity キャリブレーション時の速度
 * @param sw_type リミットスイッチのタイプ
 * @param limit_port リミットスイッチのGPIOポート
 * @param limit_pin リミットスイッチのGPIOピン
 * @retval なし
 */
void Robstride_Calibration(Robstride_DeviceInfo *const device_info, float calib_velocity, const ROBSTRIDE_SWITCH_TYPE sw_type, GPIO_TypeDef *const limit_port, const uint32_t limit_pin, DelayFunction_t f_delay) {
    switch (device_info->ctrl_param.use_internal_offset) {
        case ROBSTRIDE_USE_OFFSET_POS_INTERNAL:               // 内部オフセットを使用する場合
            return;                                           // キャリブレーション不要
        case ROBSTRIDE_USE_OFFSET_POS_INITIAL:                // 初期位置をオフセットとして使用する場合
            Robstride_SetMechposToZero(device_info, f_delay); // 現在の機械的位置をゼロに設定
            return;                                           // キャリブレーション完了
        case ROBSTRIDE_USE_OFFSET_POS_CALIB:                  // キャリブレーションによりオフセットを設定する場合
            break;                                            // キャリブレーション処理へ進む
        default:                                              // その他の場合
            return;                                           // 何もしない
    }
    Robstride_SetControl(device_info, ROBSTRIDE_CTRL_VEL, f_delay); // 制御モードを速度制御に設定
    Robstride_ControlEnable(device_info, f_delay);                  // モータ制御を有効化
    if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) {     // 回転方向が時計回り(CW)の場合
        calib_velocity *= -1;                                       // キャリブレーション速度を反転
    }
    Robstride_SetTarget(device_info, calib_velocity); // 目標速度を設定
    while (!Robstride_get_switch_state(limit_port, limit_pin, sw_type)) {
        f_delay(1);
    } // リミットスイッチが押されるまで待機
    Robstride_ControlDisable(device_info, f_delay);   // モータ制御を無効化
    Robstride_SetMechposToZero(device_info, f_delay); // 現在の機械的位置をゼロに設定 (原点確定)
    return;
}

/**
 * @brief Robstrideの制御モードを設定します。
 * この関数はPIDパラメータなどの再初期化は行いません。
 * 制御モード変更時にPIDパラメータをリセットしたい場合は Robstride_ChangeControl を使用してください。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @param new_ctrl_type 新しい制御モード
 * @retval なし
 */
void Robstride_SetControl(Robstride_DeviceInfo *const dev_info, const ROBSTRIDE_CTRL_TYPE new_ctrl_type, DelayFunction_t f_delay) {
    /*
     * ctrl_type は設定値であり、モーター側の現在値ではない。起動時には
     * 同じ値でも必ず CAN 経由で書き込んで、電源投入直後のモーターへ反映する。
     */
    Robstride_ControlDisable(dev_info, f_delay);                              // control_disable時のみ制御モードが変更可能
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;                           // 新しい制御モードを設定
    Robstride_WriteIntData(dev_info, ADDR_RUN_MODE, (uint16_t)new_ctrl_type); // 実行モード (制御モード) を書き込み
    Robstride_ControlEnable(dev_info, f_delay);                               // モータ制御を有効化
}

/**
 * @brief Robstrideの制御モードを変更し、関連する制御パラメータを初期化します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @param new_ctrl_type 新しい制御モード
 * @retval なし
 */
void Robstride_ChangeControl(Robstride_DeviceInfo *const dev_info, const ROBSTRIDE_CTRL_TYPE new_ctrl_type, DelayFunction_t f_delay) {
    Robstride_Ctrl_Struct_init(&(dev_info->ctrl_param)); // 制御パラメータ構造体を初期化 (PIDパラメータなども初期値に戻る)
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;      // 新しい制御モードを設定
    Robstride_ControlDisable(dev_info, f_delay);         // control_disable時のみ制御モードが変更可能
    while (1) {
        Robstride_WriteIntData(dev_info, ADDR_RUN_MODE, (uint16_t)new_ctrl_type);        // 実行モード (制御モード) を書き込み
        f_delay(1);                                                                      // 書き込み後に少し待機
        Robstride_RequestReadParameter(dev_info, ADDR_RUN_MODE);                         // 書き込み後に読み出し要求を送信して、設定が反映されたか確認
        if (Read_Robstride_FeedbackData(dev_info).run_mode == (uint16_t)new_ctrl_type) { // 設定が反映されたか確認
            break;                                                                       // 反映されたらループを抜ける
        }
    }
}

/**
 * @brief Robstrideの目標値を設定します。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 * @param target_value 設定する目標値
 * @retval なし
 */
void Robstride_SetTarget(Robstride_DeviceInfo *const device_info, const float target_value) {
    device_info->ctrl_param._target_value = target_value; // 内部の目標値を更新
    float value = device_info->ctrl_param._target_value;  // 送信する値を準備
    uint16_t address;                                     // 送信先アドレス

    switch (device_info->ctrl_param.ctrl_type) {                        // 現在の制御モードに応じて処理を分岐
        case ROBSTRIDE_CTRL_POS:                                        // 位置制御モードの場合
            value -= device_info->ctrl_param.offset_pos;                // 位置オフセットを適用
            address = (uint16_t)ADDR_LOC_REF;                           // 位置指令アドレス
            value /= device_info->ctrl_param.quant_per_rot;             // 単位を回転量に変換 (例: 度から回転数へ)
            if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) { // 回転方向が時計回り(CW)の場合
                value *= -1;                                            // 指令値を反転
            }
            break;
        case ROBSTRIDE_CTRL_VEL:                                        // 速度制御モードの場合
            address = (uint16_t)ADDR_SPEED_REF;                         // 速度指令アドレス
            value /= device_info->ctrl_param.quant_per_rot;             // 単位を回転量/sに変換 (例: 度/sから回転数/sへ)
            if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) { // 回転方向が時計回り(CW)の場合
                value *= -1;                                            // 指令値を反転
            }
            break;
        case ROBSTRIDE_CTRL_CURRENT:                                    // 電流制御モードの場合
            address = (uint16_t)ADDR_IQ_REF;                            // Iq電流指令アドレス
            if (device_info->ctrl_param.rotation == ROBSTRIDE_ROT_CW) { // 回転方向が時計回り(CW)の場合
                value *= -1;                                            // 指令値を反転
            }
            break;
        case ROBSTRIDE_CTRL_OPERATION: // オペレーションモード (特殊なモード、詳細不明) の場合
            return;                    // 何もせずに終了
        default:                       // その他の制御モードの場合
            return;                    // 何もせずに終了
    }
    Robstride_WriteFloatData(device_info, address, value); // 計算された指令値を書き込み
    return;
}

/**
 * @brief Robstrideのモータ制御を有効にします。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_ControlEnable(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    uint8_t data[8] = { 0x00 }; // 送信するデータ配列を初期化 (内容はCMD_ENABLEでは使用されないことが多いが、形式として送信)
    // CANメッセージを送信 (CMD_ENABLE コマンド)
    uint32_t retry;
    for (retry = 0U; retry < ROBSTRIDE_CONTROL_RETRY_LIMIT; ++retry) {
        if (retry + 1U >= ROBSTRIDE_CONTROL_RETRY_LIMIT) {
            dev_info->ctrl_param._enable_flag = 0;
            printf("[Robstride] Device id : %d enable timeout; kept disabled.\r\n", dev_info->device_id);
            return;
        }
        Robstride_SendBytes(dev_info->phcan, dev_info->device_id, CMD_ENABLE, dev_info->master_id, (uint8_t *)data, sizeof(data));
        f_delay(1);                                                                        // 書き込み後に少し待機
        if (Read_Robstride_FeedbackData(dev_info).mode_status == ROBSTRIDE_STATE_ENABLE) { // モータが有効になったか確認
            break;                                                                         // 有効になったらループを抜ける
        }
    }
    dev_info->ctrl_param._enable_flag = 1; // 有効フラグを立てる
}

/**
 * @brief Robstrideのモータ制御を無効（リセット）します。
 * @param dev_info Robstrideデバイス情報構造体へのポインタ
 * @retval なし
 */
void Robstride_ControlDisable(Robstride_DeviceInfo *const dev_info, DelayFunction_t f_delay) {
    uint8_t data[8] = { 0x00 }; // 送信するデータ配列を初期化 (内容はCMD_RESETでは使用されないことが多いが、形式として送信)
    // CANメッセージを送信 (CMD_RESET コマンド)
    uint32_t retry;
    for (retry = 0U; retry < ROBSTRIDE_CONTROL_RETRY_LIMIT; ++retry) {
        if (retry + 1U >= ROBSTRIDE_CONTROL_RETRY_LIMIT) {
            dev_info->ctrl_param._enable_flag = 0;
            printf("[Robstride] Device id : %d disable timeout; kept disabled.\r\n", dev_info->device_id);
            return;
        }
        Robstride_SendBytes(dev_info->phcan, dev_info->device_id, CMD_RESET, dev_info->master_id, (uint8_t *)data, sizeof(data));
        f_delay(1);                                                                         // 書き込み後に少し待機
        if (Read_Robstride_FeedbackData(dev_info).mode_status == ROBSTRIDE_STATE_DISABLE) { // モータが無効になったか確認
            break;                                                                          // 無効になったらループを抜ける
        }
    }
    dev_info->ctrl_param._enable_flag = 0; // 有効フラグを降ろす
}

void Robstride_CheckActiveReportStatus(Robstride_DeviceInfo *const device_info) {
    Robstride_RequestReadParameter(device_info, ADDR_EPSCAN_TIME);
}

/**
 * @brief モーターに保存されている全ての読み取り可能なパラメータを要求します。
 * 要求するアドレスのリストは`robstride_constant.h`内の
 * `ROBSTRIDE_READABLE_ADDRESS_LIST`マクロで定義されています。
 * @param device_info Robstrideデバイス情報構造体へのポインタ
 */
void Robstride_RequestAllParameters(Robstride_DeviceInfo *const device_info, DelayFunction_t f_delay) {
    printf("--- Requesting all parameters for device ID: %d ---\n\r", device_info->device_id);

    // マクロを使用してアドレスの配列を初期化
    const uint16_t addresses[] = ROBSTRIDE_READABLE_ADDRESS_LIST;

    const uint8_t num_addresses = sizeof(addresses) / sizeof(addresses[0]);

    for (uint8_t i = 0; i < num_addresses; i++) {
        Robstride_RequestReadParameter(device_info, addresses[i]);
        f_delay(1); // モーターの応答とCANバスの負荷を考慮して短い遅延を入れる
    }
}
/**
 * @brief Robstride_FeedbackData構造体の内容をすべて表示します。
 * @param fb_data 表示するデータが格納された構造体へのポインタ
 */
/**
 * @brief Robstride_FeedbackData構造体の内容をすべて表示します。
 * ファームウェアのバージョンに応じて表示項目が切り替わります。
 * @param fb_data 表示するデータが格納された構造体へのポインタ
 */
void Robstride_PrintAllParameters(const Robstride_FeedbackData *const fb_data) {
    if (fb_data == NULL) {
        printf("Error: Feedback data pointer is NULL.\n\r");
        return;
    }
    printf("\n\r--- Robstride Parameters (Device ID: %d) ---\n\r", fb_data->device_id);
    printf(" [Real-time Data]\n\r");
    printf("  - Position:         %f\n\r", fb_data->position);
    printf("  - Velocity:         %f\n\r", fb_data->velocity);
    printf("  - Current (IqF):    %f\n\r", fb_data->current);
    printf("  - Torque (Type 2):  %f\n\r", fb_data->torque);
    printf("  - Temperature:      %d C\n\r", fb_data->temperature);
    printf("  - Bus Voltage:      %f V\n\r", fb_data->vbus);
    printf("\n\r [Control & Limit Settings]\n\r");
    printf("  - Run Mode:         %u\n\r", fb_data->run_mode);
    printf("  - Iq Ref:           %f A\n\r", fb_data->iq_ref);
    printf("  - Speed Ref:        %f rad/s\n\r", fb_data->spd_ref);
    printf("  - Position Ref:     %f rad\n\r", fb_data->loc_ref);
    printf("  - Limit Torque:     %f Nm\n\r", fb_data->limit_torque);
    printf("  - Limit Speed:      %f rad/s\n\r", fb_data->limit_spd);
    printf("  - Limit Current:    %f A\n\r", fb_data->limit_cur);
    printf("\n\r [PID Gains]\n\r");
    printf("  - Current Kp:       %f\n\r", fb_data->cur_kp);
    printf("  - Current Ki:       %f\n\r", fb_data->cur_ki);
    printf("  - Current Filt Gain:%f\n\r", fb_data->cur_filt_gain);
    printf("  - Position Kp:      %f\n\r", fb_data->loc_kp);
    printf("  - Speed Kp:         %f\n\r", fb_data->spd_kp);
    printf("  - Speed Ki:         %f\n\r", fb_data->spd_ki);
    printf("  - Speed Filt Gain:  %f\n\r", fb_data->spd_filt_gain);
    printf("\n\r [Profile Settings (PP Mode)]\n\r");
#ifndef USE_OLD_FIRMWARE_ADDRESSES
    // 新しいファームウェアのみに存在するパラメータ
    printf("  - Acceleration:     %f rad/s^2\n\r", fb_data->acc_rad);
#endif
    printf("  - Max Velocity:     %f rad/s\n\r", fb_data->vel_max_pp);
    printf("  - Acceleration Set: %f rad/s^2\n\r", fb_data->acc_set_pp);
    printf("\n\r [System Settings]\n\r");
    printf("  - Active Report Time:%u\n\r", fb_data->epscan_time);
    printf("  - CAN Timeout:      %lu\n\r", fb_data->can_timeout);
#ifndef USE_OLD_FIRMWARE_ADDRESSES
    // 新しいファームウェアのみに存在するパラメータ
    printf("  - Zero Flag (zero_sta): %u\n\r", fb_data->zero_sta);
    printf("  - Add Offset:           %f\n\r", fb_data->add_offset);
#endif
    printf("------------------------------------------\n\r\n\r");
}

void Robstride_Debug_Check_All_Parameters(Robstride_DeviceInfo *const device_info, DelayFunction_t f_delay) {
    Robstride_RequestAllParameters(device_info, f_delay);
    f_delay(100); // 全てのパラメータが更新されるまで少し待機
    Robstride_FeedbackData data = Read_Robstride_FeedbackData(device_info);
    Robstride_PrintAllParameters(&data);
}
