#!/usr/bin/env bash
# 実機・ROS・ST-LINKにアクセスしないホスト回帰試験。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
test_dir=$(mktemp -d /tmp/catch2026-host-tests.XXXXXX)
cc=${CC:-gcc}
robomas=canlib_robomaster/CAN_RoboMas
arm=canlib_robstride/CAN_Robstride/Control
includes=(-Itools/c620_test_stubs -ICore/Inc -Icanlib_robomaster
  "-I$robomas" "-I$robomas/Control"
  -Icanlib_robomaster/CAN_RoboMas_Def -Icanlib_robomaster/CAN_RoboMas_System)
sources=("$robomas/CAN_RoboMas.c" "$robomas/RoboMas_Control.c" "$robomas"/Control/*.c)
for name in robomas_integration c620_commission; do
  "$cc" -std=c11 -O2 -DINTEGRATED_STARTUP_HOLD=0 "${includes[@]}" "tools/test_$name.c" "${sources[@]}" -lm -o "$test_dir/$name"
done
"$test_dir/robomas_integration"
"$cc" -std=c11 -O2 "${includes[@]}" tools/test_robomas_integration.c "${sources[@]}" -lm -o "$test_dir/startup_hold"
"$test_dir/startup_hold"
"$cc" -std=c11 -O2 "${includes[@]}" tools/test_c620_commission.c "${sources[@]}" -lm -o "$test_dir/pressed_boot_hold"
"$test_dir/pressed_boot_hold" boot_pressed
"$test_dir/pressed_boot_hold" timeout
for scenario in boot boot_pressed boot_cancel normal cancel calibration_fast_feedback calibration_wrong_mode overspeed no_stall stale timeout timeout_moving timeout_stale timeout_nan timeout_cancel survey5 survey_speed survey_stale survey_timeout restore mpc_zero slow_calibration pulse400 pulse800 pulse_stale pulse_cancel pulse_bounds pulse_time pulse_timeout; do
  "$test_dir/c620_commission" "$scenario"
  printf 'PASS: C620 %s\n' "$scenario"
done
run_test() {
  local name=$1
  shift
  local control_include="$robomas/Control"
  if [[ $name == arm_* ]]; then control_include=$arm; fi
  "$cc" -std=c11 -O2 -ICore/Inc "-I$robomas" "-I$control_include" "tools/test_$name.c" "$@" -lm -o "$test_dir/$name"
  "$test_dir/$name"
}
run_test c620_position_mpc "$robomas/Control/PositionMpc.c" "$robomas/Control/C620PositionMpc.c"
run_test id4_position_mpc "$robomas/Control/PositionMpc.c"
run_test id4_velocity_safety
run_test control_period
run_test command_auto_enable
run_test arm_position_mpc "$arm/ArmPositionMpc.c"
run_test arm_position_tracking "$arm/ArmPositionMpc.c"
run_test arm_feedback_guard
run_test arm_velocity_estimate
run_test arm_velocity_dob "$arm/Actuator_VelocityDob.c"
printf 'All host tests passed. Binaries: %s\n' "$test_dir"
