#!/usr/bin/env bash
# GNU .suとELFから既知の深い送信経路の下限を検査。全経路の上限保証ではない。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
build_dir=${1:-build/Debug}
mapfile -t usage_files < <(rg --files --no-ignore "$build_dir" -g '*.su')
test "${#usage_files[@]}" -gt 0
frames=$(awk -F '\t' '
BEGIN {
  split("StartRobstrideTask MicroRos_RefreshRobstrideTargets Robstride_SetTargetIfGeneration robstride_set_target_internal robstride_send_current Robstride_WriteFloatData Robstride_SendBytes _Robstride_SendBytesToQueue _Robstride_PopSendTx8Bytes HAL_CAN_AddTxMessage", names, " ")
  for(i in names) wanted[names[i]]=1
}
{n=split($1,a,":"); name=a[n]; if(name in wanted) {frame[name]=$2; found[name]=1}}
END {
  for(i in wanted) {if(!(i in found)) {print "missing frame: " i > "/dev/stderr"; exit 1}; sum+=frame[i]}
  print sum
}' "${usage_files[@]}")
stack_hex=$(arm-none-eabi-nm -S "$build_dir/26catch_f7.elf" | awk '$4=="RobstrideTaskBuffer" {print $2}')
test -n "$stack_hex"
stack_bytes=$((16#$stack_hex))
required=$((frames+512))
printf 'Robstride known call-chain frames=%s B, reserve=512 B, stack=%s B\n' "$frames" "$stack_bytes"
if ((stack_bytes<required)); then
  printf 'FAIL: stack below known call-chain requirement %s B\n' "$required" >&2
  exit 1
fi
# CubeMX再生成で旧サイズへ戻らないことも確認する。
rg -q 'RobstrideTask,1,2048,' 26catch_f7.ioc
rg -q 'RobomasTask,1,1024,' 26catch_f7.ioc
printf 'PASS: known call-chain budget and CubeMX stack configuration\n'
