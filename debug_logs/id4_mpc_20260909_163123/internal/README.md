# F7内部記録の回収

Mode5、825mm/s・10A、約520→220mmの試験後にRAMに残っていた内部記録。
再駆動・リセットなし、ST-LINK HOTPLUGのread/uploadで回収。
Debug ELFから得たid4_diagアドレス0x20019ce0、1レコード32bytes。
id4_diag_count=339、id4_diag_active=falseを確認後に回収した。

targetはMPCの生の解そのものではなく、安全速度制約適用後にDOBへ渡した
実際の速度指令。modelはDOB内部の一次遅れ目標速度。
静止デッドバンドに入った後は記録されないので、その区間を補間で捏造しない。
ROSログのF7時刻・実測値と照合して同一試験であることを確認する。
