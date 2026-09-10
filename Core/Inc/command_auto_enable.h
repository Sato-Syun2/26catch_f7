#ifndef COMMAND_AUTO_ENABLE_H
#define COMMAND_AUTO_ENABLE_H
#include <stdbool.h>

/* 新着・有効・世代一致の指令にのみ適用する。位置モードは停止後も復帰。 */
static inline bool CommandAutoEnable_Required(bool first_command,
                                             bool position_mode, bool enabled)
{
    return !enabled && (first_command || position_mode);
}
#endif
