#ifndef UAV_PROTO_H
#define UAV_PROTO_H

#include "event.h"

CommandType proto_parse_command(const char *raw);
const char *proto_command_name(CommandType cmd);

#endif
