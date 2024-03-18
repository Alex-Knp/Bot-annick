#pragma once

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

void setup_uart();
void send_uart(float value);
float receive_uart();