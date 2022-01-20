/*
 * Copyright 2021 Chair of EDA, Technical University of Munich
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	 http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file helloworld.c
/// @date 2022-01-12
/// @brief target software test file "hello world" on iss I/O
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdio.h"

#define PRINT_CHAR(LOGGER_ADDR, VAL) *(volatile char *)(LOGGER_ADDR) = (VAL)

void custom_print_string(volatile char *logger_addr, const char *string)
{
    char *str = (char *)string;
    while (*str)
    {
        PRINT_CHAR(logger_addr, *str);
        str++;
    }
}

int main()
{
    custom_print_string((char *)ETISSVP_LOGGER, "Hello World! From ETISS Logger Plugin!\n");
    printf("hello world from printf!\n");
    return 0;
}
