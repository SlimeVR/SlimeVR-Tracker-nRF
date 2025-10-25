/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "parse_args.h"

#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <string.h>
#include <zephyr/sys/printk.h>

// Parse the command line to find its arguments.
// Original string is modified to terminate the arguments substrings.
// Returns the number of found arguments or zero.
size_t parse_args(char *str, char *argv[], size_t size)
{
	size_t argc = 0;
	if (!strlen(str)) {
		return 0;
	}
	while (*str && *str == ' ') {
		str++;
	}
	if (!*str) {
		return 0;
	}
	argv[argc++] = str;
	while ((str = strchr(str, ' '))) {
		*str++ = '\0';
		while (*str && *str == ' ')
			str++;
		if (!*str) break;
		argv[argc++] = str;
		if (argc == size) {
			printk("Too many parameters (max %u)\n", size);
			return 0;
		}
	}
	// keep it POSIX style where argv[argc] is required to be NULL
	argv[argc] = NULL;
	return argc;
}

int32_t parse_i32(const char *str, uint8_t base)
{
 	long long res = strtoll(str, NULL, base);
	if(res < INT32_MIN) {
		res = INT32_MIN;
		errno = ERANGE;
	}
	if(res > INT32_MAX) {
		res = INT32_MAX;
		errno = ERANGE;
	}
	return (int32_t) res;
}

uint32_t parse_u32(const char *str, uint8_t base)
{
	unsigned long long res = strtoull(str, NULL, base);
	if(res > UINT32_MAX) {
		res = UINT32_MAX;
		errno = ERANGE;
	}
	return (uint32_t) res;
}

uint64_t parse_u64(const char *str, uint8_t base)
{
	return strtoull(str, NULL, base);
}
