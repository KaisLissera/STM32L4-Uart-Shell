/*
 * syscalls_minimal.c
 *
 *  Created on: 15 July 2023 г.
 *      Author: Kais Lissera
 */

void __attribute__((weak)) _exit(int i) {
	while(1);
}

void __attribute__((weak)) _write(void) {}
void __attribute__((weak)) _close(void) {}
void __attribute__((weak)) _read(void) {}
void __attribute__((weak)) _open(void) {}
void __attribute__((weak)) _kill(void) {}
void __attribute__((weak)) _getpid(void) {}
void __attribute__((weak)) _lseek(void) {}
