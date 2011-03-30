#!/bin/bash
gcc -g -O2 -Wall -I../linux/include   -c -o js_ctrl.o js_ctrl.c
gcc -g -O2 -Wall -I../linux/include   -c -o axbtnmap.o axbtnmap.c
gcc   js_ctrl.o axbtnmap.o   -o js_ctrl
