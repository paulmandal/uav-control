#!/bin/bash
gcc -O2 -Wall -D_GNU_SOURCE -c -o js_ctrl.o js_ctrl.c
gcc -O2 -Wall -c -o axbtnmap.o axbtnmap.c
gcc   js_ctrl.o axbtnmap.o   -lncurses -o js_ctrl
