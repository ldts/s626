#!/bin/bash

echo Unloading driver in case it was loaded before

./simple_unload_driver.sh

echo Loading xeno_analogy

modprobe xeno_analogy

echo Loading s626

insmod ./s626.ko


