#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )
export NP2_ROOT=$DIR
export NP2_QEMU=$HOME/qemu/arm-softmmu
export PATH=$NP2_ROOT/tools:$NP2_QEMU:$PATH
