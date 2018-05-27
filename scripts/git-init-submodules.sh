#!/bin/bash

source $(dirname "$0")/functions.sh

run_command 'git : Initialize git submodules' 'git submodule init'
run_command 'git : Update git submodules' 'git submodule update'

