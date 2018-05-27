#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'	# No Color

run_command() {
  echo TASK [${1}]
  error_msg=`${2} 2>&1`

  RESULT=$?
  if [ $RESULT -eq 0 ]; then
    echo -e "${GREEN}[OK]${NC}"
    echo
  else
    echo -e "${RED}[ERROR] => ${error_msg}${NC}"
    echo
    exit 1
  fi
}

