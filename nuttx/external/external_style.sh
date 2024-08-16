#!/bin/bash

if [ $# -lt 1 ] ; then
  echo -e "\t Usage: $0  file_name"
  exit 1
fi

# delete line which only contain space & tab
sed -i "s/^[ \t]*$//g" $1

# delete space & tab on the tail of line
sed -i "s/[ \t]*$//g" $1

exit 0
