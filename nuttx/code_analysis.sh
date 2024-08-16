#!/bin/bash
############################################################################
#   code_analysis.sh
#
#   Copyright (C) 2018 Zglue INC. All rights reserved.
#   Author: Levin Li <zhiqiang@zglue.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################


#parsing the start & end var from map
# $1 is the start symbol
# $2 is the end   symbol
# return: 0 for success , other for fail
# if success the module_start_num & module_end_num will record the data

function parsing_section_info()
{
  if [ $# -ne 2 ] ; then
    return 1
  fi

  string_start_result=`grep $1 $symbol_file`

  if [ "$string_start_result" == "" ]; then
    return 2
  fi

  string_end_result=`grep $2 $symbol_file`

  if [ "$string_end_result" == "" ]; then
    return 3
  fi

  module_start_num=`echo $string_start_result | cut -d ' ' -f 1`
  module_end_num=`echo $string_end_result | cut -d ' ' -f 1`

  if echo "$module_start_num" | grep "!provide" ; then
    return 4; # not provide such symbol
  fi

  # added 0x prefix for hex data
  if echo "$module_start_num" | grep "0x" ; then
    module_start_num=$module_start_num
    module_end_num=$module_end_num
  else
    module_start_num=0x$module_start_num
    module_end_num=0x$module_end_num
  fi

#  echo "Start: $module_start_num , End: $module_end_num"

  return 0
}


# MAIN Loop

if [ $# -eq 0 ]; then
  echo "Usage: $0  map_file"
  exit 1
fi

symbol_file=$1

if [ ! -e "$symbol_file" ]; then
  echo "Can't open the file ($symbol_file), Please check!!!"
  exit 1
fi

# var used to search symbol from map
module_start_var=
module_end_var=

prefix_text_start=_start_
prefix_text_end=_end_

prefix_data_start=_start_
prefix_data_end=_end_
suffix_data=_data

prefix_bss_start=_start_
prefix_bss_end=_end_
suffix_bss=_bss

total_text_size=0
total_data_size=0
total_bss_size=0
# num used to record section start & end address
module_start_num=
module_end_num=

# module for search , it should be define from link script
module_array=(sched mm c_cxx fs drivers soc apps others)

for element in ${module_array[@]};
do
  # echo $element
  # text section parsing
  module_start_var=${prefix_text_start}${element}
  module_end_var=${prefix_text_end}${element}
  parsing_section_info $module_start_var $module_end_var
  ret_value=$?

  if [ $ret_value -eq 0 ] ; then
    let section_size=module_end_num-module_start_num
    let total_text_size=total_text_size+section_size
#     section_size=`echo $((module_end_num-module_start_num))`
#     total_text_size=`echo $((total_text_size+section_size))`

    echo "Module [$element] , [TEXT] Info: Start: $module_start_num, End: $module_end_num, Size: $section_size"
  else
    echo "Module [$element] , [TEXT] paring Fail: $ret_value"
  fi

  # data section parsing
  module_start_var=${prefix_data_start}${element}${suffix_data}
  module_end_var=${prefix_data_end}${element}${suffix_data}
  parsing_section_info $module_start_var $module_end_var
  ret_value=$?

  if [ $ret_value -eq 0 ] ; then
    let section_size=module_end_num-module_start_num
    let total_data_size=total_data_size+section_size
    echo "Module [$element] , [DATA] Info: Start: $module_start_num, End: $module_end_num, Size: $section_size"
  else
    echo "Module [$element] , [DATA] paring Fail: $ret_value"
  fi

  # bss section parsing
  module_start_var=${prefix_bss_start}${element}${suffix_bss}
  module_end_var=${prefix_bss_end}${element}${suffix_bss}
  parsing_section_info $module_start_var $module_end_var
  ret_value=$?

  if [ $ret_value -eq 0 ] ; then
    let section_size=module_end_num-module_start_num
    let total_bss_size=total_bss_size+section_size
    echo "Module [$element] , [BSS] Info: Start: $module_start_num, End: $module_end_num, Size: $section_size"
  else
    echo "Module [$element] , [BSS] paring Fail: $ret_value"
  fi

  echo ""

done

echo "Total Text: $total_text_size"
echo "Total Data: $total_data_size"
echo "Total Bss : $total_bss_size"

exit 0

