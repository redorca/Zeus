#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import os
import sys
import re
import argparse
import collections
from typing import Optional, Match

# this program should be running under Python-3 environment
g_args=None

g_predefine_func_list=["up_assert"]

"""
 stack information data
 store function stack information
 key is function name
 value is stack size
"""
g_stack_info=dict()

"""
call graph data : use dictionary to store call graph
 key is the function name
 value is  list type for store sub-api
"""
g_call_graph=dict()

"""
key is the function name
value is dict which is : key is sub-api name , value is  the stack size
"""
g_final_result=dict()

def parameter_parsing():
    global g_args
    parser = argparse.ArgumentParser(description = 'Analyze the static stack size of function', \
                                   usage='%(prog)s [options] ')

    parser.add_argument("--directory", "-d", nargs='+', help = "The directory which will be searched for stack analysis")

    parser.add_argument("--function", "-f", help = "The name of function which will be  analyzed")

#    parser.add_argument("--max_size", "-m", default = "2048", type = int, help = "The filter of max stack size")

#    parser.add_argument("--output", "-o", help = "The output file name")

    parser.add_argument("--version", "-v", action='version', version='version 1.0')

    g_args = parser.parse_args()

    if None == g_args.directory:
        parser.print_help()
        exit(1)


def parse_call_graph(obj_file) -> object:
    """
    use arm-none-eabi-gcc to do disassemble and parsing
    the call stack
    :rtype: object
    :param obj_file: the object file name
    :return:
    """
    func_api_old_name = None
    func_api_name = None
    sub_func_list = list()
    func_start_re = re.compile("^([0-9a-fA-F]+) <(.*)>:")
    sub_func_start_re = re.compile("(.*): R_[A-Za-z0-9_]+_CALL[ \t]+(.*)")
    disassemble_file = os.popen("arm-none-eabi-objdump -dr "+obj_file)
    for line_context in disassemble_file:
        func_api = func_start_re.match(line_context)
        if None == func_api:
            sub_func_api = sub_func_start_re.match(line_context)
            if None != sub_func_api:
              #  print(sub_func_api.group(2)) # get sub-function name
                sub_func_list.append(sub_func_api.group(2))
        else:
            if None != func_api_old_name:
                # store previous function call graph
                if len(sub_func_list):
                    g_call_graph[func_api_old_name] = sub_func_list

            sub_func_list=list()
            func_api_old_name = func_api.group(2) # get the function name
    # store the last one func call graph
    if None != func_api_old_name and len(sub_func_list):
        g_call_graph[func_api_old_name] = sub_func_list
    disassemble_file.close()


def parse_line_context(str_context, su_file=None):
    """
    line-format:
    file_name:row:col:function_name  \t  stack_size  \t  type
    example: [fs_seekdir.c:59:20:seekpseudodir	32	static]
    :param str_context: line context of strinfg
    :return:
    """
    result_list = re.split(':', str_context)
    if len(result_list) < 3:
        return
    function_context = result_list[3]
    final_result_list = re.split('\t', function_context)
    #store result into g_stack_info
    g_stack_info[ final_result_list[0] ] = final_result_list[1]


def collect_function_stack_information(dir_list):
    """
    collect all function stack information in dir list
    :param dir_list: the directory list which will be analyzed
    :return:
    """
    g_func_stack=list()
    print("Enter function stack & call graph analysis")
    # first step to get out all .su files
    for dir in dir_list:
        for root, dirs, files in os.walk(dir):
            for file in files:
                if file.endswith(".su"):
                    g_func_stack.append(os.path.join(root, file))
    # second step to parse all .su files
    print("Total su files : %d" %len(g_func_stack))
    for file_su in g_func_stack:
        file_handle = open(file_su)
        file_name = os.path.basename(file_su).replace(".su", ".c")
        # parsing line context
        for line_context  in file_handle:
            if None == line_context:
              file_handle.close()
            else:
                parse_line_context(line_context, file_name)
        file_handle.close()

        # generate call graph
        obj_file = file_su.replace(".su", ".o")
        parse_call_graph(obj_file)
    # finally , remove predefine api from call - graph
    for predefine_api in g_predefine_func_list:
        if predefine_api in g_call_graph:
            g_call_graph.pop(predefine_api)



def get_api_pure_stack_size(func_name):
    """
    get the stack size of func_name and return
    :param func_name:
    :return:
    """
    if func_name in g_stack_info:
        return int(g_stack_info[func_name])

    #check if the api is constprop
    if "constprop" in func_name:
        constprop_handle=re.compile("(.*)\.[0-9]$")
        new_func_name = constprop_handle.match(func_name)
        new_func_name = new_func_name.group(1)
        if new_func_name in g_stack_info:
            return int(g_stack_info[new_func_name])
    # not found it
    return 0


# detect calling recursion list
g_recursion_caller_detec_list=list()

def parse_stack_size(func_name):
    """
    parsing the detail func_anme and sub-func stack size
    :param func_name:
    :return: int  : the stack size
    """
    # check if the call stack is recursion, and return itself stack size
    if func_name in g_recursion_caller_detec_list:
        g_recursion_caller_detec_list.append(func_name)
#        print("recursion is detect")
        return get_api_pure_stack_size(func_name)

    # added the func api into recursion detec list
    g_recursion_caller_detec_list.append(func_name)

    if func_name in g_call_graph:
        sub_api_list = g_call_graph[func_name]
  #      sub_api_list_result=list()
        sub_api_list_element_dict = dict()
        for sub_api in sub_api_list:
            stack_size = 0
            # check if the API is on final result
            if sub_api in g_final_result:
                sub_api_dict = g_final_result[sub_api]
                max_stack = 0
                for key in sub_api_dict:
                    if max_stack < sub_api_dict[key]:
                        max_stack = sub_api_dict[key]
                stack_size = max_stack + get_api_pure_stack_size(sub_api)
            elif sub_api in g_predefine_func_list: # if this is pre-define , just get pure stack size to avoid recursion
                stack_size = get_api_pure_stack_size(sub_api)
            else:
                stack_size = parse_stack_size(sub_api)
                g_recursion_caller_detec_list.pop()

            sub_api_list_element_dict[sub_api] = stack_size
   #         sub_api_list_result.append(sub_api_list_element_dict)

        sub_api_max_stack=0
        func_itself_stack = get_api_pure_stack_size(func_name)
        for dict_key in sub_api_list_element_dict:
            if sub_api_max_stack < sub_api_list_element_dict[dict_key]:
                sub_api_max_stack = sub_api_list_element_dict[dict_key]

        # store the func_name call_graph and stack size into g_final_result
        g_final_result[func_name] = sub_api_list_element_dict
        return sub_api_max_stack + func_itself_stack

    else:
        return get_api_pure_stack_size(func_name)


def generate_all_stack_output():
    print("Enter call generate_output")
    for key in g_call_graph:
        g_recursion_caller_detec_list=list()
        parse_stack_size(key)



if __name__=="__main__":
    parameter_parsing()

    print("System Max recursionlimit(): %d" %sys.getrecursionlimit())
    sys.setrecursionlimit(2000)
    print("Set system Max recursionlimit(): %d" %sys.getrecursionlimit())

    collect_function_stack_information(g_args.directory)

    print("Total API  : %d" %len(g_stack_info))
    #check if there is *.su files
    if 0 == len(g_stack_info):
        print("Please added -fstack-usage when compiling your source code!!!")
        exit(1)

    # store function stack info to file
    func_stack_handle = open("func_stack.txt", "w+")
    for key in g_stack_info:
        func_stack_handle.write(key+"\t"+g_stack_info[key]+"\n")
    func_stack_handle.close()

    # store call graph info to files
    try:
        os.mkdir("call_graph")
    except FileExistsError:
        print("call_graph Directory FileExist")

    for key in g_call_graph:
        file_name = os.path.join(os.getcwd(), "call_graph", key)
        caller_handle = open(file_name, "w+")
        sub_api_list = g_call_graph[key]

        for sub_api in sub_api_list:
            caller_handle.write(sub_api + "\n")
        caller_handle.close()

    # check if the function is in our project
    if None == g_args.function:
        print("Not special --function parameter")
        exit(2)
    #    generate_all_stack_output()
    elif g_args.function in g_stack_info:
        g_recursion_caller_detec_list=list()
        stack_size = parse_stack_size(g_args.function)
    else:
        print("< " + g_args.function + " > " + " is NOT in Project " + str(g_args.directory))
        exit(2)

    if None != g_args.function:
        print("%-32s    %8s    %8s" % ("Function", "Stack", "Frame"))
        print("=" * (32 + 4 + 8 + 4 + 8))
        print(">>%-30s    %8d    %8s" %(g_args.function, stack_size, g_stack_info[g_args.function]))

        if g_args.function in g_final_result:
            sub_api_dict_list=g_final_result[g_args.function]

            calling_frame_list=list()
            calling_frame_list.append(g_args.function)
            while True:
                sub_api_stack_max = 0
                sub_api_stack_max_key = None
                for key in sub_api_dict_list:
                    if sub_api_stack_max <= sub_api_dict_list[key]:
                        sub_api_stack_max_key = key
                        sub_api_stack_max = sub_api_dict_list[key]

                # print sub-api name and stack size
                if sub_api_stack_max_key not in calling_frame_list:
                    print("  %-30s    %8d    %8d" %(sub_api_stack_max_key, sub_api_stack_max, get_api_pure_stack_size(sub_api_stack_max_key)))
       #             calling_frame_list.append(sub_api_stack_max_key)
                else: # recursion detected
                    break

                if sub_api_stack_max_key in g_final_result:
                    sub_api_dict_list = g_final_result[sub_api_stack_max_key]
                else:
                    break

    print("\n")
    exit(0)

