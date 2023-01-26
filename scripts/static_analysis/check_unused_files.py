'''
Script to check for unused files, e.g. included from wpilib
to use
Edit zebROS_ws/src/cmake_modules/CMakeOpt.cmake :
 - comment out the first set(OPT_FLAGS) line, the one with -O3 in it
 - comment out the gcc-ar and gcc-ranlib sets
 - add 'set (CMAKE_AR     "/usr/local/bin/callarchive")
Install https://github.com/caolanm/callcatcher

sudo gvim `which callarchive`, change the last line to:
	os.execvp('ar', sys.argv)

This last one is because I can't find a way to make CMAKE's ar program be multiple words.
Ideally it would just be something like 'set (CMAKE_AR     "/usr/local/bin/callarchive ar"), but that
tries to run a program called /usr/local/bin/callarchive\ ar rather than callarchive with ar as the first argument.
sudo vim /usr/local/lib/python3.6/dist-packages/callcatcher/__init__.py.
    Edit archive() function loop, change len(arg) == 1 to len(arg) <= 3.
    Without this, the code thinks that the 'qc' from 'ar qc output.a input.o' is the output file name
    
In callcather, analyse.py, update the for call in methods: block to be 

        uncalled = []
        for call in methods:
                if call not in directcalls and call not in virtualmethods:
                        uncalled.append(call)
                elif call in directcalls and directcalls[call] == 3:
                        uncalled.append(call)
        uncalled.sort()
    
On a teminator command line, 
  export CC="callcatcher gcc"
  export CXX="callcatcher g++"
  export AR="callarchive ar"
clean.sh
Use natbuild to build the binary in question
Run this script. The 2 arguments are the root of the project output in build and the binary name from devel/.private
    python3 check_unused_files.py /home/ubuntu/2023RobotCode/zebROS_ws/build/robot_characterization/CMakeFiles/robot_characterization.dir ~/2023RobotCode/zebROS_ws/devel/.private/robot_characterization/lib/robot_characterization/robot_characterization
   
Output includes some debugging, then a list of the before/after count of used functions in each object file
Those with 0 used functions after processing will be listed after a ===== line at the bottom of the output.
These can be safely removed from the list of .cpp files
'''

#!/usr/bin/python3

import fnmatch
import os
import subprocess
import sys

def locate(pattern, root=os.curdir):
    '''
    Locate all files matching supplied filename pattern in and below
    supplied root directory.
    '''
    for path, dirs, files in os.walk(os.path.abspath(root)):
        for filename in fnmatch.filter(files, pattern):
            yield os.path.join(path, filename)
            
def remove_stdlib_functions(namespace, functionname):
    cpp_types = ['void', 'int', 'short', 'char', 'wchar_t', 'double', 'float', 'long', 'bool', 'decltype(auto)']
    cpp_signed = ['', 'signed ', 'unsigned ']
    cpp_ptr_ref = ['', '&', '&&', '*', '**']
    cpp_const = ['', ' const']

    if functionname.startswith(namespace):
        return True
    for t in cpp_types:
        for s in cpp_signed:
            for pr in cpp_ptr_ref:
                for c in cpp_const:
                    type_str = s + t + c + pr
                    if functionname.startswith(type_str + ' ' + namespace):
                        return True
    return False

    
def read_file_symbols(file_name, referenced_symbols, used, unused):

    p = subprocess.Popen(['nm', '-C', file_name], stdout=subprocess.PIPE)

    print(f"Filename = {file_name}")
    for line in iter(p.stdout):
        l = (line.decode('ascii')).strip()
        if l.endswith('.cpp.o'):
            file_name = l
            continue
        if file_name not in used:
            used[file_name] = set()
            unused[file_name] = set()
        fields = l.split(' ', 2)
        #print(f"line = {line.decode('ascii')}")
        if len(fields) < 3:
            continue
        if fields[1] in ['t', 'T', 'v', 'V']:
            if fields[2].startswith('typeinfo'):
                continue
            if fields[2].startswith('vtable'):
                continue
            if fields[2].startswith('operator new'):
                continue
            if fields[2].startswith('operator delete'):
                continue
            if fields[2].startswith('VTT for'):
                continue
            if fields[2].startswith('_GLOBAL__sub_I_'):
                continue
            if remove_stdlib_functions("std::", fields[2]):
                continue
            if remove_stdlib_functions("boost::", fields[2]):
                continue
            if remove_stdlib_functions("__gnu_cxx::", fields[2]):
                continue
            if fields[2] == 'void (&std::forward<void (&)()>(std::remove_reference<void (&)()>::type&))()':
                continue
            if (fields[1] == 't') and ('__gthread' in fields[2]):
                continue
            if fields[2] == 'DW.ref.__gxx_personality_v0':
                continue

            if fields[2] in referenced_symbols:
                used[file_name].add(fields[2])
            else:
                unused[file_name].add(fields[2])
    #print(f"Read from {file_name} : \n\tUsed = {sorted(used)}\n\tUnused = {sorted(unused)}")
    return used, unused

def remove_function(used_fns, function):
    for k in used_fns:
        if function in used_fns[k]:
            print("Found and removed " + str(function) + " from " + str(k))
            used_fns[k].remove(function)
    return used_fns

def main():

    referenced_symbols = ['main']
    unreferenced_symbols = []
    unrefrenced_flag = False
    print(str(sys.argv))
    with subprocess.Popen(['python3', '/usr/local/bin/callanalyse', '-s', '-d', sys.argv[2]], stdout=subprocess.PIPE) as p:
        for line in iter(p.stdout.readline, b''):
            line = str(line.decode('ascii')).strip()
            print(line)
            if not unrefrenced_flag:
                if line.startswith('0x'):
                    continue
                if line == '---Unreferenced symbols---':
                    unrefrenced_flag = True
                    continue
                if line.endswith(' is directly called'):
                    referenced_symbols.append(line[:-len(' is directly called')])
                if line.endswith(' is used in data'):
                    referenced_symbols.append(line[:-len(' is used in data')])
                if line.endswith(' has its address taken'):
                    referenced_symbols.append(line[:-len(' has its address taken')])
            else:
                if line != 'main':
                    unreferenced_symbols.append(line)

    used_fns = {}
    unused_fns = {}
    for obj in sorted(locate('*.o', sys.argv[1])):
        read_file_symbols(obj, referenced_symbols, used_fns, unused_fns)

    # Read .a files.
    # Note - currently doesn't give correct results. It looks like
    # it doesn't handle cases where functions in a library are also
    # used in that library?  Not sure, needs to be debugged.
    for i in range(3, len(sys.argv)):
        read_file_symbols(sys.argv[i], referenced_symbols, used_fns, unused_fns)

    before_sizes = {}
    for k in used_fns:
        before_sizes[k] = len(used_fns[k])

    #for s in unreferenced_symbols:
        #used_fns = remove_function(used_fns, s)

    after_sizes = {}
    for k in used_fns:
        after_sizes[k] = len(used_fns[k])

    for k in before_sizes:
        print("%s %d %d" %(k, before_sizes[k], after_sizes[k]))

    for k in sorted(unused_fns):
        print(k)
        for f in sorted(unused_fns[k]):
            print(f"\t{f}")

    print("="*50)
    for k in after_sizes:
        if (after_sizes[k] == 0):
            print(k)


if __name__ == "__main__":
    main()
