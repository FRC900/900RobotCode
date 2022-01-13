'''
Script to check for unused files, e.g. included from wpilib
to use
Edit zebROS_ws/src/CMakeOpt, comment out the first set(OPT_FLAGS) line, the one with -O3 in it
Install https://github.com/caolanm/callcatcher
In callcather, analyse.py, update the for call in methods: block to be 

        uncalled = []
        for call in methods:
                if not directcalls.has_key(call) and not call in virtualmethods:
                        uncalled.append(call)
                elif directcalls.has_key(call) and directcalls[call] == 3:
                        uncalled.append(call)
        uncalled.sort()
    
On a teminator command line, 
  export CC="callcatcher gcc"
  export CXX="callcatcher g++"
  export AR="callarchive ar"
clean.sh
Use natbuild to build the binary in question
Run this script. The 2 arguments are the root of the project output in build and the binard name from devel/.private
    python3 check_unused_files.py /home/ubuntu/2020Offseason/zebROS_ws/build/robot_characterization/CMakeFiles/robot_characterization.dir ~/2020Offseason/zebROS_ws/devel/.private/robot_characterization/lib/robot_characterization/robot_characterization
   
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

    
def read_file_symbols(file_name, referenced_symbols):

    ret = set()
    p = subprocess.Popen(['nm', '-C', file_name], stdout=subprocess.PIPE)

    for line in iter(p.stdout):
        fields = str(line.decode('ascii')).strip().split(' ', 2)
        if fields[1] in ['t', 'T', 'v', 'V']:
            if fields[2] not in referenced_symbols:
                continue
            if fields[2].startswith('typeinfo'):
                continue
            if fields[2].startswith('vtable'):
                continue
            if fields[2].startswith('operator new'):
                continue
            if fields[2].startswith('operator delete'):
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

            ret.add(fields[2])
    return ret

def remove_function(used_fns, function):
    for k in used_fns:
        if function in used_fns[k]:
            print("Found and removed " + str(function) + " from " +str(k))
            used_fns[k].remove(function)
    return used_fns

def main():

    referenced_symbols = []
    unreferenced_symbols = []
    unrefrenced_flag = False;
    with subprocess.Popen(['python2', '/usr/local/bin/callanalyse', '-s', '-d', sys.argv[2]], stdout=subprocess.PIPE) as p:
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
            else:
                unreferenced_symbols.append(line)

    used_fns = {}
    for obj in locate('*.o', sys.argv[1]):
        used_fns[obj] = read_file_symbols(obj, referenced_symbols)

    before_sizes = {}
    for k in used_fns:
        before_sizes[k] = len(used_fns[k])

    for s in unreferenced_symbols:
        used_fns = remove_function(used_fns, s)

    after_sizes = {}
    for k in used_fns:
        after_sizes[k] = len(used_fns[k])

    for k in before_sizes:
        print("%s %d %d" %(k, before_sizes[k], after_sizes[k]))


    for k in sorted(used_fns):
        print(k)
        for f in used_fns[k]:
            print("\t" + f)

    print("="*50)
    for k in after_sizes:
        if (after_sizes[k] == 0):
            print(k)



if __name__ == "__main__":
    main()
