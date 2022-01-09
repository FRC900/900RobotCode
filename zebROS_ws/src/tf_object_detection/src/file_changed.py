#!/usr/bin/env python
import hashlib
import os




def file_changed(f):
    sum_path = f.replace(".", "")
    sum_path = ".md5sum" + str(f) + ".txt"
    if not os.path.exists(f):
        return True
    if not os.path.exists(sum_path):
        with open(f, "rb") as file:
            old_sum = hashlib.md5(file.read()).hexdigest()
        with open(sum_path, "w") as file:
            file.write(old_sum)
        return False

    else:
        with open(sum_path, 'r') as sum_file:
            old_sum = sum_file.read()
    
    sum = hashlib.md5()
    with open(f, 'rb') as in_file:
        for chunk in iter(lambda: in_file.read(4096), b''):
            sum.update(chunk)
    text_sum = sum.hexdigest()
    if text_sum == old_sum:
        return False
    with open(sum_path, 'w') as sum_out:
        sum_out.write(text_sum);
        return True

    
