# takes in a path to a csv that looks like the output of
'''
rostopic echo -p --bag=20240407_184110_NCCMP_E05.bag /apriltag_zedx_back/apriltag_detection/tags  | awk -F, '{print $1, $3}' > elims5.csv
'''
# recommended by tagslam author to determine how much time is elapsed between reciving the tag and publishing the detection
# csv looks like 
'''
%time field.header.stamp
1712515260959217780 1712515260721789000
1712515261191776661 1712515260888477000
1712515261342252395 1712515261088483000
1712515261492710754 1712515261221727000
1712515261634345898 1712515261388399000
1712515261750801821 1712515261555067000
'''

from statistics import mean 
import matplotlib.pyplot as plt
import time

# should be the faster bag running 971's tag detector (6fps)
with open("/home/ubuntu/.2023RobotCode.readonly/bagfiles/ncdcmp2024/trimmed/elims5.csv", "r") as f:
    data = f.read().splitlines()


# the actually faster bag running cameras at lower res, (20fps)
with open("/home/ubuntu/.2023RobotCode.readonly/bagfiles/2024ncash/trimmed/ASHelims7.csv", "r") as f:
    data = f.read().splitlines()


print(data)
data.pop(0)
times = []
for timestamp in data:
    timestamp.replace("\n", "")
    inital, after = timestamp.split(" ")
    delta = (int(inital)-int(after)) / 1e9
    print(f"Time delta = {delta}")
    if delta > 0.25:
        print(f"Delta is super high, {inital}, {after}")
        time.sleep(1)
        continue
    times.append(delta)



xs = [x for x in range(len(times))]
print(xs, times)
 

#exit()
plt.plot(xs, times)
# save the image to disk
plt.savefig('foo.png')

print(f"Average time {mean(times)}")
print(f"Max time {max(times)}")