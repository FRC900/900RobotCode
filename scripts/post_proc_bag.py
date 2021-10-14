# Script to process bag files from matches
# The script parses match data to figure out enable/diable times
# in the bag file. If there is valid match data for a given enabled
# period, extraact the data from that time span into a separate bag
# file.  
# This automates the process of renaming generic bag files into ones
# with informative names, including event name, match number and time/date
import rospy
import rosbag
import datetime
from rospy_message_converter import message_converter
import sys
import os
import errno    

def matchType(i):
    matchTypes = {
        0: "N", # None
        1: "P", # Practice
        2: "Q", # Qualification
        3: "E"  # Elimination
    }
    return matchTypes.get(int(i), "U") # Default to 'U'nknown for out of range requests?

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >=2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

#Bagfile name from python script, assuming bagfile is in ../bagfiles
bagfile_name = sys.argv[1]

# Rename .bag.active to .bag, strip leading _ if present
filename, extension = os.path.splitext(bagfile_name)
if filename[0] == '_':
    filename = filename[1:]

if (extension == '.active') or (bagfile_name[0] == '_'):
    if os.path.isfile(filename):
        print 'Both ' + bagfile_name + ' and ' + filename + ' exist, exiting instead of overwiting on rename'
        sys.exit(-1)

    os.rename(bagfile_name, filename)
    bagfile_name = filename
    
# Open bagfile, reindex if necessary
try: 
    bag = rosbag.Bag(bagfile_name)
except rosbag.bag.ROSBagUnindexedException:
    print "Reindexing bag file " + str(bagfile_name)
    os.system("rosbag reindex " + str(bagfile_name))
    bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'

isEnabled = False
enable_times = []
disable_times = []
match_numbers = []
match_types = []
replay_numbers = []
event_names = []
first_time = -1
this_time = -1

#Reads through the bagfile until it sees that the robot is enabled then breaks.
#That means that the last message in each list is the first message since robot enable
for topic, msg, t in bag.read_messages([match_topic]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    # Save the first timestamp in the bag file
    if (first_time == -1):
        first_time = this_time

    if(m['Enabled'] == True and isEnabled == False):
        isEnabled = True
        # There's a short disabled period between auto and teleop
        # Combine that into one enabled period
        if (len(disable_times) > 0):
            elapsed_time = disable_times[-1] - enable_times[-1]
            if (elapsed_time > 13) and (elapsed_time < 17) and ((this_time - disable_times[-1]) < 3) and (int(match_types[-1]) > 0):
                disable_times.pop()
                print('Combining auto and teleop')
                continue

        enable_times.append(this_time)
        match_numbers.append(str(m['matchNumber']))
        if match_numbers[-1] == 0:
            replay_numbers.append(0)
            event_names.append('')
            match_types.append('')
        else:
            match_types.append(str(m['matchType']))
            replay_numbers.append(str(m['replayNumber']))
            event_names.append(str(m['eventName']))
            print("Enable at " + str(enable_times[-1]) + " " + str(match_numbers[-1]) + " " + str(match_types[-1]) + " " + str(replay_numbers[-1]) + " " + str(event_names[-1]))

    if(isEnabled == True and m['Enabled']==False):
        isEnabled = False
        disable_times.append(this_time)
        elapsed_time = disable_times[-1] - enable_times[-1]
        print("Disable at " + str(disable_times[-1]) + " elapsed = " + str(elapsed_time))

# If we're still enabled at the end of the bag file, tag the last
# message time as the end of the bag file to copy
# This should probably never happen for an actual match bag
# file but who knows
if len(enable_times) != len(disable_times):
    disable_times.append(this_time)

mkdir_p('trimmed')
mkdir_p('done')
for i in range(len(enable_times)):
    # Any one should indicate an enable when the robot was not
    # connected to the field, but check all to be sure
    if int(match_types[i]) == 0:
       continue
    if int(match_numbers[i]) == 0:
       continue
    if int(replay_numbers[i]) == 0:
        continue
    if len(event_names[i]) == 0:
        continue

    date_str = datetime.datetime.utcfromtimestamp(enable_times[i]).strftime("%Y%m%d_%H%M%S");

    # Replay 1 is the normal case of a match not being replayed - don't include that in the filename 
    replay_str = ""
    if (int(replay_numbers[i]) > 1):
        replay_str = "R"+replay_numbers[i]

    # Pad match number to two characters so sorting by filename puts e.g. match 2 before match 10
    if (len(match_numbers[i]) < 2):
        match_numbers[i] = '0' + match_numbers[i]
    file_name = date_str + "_" + event_names[i] + "_" + matchType(match_types[i]) + match_numbers[i] + replay_str + ".bag"
    print(file_name)
    print('Enable at  ' + str(enable_times[i]) + " (or " + str(enable_times[i]-first_time) + "s)") 
    print('Disable at ' + str(disable_times[i]) + " (or " + str(disable_times[i]-first_time) + "s)")
    print("Elapsed time = " + str(disable_times[i] - enable_times[i]))

    if os.path.isfile(os.path.join('trimmed', file_name)):
        print "Output file exists - exiting instead of overwriting"
        sys.exit()

    print("Time of Enable: " + str(enable_times[i]) + " (or " + str(enable_times[i]-first_time) + "s)")
    print("Match Number: " + str(match_numbers[i]))
    print("Event Name: " + str(event_names[i]))

    # Extract match plus 10 seconds padding at start and end to separate bag file
    sys_call = 'rosbag filter ' + bagfile_name + ' ' + os.path.join('trimmed', file_name) + ' \"t.to_sec() >= ' + str(enable_times[i] - 10.0) + ' and t.to_sec() <= ' + str(disable_times[i] + 10.0) + '\"'
    print (sys_call)
    os.system(sys_call)

if len(enable_times) == 0:
    print('Note - robot not enabled in bag data')

os.rename(bagfile_name, os.path.join('done', bagfile_name))
