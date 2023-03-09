#!/usr/bin/env python3
import serial
import rospy
from behavior_actions.msg import IntakeState2023

# meters
min_range = 0.05
max_range = 3

samples = 2
averages = [[],[]]

def uint16_to_dist(u16: int):
    if u16 == 65535:
        return float("inf")
    else:
        return u16 / 1000.0 # measured in millimeters

def parse_serial_msg(data: bytes):
    # functional programming is awesome
    if b"CRC error" in data:
        return None
    else:
        try:
            toReturn = tuple(map(lambda s: uint16_to_dist(int(s)), data.decode().split(",")))
            return toReturn
        except:
            return None

def safeAverage(l: list):
    total = 0
    for i in l:
        if i > max_range or i < min_range:
            return float("nan")
        total += i
    return total / len(l)

crc_error_count = 0
restart_errors_count = 10

def main():
    global samples, min_range, max_range, crc_error_count, restart_errors_count
    pub = rospy.Publisher("state", IntakeState2023, queue_size=1)
    rospy.init_node("intake_reader_2023", anonymous=True)

    port = serial.Serial(rospy.get_param("port"), rospy.get_param("baud"), timeout=1)#, dsrdtr=True, rtscts=True)
    samples = rospy.get_param("samples")
    min_range = rospy.get_param("min_range")
    max_range = rospy.get_param("max_range")
    rospy.loginfo(f"intake_reader_2023: using {port.port} @ {port.baudrate} baud. samples is {samples}. range is {min_range}-{max_range}m.")

    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        if crc_error_count >= restart_errors_count:
            # should restart board
            # not sure about dtr/rts, i think i put it in programming mode maybe?
            # leaving it out for now and hopefully a close/open will do the trick for a restart
            # port.dtr = True
            # port.rts = True
            port.close()
            port.open()
            rospy.loginfo("intake_reader_2023 : restarted RP2040")
            crc_error_count = 0
        port.flushInput()
        _ = port.readline() # if we flush the buffer between printing the first number and the second, we get "##\r\n"
        # so throw away the first line
        byteData = port.readline()
        parsed = parse_serial_msg(byteData)
        if parsed == None:
            crc_error_count += 1
            rospy.logwarn_throttle(0.1, "intake_reader_2023: CRC error")
            continue
        else:
            crc_error_count = 0
        for i in range(len(parsed)):
            averages[i].append(parsed[i])
            if len(averages[i]) > samples:
                averages[i] = averages[i][1:]
        averaged = list(map(safeAverage, averages))
        for a in averaged:
            if a != a: averaged = [a, a] # nan
        msg = IntakeState2023(leftDistance=averaged[0], rightDistance=averaged[1])
        pub.publish(msg)
        r.sleep()
    port.close()

main()