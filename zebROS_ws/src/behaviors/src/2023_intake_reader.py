#!/usr/bin/env python3
import serial
import rospy
from behavior_actions.msg import IntakeState2023
from std_srvs.srv import EmptyResponse, Empty, EmptyRequest

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
            # likely b""
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

def rebootPico(port: serial.Serial, req = None):
    rospy.logwarn("intake_reader_2023 : rebooting Pico!")
    port.write(b"r\n")
    rospy.logwarn("intake_reader_2023 : rebooted Pico, waiting 2.5 seconds")
    rospy.sleep(2.5)
    rospy.logwarn("intake_reader_2023 : finished waiting, closing and reopening port")
    port.close()
    port.open()
    return EmptyResponse()

def main():
    global samples, min_range, max_range, crc_error_count, restart_errors_count
    pub = rospy.Publisher("state", IntakeState2023, queue_size=1)
    rospy.init_node("intake_reader_2023", anonymous=True)

    port = serial.Serial(rospy.get_param("port"), rospy.get_param("baud"), timeout=1)
    samples = rospy.get_param("samples")
    min_range = rospy.get_param("min_range")
    max_range = rospy.get_param("max_range")
    flip_terabees = rospy.get_param("flip_terabees")
    rospy.loginfo(f"intake_reader_2023: using {port.port} @ {port.baudrate} baud. samples is {samples}. range is {min_range}-{max_range}m.")

    srv = rospy.Service("reboot_pico", Empty, lambda req: rebootPico(port, req))

    r = rospy.Rate(50)

    reboot_count = 0
    max_reboots = 2

    rebootPico(port, None)

    while not rospy.is_shutdown():
        if crc_error_count >= restart_errors_count and reboot_count < max_reboots:
            # should restart board
            rospy.loginfo("intake_reader_2023 : restarting RP2040")
            rebootPico(port)
            crc_error_count = 0
            reboot_count += 1
            continue
        try:
            port.flushInput()
            _ = port.readline() # if we flush the buffer between printing the first number and the second, we get "##\r\n"
            # so throw away the first line
            # why were we flushing the input?
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
            if flip_terabees: msg = IntakeState2023(leftDistance=averaged[1], rightDistance=averaged[0])
            else: msg = IntakeState2023(leftDistance=averaged[0], rightDistance=averaged[1])
            pub.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(0.1, f"intake_reader_2023 : couldn't read from pico, error = {e}")
            r.sleep()
    port.close()

main()