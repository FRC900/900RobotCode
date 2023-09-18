"""
Module to do basic timing of various events in code

"""

import cupy

class CudaEventTiming(object):
    __total_time = 0.0
    __count = -1 # Skip first frame to avoid dll load overhead
    __name = None

    def __init__(self, name, stream):
        self.__name = name
        self.__stream = stream
        self.__start_event = cupy.cuda.runtime.eventCreate()
        self.__end_event = cupy.cuda.runtime.eventCreate()
        self.__start_event_seen = False
        self.__end_event_seen = False

    def end_frame(self):
        if not self.__start_event_seen:
            return
        if not self.__end_event_seen:
            print(f"Missing end event for {self.__name}")
            return
        cupy.cuda.runtime.eventSynchronize(self.__start_event)
        cupy.cuda.runtime.eventSynchronize(self.__end_event)
        self.__start_event_seen = False
        self.__end_event_seen = False
        elapsed_time = cupy.cuda.runtime.eventElapsedTime(self.__start_event, self.__end_event) / 1000.
        self.__count += 1
        if self.__count > 0:
            self.__total_time += elapsed_time
                
    def start(self):
        if self.__start_event_seen:
            print(f"Error : duplicate start event {self.__name} seen")

        cupy.cuda.runtime.eventRecord(self.__start_event, self.__stream)
        cupy.cuda.nvtx.RangePush(self.__name)
        self.__start_event_seen = True

    def end(self):
        if self.__end_event_seen:
            print(f"Error : duplicate end event {self.__name} seen")
        cupy.cuda.runtime.eventRecord(self.__end_event, self.__stream)
        cupy.cuda.nvtx.RangePop(self.__name)
        self.__end_event_seen = True

    def __str__(self):
        if self.__count <= 0:
            return self.__name + " : no events recorded"
        return f"{self.__name} : {self.__count} events. Total time = {self.__total_time}, average time = {self.__total_time / self.__count}"


class CudaEventTimings(object):
    __timings = {}

    def __init__(self, stream):
        print(f"stream = {stream}")
        self.__stream = stream.cuda_stream

    def __del__(self):
        self.end_frame()
        print(str(self))

    def start(self, name):
        if name not in self.__timings:
            self.__timings[name] = CudaEventTiming(name, self.__stream)
        self.__timings[name].start()

    def end(self, name):
        if name not in self.__timings:
            print("Error - end called before start for " + name)
            return
        self.__timings[name].end()

    def end_frame(self):
        for t in self.__timings.values():
            t.end_frame()

    def __str__(self):
        s = ""
        for t in self.__timings.values():
            s += str(t)
            s += '\n'
        return s


