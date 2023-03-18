#!/usr/bin/env python
import time


class fake_robot:
    def __init__(self):
        self.start = None
        self.busy = False
        self.start = 0

    def start_grasp(self, pos):
        self.busy = True
        self.start = time.time()
        print('grasp at', pos)

    def is_busy(self):
        self.busy = not ((time.time() - self.start) > 1)
        print((time.time() - self.start))
        return self.busy
