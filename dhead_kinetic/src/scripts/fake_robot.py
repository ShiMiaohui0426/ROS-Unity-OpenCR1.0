#!/usr/bin/env python
import time

start = time.time()


class fake_robot:
    def __init__(self):
        self.start = None
        self.busy = False

    def start_grasp(self,pos):
        self.start = time.time()
        self.busy = True
        print('grasp at',pos)

    def is_busy(self):
        if (time.time() - 5) > start:
            self.busy = False
