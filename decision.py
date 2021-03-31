#!/usr/bin/env python
from __future__ import division,print_function
import numpy as np
from collections import deque

class Decision:
    actions = ["forward","backward","turn_ccw","turn_cw","turn_ccw_half","turn_cw_half","fly_over"]
    threshold = {"forward":220,"fly_over":100}
    def __init__(self):

        self.de= deque(100*[""],100)

    def make_decision(self,hline):
        ###################################
        # priority of checking
        # fly_over----> turn----> forward
        ###################################
        self.hline = hline
        self.n = len(hline)
        if self.n == 3:
            if self.hline[0]>=self.threshold["forward"]:
                if self.hline[-2]>=self.threshold["fly_over"]:
                    self.de.append("fly_over")
                    return "fly_over"
                else:
                    self.de.append("turn_ccw")
                    return "turn_ccw"
            else:
                self.de.append("forward")
                return "forward"

        elif self.n==1 or self.n==2:
            if self.hline[0]>=self.threshold["forward"]:
                self.de.append("turn_ccw")
                return "turn_ccw"
            else:
                self.de.append("forward")
                return "forward"
        else:
            return "ERROR"

    def _memory(self):
        return self.de


        

if __name__ == "__main__":
    a = Decision()
    print(a.make_decision([300,1,400]))
    print(a._memory()[-1])
    print(a.make_decision([300,1,1]))
    print(a._memory()[-2])
    print(a._memory())
    
