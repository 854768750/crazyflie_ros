#!/usr/bin/env python
from multi_circle_1 import Multi_circle_1

if __name__ == '__main__':
    multi_circle_1 = Multi_circle_1(
        [
            #x   ,   y,   z, yaw, sleep
            [0.0  ,  0.0, 1.0, 0, 8],
	    [0.0  ,  0.0 ,1.0, 0, 3],
  	    [-0.3 , -1.4, 0.0, 0, 0],
        ]
    )
    multi_circle_1.run()
