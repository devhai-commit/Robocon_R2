#!/usr/bin/env python3
# Entry point — toàn bộ logic đã chuyển sang r2_bt/
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from r2_bt.main import main

if __name__ == '__main__':
    main()
