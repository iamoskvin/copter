#!/usr/bin/env python

import speedtest

st = speedtest.Speedtest()
print(st.download())
print(st.upload())