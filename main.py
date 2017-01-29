#!/usr/bin/env python
from classes.myApp import MyApp, e
app = MyApp()

try:
    app.run()

finally:
    e.exceptionFinally()

# plt.show()  # for async plt plot window from closing
