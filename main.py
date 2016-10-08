#!/usr/bin/env python
from classes.exceptionHandlers import ExceptionHandlers
from myApp import MyApp
from params import parameters

e=ExceptionHandlers(parameters)

app = MyApp()

try:
    app.run()
finally:
    e.exceptionFinally()

plt.show()  # for async plt plot window from closing
