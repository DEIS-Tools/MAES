#!/usr/bin/env python3
import sys
import csvreader
import numpy
import scipy
import statistics as stats

def blerg(args):
    csv_files = []
    csv = csvreader.CsvReader('test', args[1], False)


    x_name, x = csv.get_column_by_name('radius')
    y_name, y = csv.get_column_by_name('ratio')
    x_log = numpy.log(x[1:])
    print(x[1:])
    #print(x_log)
    ylog = numpy.log(y[1:])
    #print(y[1:])
    #for point in range(len(y)):
    #    y[point] -= trolol
    #print(y[1:])

    #blerp = stats.linear_regression(x_log, y[1:], proportional=False)
    blerp = numpy.polyfit(x_log, y[1:], 1, w=numpy.sqrt(x[1:]))
    #
#    blerp = scipy.optimize.curve_fit(func, x[1:], y[1:])
    print(blerp)

#    p = numpy.poly1d(blerp)
#    print(p)

def func(x, a, b, c):
    return a*numpy.log(b+x)+c

blerg(sys.argv)
