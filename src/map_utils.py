#! /usr/bin/env python
import math


def mapToGrid(x, y):
    '''
    Alert! Returns (X, Y) on Grid from (x, y) on map
    '''
    X = x + 9
    Y = 9 - y
    return (X, Y)


def gridToMap(X, Y):
    '''
    Alert! Returns (x, y) on map from (X, Y) on Grid
    '''
    x = X - 9
    y = 9 - Y
    return (x, y)


def inGridCell(x, y):
    '''
    Alert! Returns a tuple (column, row)
    '''
    (X, Y) = mapToGrid(x, y)
    return (int(math.floor(X)), int(math.ceil(Y)))


def gridCenter(row, col):
    '''
    Alert! Input is (row, col) but output is (x, y)
    '''
    X = col + 0.5
    Y = row - 0.5

    #if col == 8 and row == 13:
    #    X += 0.3
    #    Y += 0.4
    return gridToMap(X, Y)


if __name__ == "__main__":
    print gridToMap(13, 1)
    print mapToGrid(-8.5, -2.5)
    print inGridCell(-5.5, 1.5)
    print gridCenter(0, 0)
