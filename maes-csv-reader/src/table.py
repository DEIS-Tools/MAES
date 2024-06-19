# Copyright 2024 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http:#www.gnu.org/licenses/.
# 
# Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
# 
# Original repository: https:#github.com/Molitany/MAES

class TableRow:
    name: str
    average: float
    successes: int
    timeouts: int
    successRate: float
    fastestSuccess: int
    slowestSuccess: int
    def __init__(self, name, average, successes, timeouts, successRate, fastestSuccess, slowestSuccess):
        self.name = name
        self.average = average
        self.successes = successes
        self.timeouts = timeouts
        self.successRate = successRate
        self.fastestSuccess = fastestSuccess
        self.slowestSuccess = slowestSuccess

class Table:
    rows: [TableRow]
    def __init__(self):
        self.rows = []

    def add_row(self, row: TableRow):
        self.rows.append(row)
