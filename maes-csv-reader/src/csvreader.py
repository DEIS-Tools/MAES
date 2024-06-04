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

import os

import csv
import utility

class CsvReader():
    """
    Class in charge of loading and storing a csv file.
    """
    object_name: str
    file_name: str
    csv_text: [str]
    absolute_path: bool
    def __init__(self, name: str, file_name: str, absolute_path):
        """
        Constructor for the CsvReader class, expects a name (identifier) and file name.
        """
        self.object_name = name
        self.file_name = file_name
        self.absolute_path = absolute_path
        self.csv_text = []
        self.__load_csv()

    def __load_csv(self):
        """
        Loads a csv file into the self.csv_text variable. Private method.
        """
        if self.absolute_path == True:
            with open(self.file_name) as f:
                text = csv.reader(f, delimiter=',', quotechar='"')
                for line in text:
                    self.csv_text.append(line)
        else:
            with open(f"{os.path.join(utility.get_root_dir(), 'datasets', f'{self.file_name}')}", newline='') as f:
                text = csv.reader(f, delimiter=',', quotechar= '"')
                for line in text:
                    self.csv_text.append(line)

    def get_column(self, column: int):
        """
        Gets column name and values based on a column number.
        Returns column name: str, column value list: [float].
        """
        result = []
        column_name = ""
        for x, row in enumerate(self.csv_text):
            if x == 0:
                column_name = row[column]
                result.append(0)
            else:
                result.append(float(row[column]))
        return column_name, result

    def get_column_by_name(self, name: str):
        """
        Gets column name and values based on column name.
        Returns column name: str, column value list: [float].
        """
        for x, col_name in enumerate(self.csv_text[0]):
            if col_name == name:
                return self.get_column(x)

    def get_last_column_element(self, name: str):
        col = self.get_column_by_name(name)[1]
        return col[-1]


    def get_csv_name(self):
        """
        Returns identifier name.
        """
        return self.object_name

    def print_csv_text(self):
        """
        Prints self.csv_text.
        """
        for row in self.csv_text:
            print(row)

if __name__ == '__main__':
    """
    Debugging code, disregard.
    """
    csv_reader = CsvReader()
    csv_reader.load_csv('test')
    csv_reader.print_csv_text()
