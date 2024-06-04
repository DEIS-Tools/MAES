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

import utility
import plot as p
import table as t

class Exporter:
    export_text_tikz: str
    export_text_table: str
    file_extension: str
    plots: [p.Plot]
    table: t.Table

    def __init__(self, plots, table):
        self.export_text = ''
        self.file_extension = ''
        self.plots = plots
        self.table = table

    def export_to_tikz(self, options):
        """
        Method to export plots to tikz
        """
        self.export_text = ''
        self.export_text += f'\\begin{{tikzpicture}}\n' + \
                       f'\\begin{{axis}}[\n' + \
                       f'xlabel={{Map}},\n' + \
                       f'ylabel={{{options.y_column_name}}},\n' + \
                       f'legend style={{font=\\fontsize{{7}}{{7}}\\selectfont}},\n' + \
                       f'legend pos = north west,\n' + \
                       f'cycle list name=custom,\n' + \
                       f'ymode=log,\n' + \
                       f'log ticks with fixed point,\n' + \
                       f'ytick={{10,100,1000,10000,36000}},\n' + \
                       f'grid=both,\n' + \
                       f'hide obscured y ticks=false,\n' + \
                       f'ymin=100,\n' + \
                       f']\n\n'

        for i, plot in enumerate(self.plots):
            self.export_text += f'\\addplot+[mark=none] coordinates {{\n'
            for datapoint in plot.cords:
                self.export_text += f'{datapoint}\n'

            self.export_text +=  f'}};\n' + \
                            f'\\addlegendentry{{{plot.name}}}\n\n\n'

        self.export_text += f'\\end{{axis}}\n' + \
                        f'\\end{{tikzpicture}}\n'
        self.file_extension = ".tikz"

    def export_to_figure(self, options):
        splittedName = self.plots[0].name.split('-')
        spawn_text = "spawning together" if splittedName[6] == "spawntogether" else "spawning apart"

        self.export_text = ''
        self.export_text = '\\begin{figure*}\n' + \
            '\\centering\n' + \
            f'\\includegraphics[width=\\textwidth]{{figures/tikz/{options.export_file_name}}}\n' + \
            f'\\caption{{Cactus plot over the various strategies with the configuration that uses a map size of {splittedName[2]}, using {splittedName[3]} for communication, and {spawn_text}.}}\n' + \
            f'\\label{{fig:{splittedName[2]}-{splittedName[6]}-{splittedName[3]}}}\n' + \
            '\\end{figure*}'

        self.file_extension = ".tex"

    def export_latex_table(self, options):
        """
        Method to export tables to LaTeX
        """
        if self.table == None:
            print("no table was provided")
            return 1
        splittedName = self.table.rows[0].name.split('-')

        self.export_text = ''
        self.export_text += f'\\begin{{table*}}[]\n' + \
                        f'\\begin{{adjustbox}}{{max width=\\textwidth}}\n' + \
                        f'\\begin{{tabular}}{{|c|c|c|c|c|c|c|}}\n' + \
                        f'\\hline\n' + \
                        f'Strategy & Average Ticks & Successes & Timeouts & Success Rate & Fastest Success (Ticks) & Slowest Success (Ticks) ' + '\\' + '\\' + '\\hline\n'

        for row in self.table.rows:
            self.export_text += f'{row.name} & {"{:,.2f}".format(round(row.average, 2))} & {row.successes} & {row.timeouts} & {"{:,}".format(round(row.successRate, 2))} & {"{:,}".format(row.fastestSuccess) if row.fastestSuccess != 36000 else "DNF"} & {"{:,}".format(row.slowestSuccess) if row.slowestSuccess != 0 else "DNF"}' + '\\' + '\\' + '\\hline\n'

        spawn_text = "spawning together" if splittedName[6] == "spawntogether" else "spawning apart"
        self.export_text += f'\\end{{tabular}}\n' + \
                        f'\\end{{adjustbox}}\n' + \
                        f'\\caption{{Table over the various strategies with the configuration that uses a map size of {splittedName[2]}, using {splittedName[3]} for communication, and {spawn_text}.}}\n' + \
                        f'\\label{{tab:{splittedName[2]}-{splittedName[6]}-{splittedName[3]}}}\n' + \
                        f'\\end{{table*}}\n'

        self.file_extension = ".tex"



    def save_to_file(self, filename):
        """
        Method to save the output of an export method
        """
        if self.file_extension == '':
            options.append_error("Please make sure to pick an export format!")
            return

        file = open(f"{os.path.join(utility.get_root_dir(), 'datasets', 'exports', filename + self.file_extension)}" ,"w")
        file.write(self.export_text)
        file.close()

if __name__ == '__main__':
    exporter = Exporter('testx', 'testy', [5.4, 5.3, 2.2], 'test')
    exporter.export_to_tikz()
    exporter.save_to_file()
