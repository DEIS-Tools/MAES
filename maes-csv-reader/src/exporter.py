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
                       f'legend pos = outer north east,\n' + \
                       f'cycle list name=custom,\n' + \
                       f'scaled y ticks = false,\n' + \
                       f'yticklabel={{\\pgfmathprintnumber[fixed]{{\\tick}}}}]\n\n'

        for i, plot in enumerate(self.plots):
            self.export_text += f'\\addplot+[mark=none] coordinates {{\n'
            for datapoint in plot.cords:
                self.export_text += f'{datapoint}\n'

            self.export_text +=  f'}};\n' + \
                            f'\\addlegendentry{{{plot.name}}}\n\n\n'

        self.export_text += f'\\end{{axis}}\n' + \
                        f'\\end{{tikzpicture}}\n'
        self.file_extension = ".tikz"

    def export_latex_table(self, options):
        """
        Method to export tables to LaTeX
        """
        if self.table == None:
            print("no table was provided")
            return 1

        self.export_text = ''
        self.export_text += f'\\begin{{table*}}[]\n' + \
                        f'\\begin{{tabular}}{{|c|c|c|c|c|c|c|}}\n' + \
                        f'\\hline\n' + \
                        f'Strategy & Average Ticks & Successes & Timeouts & Success Rate & Fastest Success (Ticks) & Slowest Success (Ticks) ' + '\\' + '\\' + '\\hline\n'

        for row in self.table.rows:
            self.export_text += f'{row.name} & {row.average} & {row.successes} & {row.timeouts} & {row.successRate} & {row.fastestSuccess if row.fastestSuccess != 36000 else "DNF"} & {row.slowestSuccess if row.slowestSuccess != 0 else "DNF"}' + '\\' + '\\' + '\\hline\n'

        self.export_text += f'\\end{{tabular}}\n' + \
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
