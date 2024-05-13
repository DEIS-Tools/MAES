class Options:
    """
    Class to hold globally used options
    """
    x_column_name: str
    y_column_name: str
    plot_type: str
    absolute_path: bool
    file_names: [str]
    file_identifiers: [str]
    directory_names: [str]
    export_file_name: [str]
    show_plot: bool
    errors: [str]

    def __init__(self):
        self.x_column_name = ""
        self.y_column_name = ""
        self.plot_type = "base"
        self.absolute_path = False
        self.file_names = []
        self.file_identifiers = []
        self.directory_names = []
        self.export_file_name = ""
        self.show_plot = False
        self.errors = []

    def set_x(self, arg: str):
        self.x_column_name = arg

    def set_y(self, arg: str):
        self.y_column_name = arg

    def set_plot_type(self, arg: str):
        self.plot_type = arg

    def set_abs_path(self, arg: bool):
        self.absolute_path = arg

    def append_filename(self, arg: str):
        self.file_names.append(arg)

    def append_fileid(self, arg: str):
        self.file_identifiers.append(arg)

    def append_directory(self, arg: str):
        self.directory_names.append(arg)

    def set_export_name(self, arg: str):
        self.export_file_name = arg

    def set_show_plot(self, arg: bool):
        self.show_plot = arg

    def append_error(self, err: str):
        self.errors.append(err)

    def print_options(self):
        print(self.x_column_name)
        print(self.y_column_name)
        print(self.file_names)
        print(self.file_identifiers)
        print(self.export_file_name)
        print(self.show_plot)
        print(self.errors)
