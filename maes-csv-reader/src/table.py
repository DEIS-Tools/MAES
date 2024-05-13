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
