import commands2

class PrintSomethingCommand(commands2.Command):
    def __init__(self, msg):
        super().__init__()
        self.msg = msg

    def initialize(self):  # Setting function
        pass

    def execute(self):  # What actions it does
        print(self.msg, flush=True)

    def isFinished(self):
        return True
