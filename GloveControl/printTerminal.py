import sys

class PrintTerminal:
    def __init__(self):
        sys.stdout.write("\033[2J")
        self.outputLines = ["--- REALTIME MONITOR ---"]
        self.finalOutput = ""

    def addLine(self, send:str):
        self.outputLines.append(send)
        
    def printAllLines(self):
        self.finalOutput = "\n".join(self.outputLines)
        sys.stdout.write("\033[H" + self.finalOutput)
        sys.stdout.flush()