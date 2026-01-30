import sys

class PrintTerminal:
    def __init__(self):
        sys.stdout.write("\033[2J")
        self.outputLines = ["--- REALTIME MONITOR ---"]
        self.finalOutput = ""

    def addLine(self, send:str):
        self.outputLines.append(send)
        
    def clearBuffer(self):
        self.outputLines = ["--- REALTIME MONITOR ---"]
        self.finalOutput = ""
        sys.stdout.write("\033[2J")
        sys.stdout.flush()
        
    def printAllLines(self):
        self.finalOutput = "\n".join(self.outputLines)
        sys.stdout.write("\033[H" + self.finalOutput)
        sys.stdout.flush()
        self.outputLines = ["--- REALTIME MONITOR ---"]
        self.finalOutput = ""