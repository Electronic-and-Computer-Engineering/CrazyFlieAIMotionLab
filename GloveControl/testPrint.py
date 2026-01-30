import sys
from printTerminal import PrintTerminal

def main():
    terminal = PrintTerminal()
    
    i = 0
    
    while 1:
    
        terminal.addLine(f"1: {i}")
        i = i+1
        terminal.addLine(f"2: {i}")
        i = i+1
        terminal.addLine(f"3: {i}")
        i = i+1
        terminal.addLine(f"4: {i}")
        
        terminal.printAllLines()
        #terminal.clearBuffer()
    
    
if __name__ == '__main__':
    main()