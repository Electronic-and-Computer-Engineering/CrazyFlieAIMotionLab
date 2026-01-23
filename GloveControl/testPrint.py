import sys

sys.stdout.write("\033[2J")

def main():
    i = 0
    
    while True:
        i = i+1
        j = i+1
        
        output_lines = ["--- REALTIME MONITOR ---"]
        
        send = f"POS: ({i:6.3f}) n "
        output_lines.append(send)
        
        send = f"POS: ({i:6.3f}) n "
        output_lines.append(send)
        
        final_output = "\n".join(output_lines)
        sys.stdout.write("\033[H" + final_output)
        sys.stdout.flush()
    

if __name__ == '__main__':
    main()