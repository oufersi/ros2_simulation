import subprocess
import math

if __name__ == "__main__":
    # while True:
    #     angle = float(input())
    #     print(angle * math.pi / 180)
    while True:
        symbol = int(input())
        if symbol == 1:
            subprocess.Popen(
                ["gz", "topic", "-t", "/B1/detach", "-m", "gz.msgs.Empty", "-p", "unused: true"]
            )
        elif symbol == 2:
            subprocess.Popen(
                ["gz", "topic", "-t", "/B1/attach", "-m", "gz.msgs.Empty", "-p", "unused: true"]
            )
    