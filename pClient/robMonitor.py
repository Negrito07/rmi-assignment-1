import csv
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Monitor:
    def __init__(self, filename):
        self.filename = filename
        self.fieldnames = ['ti', 'error', 'p', 'i', 'd', 'lpow', 'rpow']
    
    def animate(self, i):
        data = pd.read_csv(self.filename)

        x = data['ti']

        plt.cla()
        plt.plot(x, data['error'], label='error')

        plt.legend(loc='upper right')

    def start(self):
        with open(self.filename, 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()

        ani = FuncAnimation(plt.gcf(), self.animate, interval=1000)

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    mon=Monitor("log")
    
    mon.start()