"""
Created on Sat Jul 4 14:51:36 2020

@author: dgupta
"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class plotBagTest():
    """ Reading Data from Greg's BAG Test csv file and plot the columns vs time(s)"""
    def __init__(self, filen, title=None):
        self.filen = filen
        self.title = title
        self.head = ['millis', 'mSLPM', 'umH2O', 'cpuLoad', 'position',
                'temperature' , 'direction', 'pwmSpeed', 'analogRead']
        if title==None:
            self.title = self.filen
        
    def loadfile(self, skiprows=8):
        df = pd.read_csv(self.filen, skiprows=skiprows, header=None, 
            names=self.head)
        df.index = (df.iloc[:,0] - df.iloc[0,0])/1000
        df.index.name = 'sec'
        return df
    
    def run(self): #, png=False):
        df = self.loadfile()
        df.plot(figsize=(10,12), subplots=True, grid=True, title=self.title);
        # if png:
        #     plt.savefig('out.png', bbox_inches='tight')
        plt.show();

if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		usage='python {0} datafile.csv -t "title_text"'.format(__file__))
	parser.add_argument("filename", help="Filename of .csv data file")
	parser.add_argument('-t', 
		help = "Graph title text. Uses filename if none.")
	args = parser.parse_args()

	if args.t is None:
		plotBagTest(args.filename, args.filename).run()
	else:
		plotBagTest(args.filename, args.t).run()
	
