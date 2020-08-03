#%reset -f
#%matplotlib notebook 
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class plotBagTest():
    """ Reading Data from Greg's BAG Test csv file and plot the columns vs time(s)"""
    def __init__(self, filen, title):
        self.filen = filen
        self.title = title
        self.head = ['millis', 'mSLPM', 'umH2O', 'cpuLoad', 'position',
                'temperature' , 'direction', 'pwmSpeed', 'analogRead']
        
    def loadfile(self):
        df = pd.read_csv(self.filen, skiprows=10, header=None, names=self.head)
        df.index = (df.iloc[:,0] - df.iloc[0,0])/1000
        df.index.name = 'sec'
        return df
    
    def run(self):
        df = self.loadfile()
        df.plot(figsize=(10,12), subplots=True, grid=True, title=self.title);
