# -*- coding: utf-8 -*-
"""
Created on Wed Nov 16 18:32:26 2016

@author: deepblue
@author: sherrardtr4129
"""

import pandas as pd
import numpy as np
import sys
import serial
from sklearn import linear_model, datasets

port = "/dev/ttyACM0"
Serial = serial.Serial(port, 9600, timeout=5)

def ClassifyPoint(InPoint):
     dataseta = pd.read_csv("/home/sherrardtr/CHAAC_Code/Python/data/weather_data.csv", header =-1)
     dataset=dataseta.as_matrix()
     data=np.zeros((365,8))
     data= dataset
     training_data=np.zeros((365,3))
     trainingdata = data[:,[3,4,5]]
     trainingdata=trainingdata.astype(float)
     labels=np.zeros((365,1))
     labels=data[:,[6]]
     labels=labels.astype(float)
     logreg = linear_model.LogisticRegression(C=1e5)
     logreg.fit(trainingdata, labels)
     testpoint=np.zeros((1,3))
     testpoint=InPoint
     expected=logreg.predict(testpoint)
     return expected


while True:
     msg = Serial.readline()
     if(msg != "" and 'T' in msg ):
          DataList = msg.split(',')
          Temp = DataList[0][1:]
          Pressure = DataList[1][1:]
          Humidity = DataList[2][1:]
          Classification = ClassifyPoint([float(Temp),float(Humidity),float(Pressure)])
          if(Classification == 1):
               Serial.write(unicode('x'))
               Serial.write(unicode('u'))
          elif(Classification == 2):
              Serial.write('x')
              Serial.write('q')
          elif(Classification == 3):
              Serial.write('x')
              Serial.write('x')
          elif(Classification == 4):
              Serial.write('x')
              Serial.write('y')
          elif(Classification == 5):
              Serial.write('x')
              Serial.write('z')

Serial.close()

