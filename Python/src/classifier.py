# -*- coding: utf-8 -*-
"""
Created on Wed Nov 16 18:32:26 2016

@author: deepblue
@author: sherrardtr4129
"""

import pandas as pd
import numpy as np
import serial
import RPi.GPIO as GPIO
from sklearn import linear_model, datasets

#constants
ExpectedValue = 0
#need to change this pin number
TeensyInterruptPin = 0

#attach "Interrupts" to Pin for teensy to start 
#weather data collection and classification
GPIO.add_event_detect(TeensyInterruptPin, GPIO.RISING)

def ClassifyPoint():
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
     testpoint=[10,24,30]
     expected=logreg.predict(testpoint) 
     ExpectedValue = expected

GPIO.add_event_callback(TeensyInterruptPin, ClassifyPoint)

