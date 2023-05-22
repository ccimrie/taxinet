import numpy as np
import re
import cv2
from ast import literal_eval

f=open('data.txt','r')
datapoint_string=f.readline()
datapoint_formatted=re.sub("\s+",",",datapoint_string[:-1])
datapoint_formatted=re.sub("\[,","[",datapoint_formatted)
datapoint=np.array(literal_eval(datapoint_formatted))
datapoint_image=datapoint.astype(np.uint8)
cv2.imshow('test2', datapoint_image)
cv2.waitKey(10000)