import matplotlib.pyplot as plt 
import csv 
  
x = [] 
y = [] 
  
with open('rpm_data.csv','r') as csvfile: 
    plots = csv.reader(csvfile, delimiter = ',') 
      
    for row in plots: 
        x.append(row[0]) 
        y.append(float(row[1])) 
  
plt.plot(x, y, color = 'b') 
plt.xlabel('Time (seconds)') 
plt.ylabel('RPM') 
plt.title('Time vs RPM') 
plt.legend() 
plt.show() 

