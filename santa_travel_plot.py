import numpy as np

#For plotting
import seaborn as sns
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt

datafile = "nicelist.txt"
data_txt = np.genfromtxt(datafile, delimiter=';')

results = {}
i0=0

filepath = "result.txt"
with open(filepath) as fp:  
	while True:
		line = fp.readline()
		if len(line)>1:
			lines = line[:-1].split(';')
			temp = np.array([29.315278,68.073611])
			for i in lines:
				lat  = data_txt[np.isin(data_txt[:,0],int(i))][:,2][0]
				long = data_txt[np.isin(data_txt[:,0],int(i))][:,1][0]
				temp = np.vstack((temp, np.array([lat,long])))
			temp = np.vstack((temp,	np.array([29.315278,68.073611])))
		if len(line)<1:
			break
		results[i0]=temp

		i0 += 1


#Creating and saving images from results		
temp_results = []
for i in range(len(results)):

	if len(temp_results) < 10:
		temp_results.append(results[i])
	if len(temp_results) == 10:
		temp_results.pop(0)
	if i > 1:
		fig = plt.figure(figsize=(12, 6))
		map = Basemap()
		map.drawcoastlines()
		map.fillcontinents(color='lightgray', alpha=0.1)

		lat = data_txt[:,1]
		long = data_txt[:,2]

		x, y = map(long, lat)
		
		map.scatter(x, y, marker='.',color='blue', alpha=0.1)
		map.scatter(29.315278,68.073611, color='red')
		plt.title("Santas travel "+str(i))

		i0=1
		for k in temp_results:
			temp = k
			map.plot(temp[:,0],temp[:,1],color='red',alpha=0.1*i0)
			i0 += 1
		
		im_name = str(i)+'.png'
		plt.savefig(im_name)
		plt.close()




