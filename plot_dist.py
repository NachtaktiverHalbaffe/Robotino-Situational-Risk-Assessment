
import pandas as pd

import matplotlib.pyplot as  plt
from matplotlib import pyplot
import pandas as pd
import numpy as np

finaloutput = "dist_loc/error_dist_loc_dist_filtered2.csv"
# finaloutput = "dist_loc/error_dist_loc_dist2.csv"
# finaloutput = "dist_loc/error_dist_loc_rot_abs_chair4.csv"
finaloutput = "dist_loc/error_dist_loc_dist_filtered6.csv"
dataset_final = pd.read_csv(finaloutput, delimiter= ",")
dataset_final.columns = ['data']
#print(dataset_final)
#dataset_final = dataset_final.astype({"yolo_center_x":"float","yolo_width":"float","ref_center_x":"float","ref_width":"float"})

#plt.hist(dataset_final["center_error"], bins = 5)
#plt.show()
#print(dataset_final)
X= dataset_final["data"]
# for x,i in zip(X,range(len(X))):
    # X[i] = float(x[7:-23])
    # X[i] = float(x[7:-1])

print(np.mean(X))
print(max(X))
print(min(X))
amount_bins = 5+1#plus one is needed to make the number before the actual amount
# bins = np.linspace(0.0,0.20,amount_bins)
bins = np.linspace(0.0,10,amount_bins)
# counts, bins = np.histogram(X, bins = bins, range = [-5, 10], density = False)
counts, bins = np.histogram(X, bins = bins,range = [-5, 10], density = False)
binscenters = np.array([0.5 * (bins[i] + bins[i+1])for i in range(len(bins)-1)])
counts = counts/len(X)
print(sum(counts))
xspace = bins
plt.bar(binscenters, counts, color='navy',width=7.17/amount_bins, label=r'Histogram entries')
#plt.plot(xspace, exponential(xspace, *popt_exponential), color='darkorange', linewidth=2.5, label=r'Fitted function')
plt.xlabel("Deviation in pixels")
plt.ylabel("No.Of Observations")
plt.show()

a = [2,3,4,5]

