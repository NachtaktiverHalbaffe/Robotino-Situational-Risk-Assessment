# Create a Pandas DataFrame.
from pickletools import uint8
import pandas as pd
import numpy as np
technologies = {
    'Courses':["Spark","PySpark","Python","pandas"],
    'Fee' :[20000,25000,22000,30000],
    'Duration':['30days','40days','35days','50days'],
    'Discount':[1000,2300,1200,2000]
              }
index_labels=['0','1','2','3']
df = pd.DataFrame(technologies,index=index_labels)
#print(df)

df = pd.DataFrame(technologies,index=index_labels)
df['new_columns'] = np.where(df['Fee'] >= 25000, df['Fee'], df.Duration[df['Fee'].index.astype(int)])

print(df)