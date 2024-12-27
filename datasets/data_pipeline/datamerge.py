import numpy as np
import pickle
import os
from scipy import signal
data=[]
for i in range(0,98):
    filename = '{}.dat'.format(i)
    outfile = open(filename, 'r+b')
    obj=pickle.load(outfile)
    data.extend(obj)
    outfile.close()
xs=[]
ys=[]
    # 平滑特征 预处理数据  插值法
def resample_by_interpolation(signal, n):
    # use linear interpolation
    # endpoint keyword means than linspace doesn't go all the way to 1.0
    # If it did, there are some off-by-one errors
    # e.g. scale=2.0, [1,2,3] should go to [1,1.5,2,2.5,3,3]
    # but with endpoint=True, we get [1,1.4,1.8,2.2,2.6,3]
    # Both are OK, but since resampling will often involve
    # exact ratios (i.e. for 44100 to 22050 or vice versa)
    # using endpoint=False gets less noise in the resampled sound
    resampled_signal = np.interp(
        np.linspace(0.0, 1.0, n, endpoint=False),  # where to interpret
        np.linspace(0.0, 1.0, len(signal), endpoint=False),  # known positions
        signal,  # known data points
    )
    return resampled_signal
for e in data:
    n=len(e[0])
    y=np.concatenate((e[0][0],e[0][n-1],e[1],e[2],e[3]))
    res=[]
    for i in range(7):
        res.append(np.reshape(resample_by_interpolation(e[0][:,i],254),[-1,1]))
    x=np.concatenate(res,1)
    xs.append(x)
    ys.append(y)
xs=np.array(xs)
ys=np.array(ys)
npz_data = {}
npz_data['x_train'] = xs
npz_data['y_train'] = ys
np.savez(os.path.join("robot.npz"), **npz_data)