from math import fabs
import numpy as np


def rmedian(x, n, v=0.0, align=0.5):
    a = x.astype(np.float64) if not x.dtype == np.float64 else x
    z = np.full(x.shape[0], v, dtype=np.float64)
    buf = np.sort(a[:n])
    h = max(0, min(int(n * align), n - 1))
    #
    if n & 1:
        z[h] = buf[h]
    else:
        z[h] = (buf[h - 1] + buf[h]) / 2.0
    #
    for i in range(a.shape[0] - n):
        ins = a[i + n]
        rem = a[i]
        j = 0
        l = n
        k = h
        #
        while k > 0:
            if buf[j + k] < rem:
                j = j + k
            else:
                l = j + k
            k = (l - j) >> 1
        #
        if not (l == n or fabs(buf[j] - rem) < fabs(buf[l] - rem)):
            j = l
        #
        if ins > rem:
            while j < n - 1 and buf[j + 1] < ins:
                buf[j] = buf[j + 1]
                j += 1
            buf[j] = ins
        elif ins < rem:
            while j > 0 and buf[j - 1] > ins:
                buf[j] = buf[j - 1]
                j -= 1
            buf[j] = ins
        #
        if n & 1:
            z[i + h + 1] = buf[h]
        else:
            z[i + h + 1] = (buf[h - 1] + buf[h]) / 2.0
    return z

def rmean(x, n, pad=1, align=0.5):
    xl = x.shape[0]
    a = x.astype(np.float64) if not x.dtype == np.float64 else x
    z = np.zeros(xl, np.float64)
    h = 1 + max(0, min(int(n * align), n - 1))
    s = 0.0
    #
    for i in range(n):
        s += a[i]
    z[h - 1] = s / n
    #
    for i in range(xl - n):
        s += a[i + n] - a[i]
        z[i + h] = s / n
    #
    if pad == 0:
        return z
    #
    elif pad == 1:
        s = z[h - 1]
        for i in range(h - 1):
            z[i] = s
        s = z[xl + h - n - 1]
        for i in range(xl + h - n - 1, xl):
            z[i] = s
        #
        return z
    #
    elif pad == -1:
        return z[h - 1:xl + h - n]


def umw_denoise(x, sens=5, rad=3, detrend=True):  
  if x.ndim == 2:
    for i in range(x.shape[1]):
      x[:, i] = umw_denoise(x[:, i], sens, rad)
    return x
  w = 2 * rad
  xtype = x.dtype
  base_line = x.mean()
  x = x - base_line
  mu = rmean(x, w)
  md = rmedian(x, w)
  ad = np.abs(x - mu)
  ad /= ad.std()
  marker = np.where(ad >= (ad.mean() + ad.std() * sens))[0]
  mj = np.clip(marker - rad - 1, 0, x.shape[0] - 1)
  mk = np.clip(marker + rad + 1, 0, x.shape[0] - 1)
  x[marker] = 0.5 * (md[mj] + md[mk])
  #
  for i in range(1, rad):
    j = np.maximum(0, marker - i)
    k = np.minimum(x.shape[0] - 1, marker + i)
    x[j] = md[mj]
    x[k] = md[mk]
  x += base_line
  if(detrend):
    ## remove linear component
    coefficients = np.polyfit(range(len(x)), x, 1)
    fit_line = np.polyval(coefficients, x)
    x = x - fit_line
    #
  return x.astype(xtype)
