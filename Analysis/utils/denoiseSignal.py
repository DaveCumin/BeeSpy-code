import numpy as np
from scipy.ndimage import uniform_filter1d, median_filter


def rmean(x, n, pad=1, align=0.5):
    """Running mean via scipy's C implementation — replaces the pure-Python loop."""
    a = x.astype(np.float64) if x.dtype != np.float64 else x
    return uniform_filter1d(a, size=n, mode='nearest')


def rmedian(x, n, v=0.0, align=0.5):
    """Running median via scipy's C implementation — replaces the pure-Python loop."""
    a = x.astype(np.float64) if x.dtype != np.float64 else x
    return median_filter(a, size=n, mode='nearest')


def umw_denoise(x, sens=5, rad=3):
    """Spike / impulse noise removal.

    Identical logic to ProPro076's umw_denoise (winf.pyx), but using
    scipy.ndimage C kernels instead of pure-Python rolling loops.

    The detrend step present in the previous version has been removed:
      - it contained a bug (polyval was evaluated at signal values, not
        time indices, so it did not remove a linear trend)
      - it is redundant: dospectrogram() already passes detrend='linear'
        to scipy.signal.spectrogram, which detrends each FFT window.
    """
    if x.size == 0:
        return x

    if x.ndim == 2:
        for i in range(x.shape[1]):
            x[:, i] = umw_denoise(x[:, i], sens, rad)
        return x

    w = 2 * rad
    xtype = x.dtype
    base_line = x.mean()
    x = x.astype(np.float64) - base_line

    mu = rmean(x, w)
    md = rmedian(x, w)

    ad = np.abs(x - mu)
    std_ad = ad.std()
    if std_ad == 0 or not np.isfinite(std_ad):
        x += base_line
        return x.astype(xtype)
    ad /= std_ad
    marker = np.where(ad >= (ad.mean() + ad.std() * sens))[0]

    mj = np.clip(marker - rad - 1, 0, x.shape[0] - 1)
    mk = np.clip(marker + rad + 1, 0, x.shape[0] - 1)
    x[marker] = 0.5 * (md[mj] + md[mk])

    for i in range(1, rad):
        j = np.maximum(0, marker - i)
        k = np.minimum(x.shape[0] - 1, marker + i)
        x[j] = md[mj]
        x[k] = md[mk]

    x += base_line
    return x.astype(xtype)
