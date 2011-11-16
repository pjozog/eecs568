from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np
import os
from pylab import *

rc('text', usetex=True)

fig = figure()

numIterations = array([1, 2, 3, 4, 5,6,7,8,9,10,11,12,13,14,15,16])

chi2Value = array([92.20896169749103,
                   92.2087246046346,
                   92.2087247492832,
                   92.20872474190188,
                   92.20872474245873,
                   92.20872474245873,
                   92.20872474245873,
                   92.20872474245873,
                   92.20872474245873,
                   92.20872474305128,
                   92.20872474304876,
                   92.20872474304794,
                   92.2087247430506,
                   92.2087247430506,
                   92.2087247430506,
                   92.2087247430506])

plot(numIterations, chi2Value)
title("Normalized $\displaystyle\chi^2$ Value versus Number of Iterations for Final State Vector")
xlabel("Number of Iterations")
ylabel("Normalized \chi^2")
grid()
# savefig("%sLearningCurve.png" % imageFileNames[i], transparent=False)
