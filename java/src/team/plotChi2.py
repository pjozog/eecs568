from matplotlib import rc
import matplotlib.pyplot as plt
import numpy as np
import os
from pylab import *

rc('text', usetex=True)

fig = figure()

numIterations = array([1, 2, 3, 4])
chi2Value = array([100, 90, 80, 70])

plot(numIterations, chi2Value)
title("Normalized $\displaystyle\chi^2$ Value versus Number of Iterations for Final State Vector")
xlabel("Number of Iterations")
ylabel("Normalized \chi^2")
grid()
# savefig("%sLearningCurve.png" % imageFileNames[i], transparent=False)


