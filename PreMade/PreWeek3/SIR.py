import random
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

def prior(k):
    sample = np.random.uniform(0, 15, k)
    return sample

def N(x, mu, sigma):
    N = (1 / (np.sqrt(2 * np.pi) * sigma)) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)
    return N

def p(x):
    return (0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0))

def normweights(sample):
    weights = p(sample)
    weights = weights / np.sum(weights)
    return weights

def resampling(weights, N=15):
    nice_samples = prior(N)
    nice_weights = p(nice_samples)
    nice_normweights = normweights(nice_samples)
    new_sample = np.random.choice(nice_samples, size=N, p=nice_normweights)
    return new_sample

# Set k as a single integer, not a list
k = 20
sample = prior(k)
weights = normweights(sample)
new_sample = resampling(weights, N=k)

# Plot the sample using matplotlib
plt.hist(sample, bins=25, facecolor='red')
plt.show()
