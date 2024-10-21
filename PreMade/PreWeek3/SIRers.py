import numpy as np
import matplotlib.pyplot as plt

def prior_distribution_uniform(size):
    return np.random.uniform(0, 15, size=size)

def prior_distribution_normal(size):
    return np.random.normal(loc=5, scale=4, size=size)

def N(x, mu, sigma):
    return (1/(np.sqrt(2*np.pi)*sigma) * np.exp(-0.5 * ((x - mu) / sigma) ** 2))

def p(x):
    return (0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0))

def SIR(N_samples=1000, resample_size=100, prior_func=prior_distribution_uniform):
    nice_samples = prior_func(N_samples)
    
    weights = p(nice_samples)
    
    normalized_weights = weights / np.sum(weights)
    
    resampled_indices = np.random.choice(np.arange(N_samples), size=resample_size, p=normalized_weights)
    resampled_nice_samples = nice_samples[resampled_indices]
    
    return resampled_nice_samples

def plot_results(k_values, prior_func):
    x = np.linspace(0, 15, 1000)
    px = p(x)

    for k in k_values:
        posterior_samples = SIR(N_samples=k, resample_size=k, prior_func=prior_func)
        
        plt.hist(posterior_samples, bins=30, density=True, alpha=0.7, label=f'k={k} Resampled')

        plt.plot(x, px, 'r-', label='True Distribution p(x)', lw=2)
        
        plt.title(f'Posterior Distribution (SIR) with k={k}')
        plt.xlabel('State')
        plt.ylabel('Density')
        plt.legend()
        plt.show()

k_values = [20, 100, 1000]
print("Question 1: Uniform proposal distribution")
plot_results(k_values, prior_func=prior_distribution_uniform)

print("Question 2: Normal proposal distribution")
plot_results(k_values, prior_func=prior_distribution_normal)
