import numpy as np
import matplotlib.pyplot as plt

def prior_distribution_uniform(size):
    # Uniform distribution for Question 1
    return np.random.uniform(0, 15, size=size)

def prior_distribution_normal(size):
    # Normal distribution N(5, 4) for Question 2
    return np.random.normal(loc=5, scale=4, size=size)

def N(x, mu, sigma):
    # Gaussian distribution
    return (1/(np.sqrt(2*np.pi)*sigma) * np.exp(-0.5 * ((x - mu) / sigma) ** 2))

def p(x):
    # Mixture of 3 Gaussians
    return (0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0))

def SIR(y_obs, N_samples=1000, resample_size=100, prior_func=prior_distribution_uniform):
    # Step 1: Sample from the proposal distribution (prior)
    theta_samples = prior_func(N_samples)
    
    # Step 2: Compute importance weights (based on p(x))
    weights = p(theta_samples)
    
    # Step 3: Normalize the weights
    normalized_weights = weights / np.sum(weights)
    
    # Step 4: Resample based on weights
    resampled_indices = np.random.choice(np.arange(N_samples), size=resample_size, p=normalized_weights)
    resampled_theta = theta_samples[resampled_indices]
    
    return resampled_theta

def plot_results(k_values, prior_func):
    x = np.linspace(0, 15, 1000)
    px = p(x)

    for k in k_values:
        posterior_samples = SIR(y_obs=2.0, N_samples=k, resample_size=k, prior_func=prior_func)
        
        # Plot histogram of resampled samples
        plt.hist(posterior_samples, bins=30, density=True, alpha=0.7, label=f'k={k} Resampled')

        # Plot the true distribution p(x)
        plt.plot(x, px, 'r-', label='True Distribution p(x)', lw=2)
        
        # Set title and labels
        plt.title(f'Posterior Distribution (SIR) with k={k}')
        plt.xlabel('Theta')
        plt.ylabel('Density')
        plt.legend()
        plt.show()

# For Question 1: Uniform proposal distribution q(x)
k_values = [20, 100, 1000]
print("Question 1: Uniform proposal distribution")
plot_results(k_values, prior_func=prior_distribution_uniform)

# For Question 2: Normal proposal distribution q(x) ~ N(5, 4)
print("Question 2: Normal proposal distribution")
plot_results(k_values, prior_func=prior_distribution_normal)
