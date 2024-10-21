import matplotlib.pyplot as plt

# Example data (replace with your actual data)
actual_distances = [300, 500, 1000, 1500, 2000]  # Actual distances (expected)
measured_distances = [309.5, 504.75, 995.5, 1500.25, 1995]  # Measured distances
#300mm
#mean = 9.5, variance = 9, standard deviation = 3

#500mm
#mean = 4.75, variance = 8.25, standard deviation = 2.87

#1000mm
#mean = -4.5, variance = 11, standard deviaton = 3.32

#1500mm
#mean = 0.25, variance = 53.58, sd = 7.32

#2000mm
#mean = -5, variance = 0.66, sd = 0.82

# Create the scatter plot
plt.figure(figsize=(8, 6))
plt.scatter(actual_distances, measured_distances, color='blue', label='Measured Distances')

# Add a reference line y = x
plt.plot(actual_distances, actual_distances, color='red', linestyle='--', label='Expected (y=x)')

# Add labels and title
plt.xlabel('Actual Distance (mm)')
plt.ylabel('Measured Distance (mm)')
plt.title('Measured Distance vs Actual Distance')

# Add a legend
plt.legend()

# Show the plot
plt.grid(True)

plt.savefig('scatterplot.png')
plt.show()
