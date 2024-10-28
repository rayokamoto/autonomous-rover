import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data from CSV file
df = pd.read_csv('results/final/rpe_optimisemaxerror.csv')  # Change to desired csv file

# Extracting data for plotting
statistics = ['RMSE', 'Mean', 'Median', 'Std', 'Min', 'Max']
data = df[statistics].values
labels = df['OptimiseMaxError']  # Change to desired label

# Set the number of statistics and feature types
n_statistics = len(statistics)
n_features = len(labels)

# Create a new index for the bar positions
bar_width = 0.1
x = np.arange(n_statistics)

# Define colours
colours = ['#1f77b4', '#2ca02c', '#d62728', '#9467bd', '#c99c5a', '#17becf']

# Plotting
plt.figure(figsize=(12, 8))

# Creating horizontal bars for each statistic with a unique color and white edge
for i in range(n_features):
    plt.barh(x + i * bar_width, data[i], height=bar_width, 
             label=labels[i], color=colours[i % n_statistics], edgecolor='white')

# Adding labels and title
plt.xlabel('RPE (m)')  # Change to desired label
plt.yticks(x + bar_width * (n_features - 1) / 2, statistics)
plt.legend()
plt.grid(axis='x')

# Show the plot
plt.tight_layout()
plt.show()
