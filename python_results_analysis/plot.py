import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

# Read the CSV file
df = pd.read_csv('resultsSnapCont.csv')

# Create the figure and 3D axes
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Create a surface plot
surf = ax.plot_trisurf(df['TargetX'], df['TargetY'], df['Time'],
                      cmap=cm.viridis,
                      linewidth=0.2,
                      antialiased=True)

# Customize the plot
ax.set_xlabel('Target X')
ax.set_ylabel('Target Y')
ax.set_zlabel('Time (seconds)')
ax.set_title('3D Surface Plot of Target Positions and Time')

# Add a color bar
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# Adjust the viewing angle for better visualization
ax.view_init(elev=30, azim=45)

# Save the plot
plt.savefig('3d_surface_plot.png', dpi=300, bbox_inches='tight')

# Show the plot
plt.show()