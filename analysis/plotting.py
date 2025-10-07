# import pandas as pd
# import matplotlib.pyplot as plt

# # Read the CSV files with correct names
# reward_df = pd.read_csv('reward_data.csv')
# distance_df = pd.read_csv('distance_data.csv')

# # Create a figure with two subplots (one above the other)
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# # Plot 1: Time vs Reward (convert to numpy arrays with .values)
# ax1.plot(reward_df['time'].values, reward_df['reward'].values, 'b-', linewidth=2)
# ax1.set_xlabel('Time (s)', fontsize=12)
# ax1.set_ylabel('Reward', fontsize=12)
# ax1.set_title('Reward over Time', fontsize=14)
# ax1.grid(True, alpha=0.3)

# # Plot 2: Time vs Distance (convert to numpy arrays with .values)
# ax2.plot(distance_df['time'].values, distance_df['distance'].values, 'r-', linewidth=2)
# ax2.set_xlabel('Time (s)', fontsize=12)
# ax2.set_ylabel('Distance (m)', fontsize=12)
# ax2.set_title('Cumulative Distance over Time', fontsize=14)
# ax2.grid(True, alpha=0.3)

# # Adjust spacing between subplots
# plt.tight_layout()

# # Save the figure
# plt.savefig('results.png', dpi=300, bbox_inches='tight')

# # Display the plot
# plt.show()

# print("Plot saved as 'results.png'")

import pandas as pd
import matplotlib.pyplot as plt

reward_df = pd.read_csv('reward_data.csv')
distance_df = pd.read_csv('distance_data.csv')

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Step plot for reward
ax1.step(reward_df['time'].values, reward_df['reward'].values, 'b-', 
         linewidth=2, where='post')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Cumulative Reward')
ax1.set_title('Reward over Time')
ax1.grid(True, alpha=0.3)

# Regular plot for distance
ax2.plot(distance_df['time'].values, distance_df['distance'].values, 'r-', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cumulative Distance (m)')
ax2.set_title('Distance over Time')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('results.png', dpi=300)
plt.show()