import pandas as pd
import matplotlib.pyplot as plt

reward_df = pd.read_csv('reward_data.csv')
discounted_reward_df = pd.read_csv('discounted_reward_data.csv')
distance_df = pd.read_csv('distance_data.csv')

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

# Step plot for raw reward
time_with_zero = [0] + reward_df['time'].tolist()
reward_with_zero = [0] + reward_df['reward'].tolist()
ax1.step(time_with_zero, reward_with_zero, 'b-', 
         linewidth=2, where='post')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Cumulative Reward')
ax1.set_title('Cumulative Reward Achieved by Team')
ax1.grid(True, alpha=0.3)

# Step plot for discounted reward
time_with_zero_disc = [0] + discounted_reward_df['time'].tolist()
discounted_with_zero = [0] + discounted_reward_df['discounted_reward'].tolist()
ax2.step(time_with_zero_disc, discounted_with_zero, 'g-', 
         linewidth=2, where='post')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cumulative Discounted Reward')
ax2.set_title('Cumulative Discounted Reward Achieved by Team')
ax2.grid(True, alpha=0.3)

# Regular plot for distance
ax3.plot(distance_df['time'].values, distance_df['distance'].values, 'r-', linewidth=2)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Cumulative Distance (px)')
ax3.set_title('Cumulative Team Distance')
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('results.png', dpi=300)
plt.show()