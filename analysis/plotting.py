import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# Get list of method names from command line arguments
# Usage: python plotting.py CBBA CBGA Greedy
methods = sys.argv[1:] if len(sys.argv) > 1 else []

# If no methods specified, check which files exist
if not methods:
    if os.path.exists('reward_data.csv'):
        methods = ['Default']
    else:
        print("No data files found!")
        sys.exit(1)

# Define colors for each method
colors = {
    'CBBA': 'blue',
    'CBGA': 'green', 
    'Greedy': 'red',
    'Default': 'black'
}

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot reward for each method
for method in methods:
    filename = f'reward_data_{method}.csv' if method != 'Default' else 'reward_data.csv'
    
    if os.path.exists(filename):
        df = pd.read_csv(filename)
        time_with_zero = [0] + df['time'].tolist()
        reward_with_zero = [0] + df['reward'].tolist()
        
        ax1.step(time_with_zero, reward_with_zero, 
                 color=colors.get(method, 'black'),
                 linewidth=2, where='post', label=method)

ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Cumulative Reward')
ax1.set_title('Cumulative Reward Achieved by Team')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot score for each method
for method in methods:
    filename = f'score_data_{method}.csv' if method != 'Default' else 'score_data.csv'
    
    if os.path.exists(filename):
        df = pd.read_csv(filename)
        time_with_zero = [0] + df['time'].tolist()
        score_with_zero = [0] + df['score'].tolist()
        
        ax2.step(time_with_zero, score_with_zero,
                 color=colors.get(method, 'black'),
                 linewidth=2, where='post', label=method)

ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cumulative Task Allocation Score')
ax2.set_title('Cumulative Task Allocation Score Achieved by Team')
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.savefig('results.png', dpi=300)
plt.show()