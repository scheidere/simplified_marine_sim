import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import glob
import random

# Auto-detect all reward and score files
reward_files = glob.glob('reward_data*.csv')
score_files = glob.glob('score_data*.csv')

if not reward_files and not score_files:
    print("No data files found!")
    sys.exit(1)

# Extract method names from filenames
methods = set()
for f in reward_files + score_files:
    # Remove 'reward_data_' or 'score_data_' prefix and '.csv' suffix
    if f.startswith('reward_data_'):
        method = f.replace('reward_data_', '').replace('.csv', '')
        methods.add(method)
    elif f.startswith('score_data_'):
        method = f.replace('score_data_', '').replace('.csv', '')
        methods.add(method)
    elif f in ['reward_data.csv', 'score_data.csv']:
        methods.add('Default')

methods = sorted(methods)
print(f"Found methods: {methods}")

# Generate random colors for each method
random.seed(42)  # For reproducible colors across runs
colors = {}
for method in methods:
    colors[method] = (random.random(), random.random(), random.random())

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot reward for each method
for method in methods:
    if method == 'Default':
        filename = 'reward_data.csv'
    else:
        filename = f'reward_data_{method}.csv'
    
    if os.path.exists(filename):
        df = pd.read_csv(filename)
        time_with_zero = [0] + df['time'].tolist()
        reward_with_zero = [0] + df['reward'].tolist()
        
        ax1.step(time_with_zero, reward_with_zero, 
                 color=colors[method],
                 linewidth=2, where='post', label=method)

ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Cumulative Reward')
ax1.set_title('Cumulative Reward Achieved by Team (Helper Rewards Included)')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot score for each method
for method in methods:
    if method == 'Default':
        filename = 'score_data.csv'
    else:
        filename = f'score_data_{method}.csv'
    
    if os.path.exists(filename):
        df = pd.read_csv(filename)
        time_with_zero = [0] + df['time'].tolist()
        score_with_zero = [0] + df['score'].tolist()
        
        ax2.step(time_with_zero, score_with_zero,
                 color=colors[method],
                 linewidth=2, where='post', label=method)

ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Cumulative Score for Task Completion')
ax2.set_title('Cumulative Task Score Achieved by Team (Helper Rewards Not Included)')
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.savefig('results.png', dpi=300)
plt.show()