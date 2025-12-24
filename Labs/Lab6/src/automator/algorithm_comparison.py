import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

base_path = '/home/stanley/vnav_ws/src/lab6/log/'
files = {
    '5-Point (Nister)': os.path.join(base_path, 'rpe_5pt.csv'),
    '8-Point (Longuet-Higgins)': os.path.join(base_path, 'rpe_8pt.csv'),
    '2-Point (Known Rotation)': os.path.join(base_path, 'rpe_2pt.csv')
}

colors = {
    '5-Point (Nister)': 'blue',
    '8-Point (Longuet-Higgins)': 'green',
    '2-Point (Known Rotation)': 'orange'
}

data_frames = {}
for name, path in files.items():
    if os.path.exists(path):
        print(f"Loading {name} from {path}...")
        data_frames[name] = pd.read_csv(path)
    else:
        print(f"[WARNING] File not found: {path}")

if not data_frames:
    print("No data found! Please check file paths.")
    exit()

fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# --- 1. 绘制平移误差 (Translation Error) ---
for name, df in data_frames.items():
    frames = df['frame'].to_numpy()
    trans_err = df['trans_error'].to_numpy()
    
    axs[0].plot(frames, trans_err, label=name, color=colors[name], alpha=0.7, linewidth=1.5)

axs[0].set_title('Relative Translation Error Comparison', fontsize=14)
axs[0].set_ylabel('Error (Euclidean Dist)', fontsize=12)
axs[0].grid(True, which='both', linestyle='--', alpha=0.7)
axs[0].legend(
    loc='upper right',
    bbox_to_anchor=(1.0, 1.0)
)
axs[0].set_ylim(0, 1.0) 

# --- 2. 绘制旋转误差 (Rotation Error) ---
for name, df in data_frames.items():
    frames = df['frame'].to_numpy()
    rot_err = df['rot_error_deg'].to_numpy()
    
    axs[1].plot(frames, rot_err, label=name, color=colors[name], alpha=0.7, linewidth=1.5)

axs[1].set_title('Relative Rotation Error Comparison', fontsize=14)
axs[1].set_ylabel('Error (Degrees)', fontsize=12)
axs[1].set_xlabel('Frame Index', fontsize=12)
axs[1].grid(True, which='both', linestyle='--', alpha=0.7)
axs[1].legend(
    loc='upper right',
    bbox_to_anchor=(1.0, 1.0)
)
axs[1].set_ylim(0, 5.0)

plt.tight_layout()

output_file = 'algorithm_comparison.png'
plt.savefig(output_file, dpi=1200)
plt.show()