import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

file_path = '/home/stanley/vnav_ws/src/lab6/log/rpe_3pt.csv'

if os.path.exists(file_path):
    print(f"Loading 3D-3D results from {file_path}...")
    df = pd.read_csv(file_path)
else:
    print(f"[ERROR] File not found: {file_path}")
    print("Please make sure you ran the simulation with pose_estimator=3")
    exit()

fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

frames = df['frame'].to_numpy()
trans_err = df['trans_error'].to_numpy()
rot_err = df['rot_error_deg'].to_numpy()

# --- 绘制平移误差 (Translation Error) ---
# 注意：对于 3D-3D，这里的单位是 米 (Meters)
axs[0].plot(frames, trans_err, label='3-Point Arun (3D-3D)', color='purple', alpha=0.8, linewidth=1.5)

axs[0].set_title('Arun\'s 3-Point Method: Translation Error', fontsize=14)
axs[0].set_ylabel('Error (Meters)', fontsize=12) # 这里的单位是米
axs[0].grid(True, which='both', linestyle='--', alpha=0.7)
axs[0].legend(
    loc='upper right',
    bbox_to_anchor=(1.0, 1.0)
)

# 设置 Y 轴范围：通常 3D-3D 误差在 0.05m ~ 0.5m 之间
# 如果误差偶尔飙高，可以适当调大这个上限
axs[0].set_ylim(0, 0.5) 

# --- 绘制旋转误差 (Rotation Error) ---
axs[1].plot(frames, rot_err, label='3-Point Arun (3D-3D)', color='purple', alpha=0.8, linewidth=1.5)

axs[1].set_title('Arun\'s 3-Point Method: Rotation Error', fontsize=14)
axs[1].set_ylabel('Error (Degrees)', fontsize=12)
axs[1].set_xlabel('Frame Index', fontsize=12)
axs[1].grid(True, which='both', linestyle='--', alpha=0.7)
axs[1].legend(
    loc='upper right',
    bbox_to_anchor=(1.0, 1.0)
)
axs[1].set_ylim(0, 5.0) # 3D-3D 的旋转通常非常准，< 1度

plt.tight_layout()
plt.savefig('arun_3d.png', dpi=1200)
plt.show()