import matplotlib.pyplot as plt
import pandas as pd
import os

path_ransac = '/home/stanley/vnav_ws/src/lab6/log/rpe_with_ransac.csv'
path_no_ransac = '/home/stanley/vnav_ws/src/lab6/log/rpe_with_no_ransac.csv'

df_ransac = pd.read_csv(path_ransac) if os.path.exists(path_ransac) else None
df_no_ransac = pd.read_csv(path_no_ransac) if os.path.exists(path_no_ransac) else None

fig, axs = plt.subplots(2, 1, figsize=(12, 10))

# --- 1. 绘制平移误差 ---
if df_ransac is not None:
    axs[0].plot(df_ransac['frame'].to_numpy(), df_ransac['trans_error'].to_numpy(), label='With RANSAC', color='blue', alpha=0.7)

if df_no_ransac is not None:
    axs[0].plot(df_no_ransac['frame'].to_numpy(), df_no_ransac['trans_error'].to_numpy(), label='Without RANSAC', color='red', alpha=0.6)

axs[0].set_title('Relative Translation Error (Direction)')
axs[0].set_ylabel('Error (Euclidean dist)')
axs[0].set_xlabel('Frame Index')
axs[0].legend(
    loc='upper right', 
    bbox_to_anchor=(1.0, 1.0)
)
axs[0].grid(True)
axs[0].set_ylim(0, 1.0) 

# --- 2. 绘制旋转误差 ---
if df_ransac is not None:
    axs[1].plot(df_ransac['frame'].to_numpy(), df_ransac['rot_error_deg'].to_numpy(), label='With RANSAC', color='blue', alpha=0.7)

if df_no_ransac is not None:
    axs[1].plot(df_no_ransac['frame'].to_numpy(), df_no_ransac['rot_error_deg'].to_numpy(), label='Without RANSAC', color='red', alpha=0.6)

axs[1].set_title('Relative Rotation Error')
axs[1].set_ylabel('Error (Degrees)')
axs[1].set_xlabel('Frame Index')
axs[1].legend(
    loc='upper right', 
    bbox_to_anchor=(1.0, 1.0)
)
axs[1].grid(True)
axs[1].set_ylim(0, 10)

plt.tight_layout()
plt.savefig('rpe_comparison.png', dpi=1200) 
plt.show()