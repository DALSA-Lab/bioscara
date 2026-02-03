import matplotlib.pyplot as plt
import pandas as pd
import sys
plt.style.use('../thesis_style.mplstyle')

# Load data
high_speed = pd.read_csv("j4_HIGH_SPEED_STALL_adaptive_stall_threshold_two_levels.csv",skipinitialspace=True)
low_speed = pd.read_csv("j4_LOW_SPEED_STALL_adaptive_stall_threshold_two_levels.csv",skipinitialspace=True)


# Plot
plt.plot(high_speed['qd [rad/s]'],high_speed['threshold'],label=f'TH')
plt.plot(high_speed['qd [rad/s]'],high_speed['abs(pid-error) spikes removed LP filtered'],label=f'ER (HS)')
plt.plot(low_speed['qd [rad/s]'],low_speed['abs(pid-error) spikes removed LP filtered'],label=f'ER (LS)')

plt.ylabel(r'$\mu$-steps')
plt.xlabel(r'$\dot{\alpha}$ [rad/s]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)


plt.tight_layout()
plt.savefig(f"j4_HIGH_LOW_stall_comp.pdf", bbox_inches='tight')
plt.savefig(f"j4_HIGH_LOW_stall_comp.png", bbox_inches='tight')
plt.show()

plt.figure()
x_high = [t-high_speed['Program Time [s]'][0] for t in high_speed['Program Time [s]']]
x_low = [t-low_speed['Program Time [s]'][0] for t in low_speed['Program Time [s]']]

plt.plot(x_high,high_speed['threshold'],label=f'TH')
plt.plot(x_high,high_speed['abs(pid-error) spikes removed LP filtered'],label=f'ER (HS)')
plt.plot(x_low,low_speed['abs(pid-error) spikes removed LP filtered'],label=f'ER (LS)')

plt.ylabel(r'$\mu$-steps')
plt.xlabel(r'$t$ [s]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)


plt.tight_layout(h_pad=0.3)
plt.savefig(f"j4_HIGH_LOW_stall_comp_ts.pdf", bbox_inches='tight')
plt.savefig(f"j4_HIGH_LOW_stall_comp_ts.png", bbox_inches='tight')
plt.show()