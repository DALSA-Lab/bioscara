import matplotlib.pyplot as plt
import pandas as pd
import sys
plt.style.use('../thesis_style.mplstyle')

if len(sys.argv[1:]) != 1:
    print("Specify a file name")
    quit()

# Load data
base_file = ".".join(sys.argv[1].split(".")[0:-1])
df = pd.read_csv(sys.argv[1],skipinitialspace=True)


# Plot
plt.plot(df['qd [rad/s]'],df['threshold'],label=f'TH')
plt.plot(df['qd [rad/s]'],df['abs(pid-error) spikes removed LP filtered'],label=f'ER')

plt.ylabel(r'$\mu$-steps')
plt.xlabel(r'$\dot{\alpha}$ [rad/s]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=2)


plt.tight_layout()
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()

plt.figure()
x = [t-df['Program Time [s]'][0] for t in df['Program Time [s]']]
plt.plot(x,df['threshold'],label=f'TH')
plt.plot(x,df['abs(pid-error) spikes removed LP filtered'],label=f'ER')

plt.ylabel(r'$\mu$-steps')
plt.xlabel(r'$t$ [s]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=2)


plt.tight_layout()
plt.savefig(f"{base_file}_ts.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}_ts.png", bbox_inches='tight')
plt.show()