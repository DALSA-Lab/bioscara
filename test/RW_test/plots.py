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
binwidth=1
plt.hist(df['read [us]'], bins=range(int(min(df['read [us]'])), int(max(df['read [us]']))+binwidth, binwidth),label=f'read (mean: {df['read [us]'].mean():.2f} max: {df['read [us]'].max():.2f})',histtype="step")
plt.hist(df['write [us]'], bins=range(int(min(df['write [us]'])), int(max(df['write [us]']))+binwidth, binwidth),label=f'write (mean: {df['write [us]'].mean():.2f} max: {df['write [us]'].max():.2f})',histtype="step")

plt.ylabel(r'Samples')
plt.xlabel(r'$t_{total}$ [$\mu $s]')
plt.legend(loc='upper center', bbox_to_anchor=(0.4, -0.3),
          fancybox=True, ncol=1)


plt.tight_layout(h_pad=0.3)
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()
