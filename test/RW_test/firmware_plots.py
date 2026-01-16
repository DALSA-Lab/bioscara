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
ANGLEMOVED_val = df[df["command"] == "ANGLEMOVED"]["t_exec [us]"]
MOVETOANGLE_val = df[df["command"] == "MOVETOANGLE"]["t_exec [us]"]

plt.hist(ANGLEMOVED_val, bins=range(int(min(ANGLEMOVED_val)), int(max(ANGLEMOVED_val))+binwidth, binwidth),label=f'read (mean: {ANGLEMOVED_val.mean():.2f} max: {ANGLEMOVED_val.max():.2f})',histtype="step")
plt.hist(MOVETOANGLE_val, bins=range(int(min(MOVETOANGLE_val)), int(max(MOVETOANGLE_val))+binwidth, binwidth),label=f'write (mean: {MOVETOANGLE_val.mean():.2f} max: {MOVETOANGLE_val.max():.2f})',histtype="step")

plt.ylabel(r'Samples')
plt.xlabel(r'$t_{total}$ [$\mu $s]')
plt.legend()


plt.tight_layout()
plt.savefig(f"{base_file}.pdf")
plt.savefig(f"{base_file}.png")
plt.show()
