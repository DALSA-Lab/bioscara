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
plt.plot(df['SDA'],label="SDA")
plt.plot(df['SCL'], label="SCL")

plt.ylabel(r'Voltage')
plt.xlabel(r'Time')
plt.xticks([], [])
plt.yticks([], [])
# plt.tick_params(
#     axis='both',          # changes apply to the x-axis
#     which='both',      # both major and minor ticks are affected
#     bottom=False,      # ticks along the bottom edge are off
#     top=False,         # ticks along the top edge are off
#     labelbottom=False,
#     left = False) # labels along the bottom edge are off
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1),
          fancybox=True, ncol=2)


plt.tight_layout()
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()
