import matplotlib.pyplot as plt
import pandas as pd
import sys
import yaml
plt.style.use('../thesis_style.mplstyle')

if len(sys.argv[1:]) != 1:
    print("Specify a file name")
    quit()

# Load data
base_file = ".".join(sys.argv[1].split(".")[0:-1])
# df = pd.read_csv(sys.argv[1],skipinitialspace=True)

f = open(sys.argv[1], "r")
topic_echo_file = f.read()
f.close()

# with open(sys.argv[1], 'r') as file:
loaded_data = yaml.safe_load_all(topic_echo_file)
topic_list = list(loaded_data)

traj_points = topic_list[0]["trajectory"][0]["joint_trajectory"]["points"]
joint_names = topic_list[0]["trajectory"][0]["joint_trajectory"]["joint_names"]


# AI START
out = {"position": [], "velocity": [], "acceleration": [], "time": []}
for p in traj_points:
    try:
        out["position"].append(p.get("positions", []))
        out["velocity"].append(p.get("velocities", []))
        out["acceleration"].append(p.get("accelerations", []))
        t = p.get("time_from_start", {})
        sec = t.get("sec", 0) or 0
        nsec = t.get("nanosec", 0) or 0
        out["time"].append(float(sec) + float(nsec) * 1e-9)
# AI STOP
    except AttributeError:
        print("Failed to extract position, velocity, acceleration and time from point: ")
        print(p)
        print("The recorded trajectory is most likely too long and parts of the 'points' array have been truncated")


# Plot
fig, ax = plt.subplots(3, 1,sharex="col")
ax[0].plot(out["time"],out["position"], label=joint_names) #, marker='o', markersize=2)
ax[0].set_ylabel("$q$ [rad]")
ax[1].plot(out["time"],out["velocity"], label=joint_names) #, marker='o', markersize=2)
ax[1].set_ylabel(r'$\dot{q}$ [rad/s]')
ax[2].plot(out["time"],out["acceleration"], label=joint_names) #, marker='o', markersize=2)
ax[2].set_ylabel(r'$\ddot{q}$ [$\mathrm{rad}/s^2$]')


plt.xlabel(r'$t$ [s]')
ax[0].legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=len(joint_names))

fig.align_ylabels()
fig.align_xlabels()
plt.tight_layout()
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()
