import matplotlib.pyplot as plt
import pandas as pd
import sys

plt.style.use('../thesis_style.mplstyle')

if len(sys.argv[1:]) != 1:
    print("Specify a file name")
    quit()

base_file = ".".join(sys.argv[1].split(".")[0:-1])
df = pd.read_csv(sys.argv[1],skipinitialspace=True)



ax1 = plt.subplot(211)
x = [t-df['__time'][0] for t in df['__time']]
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'],label="J1")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/positions[1]'],label="J2")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/positions[2]'],label="J3")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/positions[3]'],label="J4")

plt.ylabel(r'$q$')# [rad]/[m]
plt.tick_params('x', labelbottom=False)

ax2 = plt.subplot(212, sharex=ax1)
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'],label="J1")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/velocities[1]'],label="J2")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/velocities[2]'],label="J3")
plt.plot(x,df['/velocity_joint_trajectory_controller/controller_state/reference/velocities[3]'],label="J4")


plt.xlabel(r'$t$ [s]')
plt.ylabel(r'$\dot{q}$') #[$\text{rad}\text{ s}^{-1}$]/[$\text{m}\text{ s}^{-1}$]
ax1.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=4)
plt.tight_layout()
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()
