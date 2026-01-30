import matplotlib.pyplot as plt
import pandas as pd
plt.style.use('../thesis_style.mplstyle')



data_pid = pd.read_csv('record_0p314_0p157_j1_vel_p0p5_i0p0_d0p0_kff1p0.csv')
data_ff = pd.read_csv('record_0p314_0p157_j1_vel_p0p0_i0p0_d0p0_kff1p0.csv')

data_ff.iloc[1454:] = data_ff.iloc[1453]



# Position
plt.figure()
x = [t-data_pid['__time'][0] for t in data_pid['__time']]
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'].shift(-1086),"k--",label="Ref. $r_1$")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'].shift(-1086),label="CL")

x = [t-data_ff['__time'][0] for t in data_ff['__time']]
plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'],label="FF")
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'],label="Ref. $r_1$")

plt.xlim((16,36))
plt.xlabel(r'$t$ [s]')
plt.ylabel(r'$q_1$ [rad]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)
plt.tight_layout()
base_file = "tracking_low_accel_pos"
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()


# Velocity
plt.figure()
x = [t-data_pid['__time'][0] for t in data_pid['__time']]
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'].shift(-1086),"k--",label="Ref. $r_1$")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/velocities[0]'].shift(-1086),label="CL")

x = [t-data_ff['__time'][0] for t in data_ff['__time']]
plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/feedback/velocities[0]'],label="FF")
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'],label="Ref. $r_1$")

plt.xlim((16,36))
plt.ylim(bottom=-0.15)
plt.xlabel(r'$t$ [s]')
plt.ylabel(r'$\dot{q}_1$ [rad]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)
plt.tight_layout()
base_file = "tracking_low_accel_vel"
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()

############## High Accel

data_pid = pd.read_csv('record_0p314_1p57_j1_vel_p0p5_i0p0_d0p0_kff1p0.csv')


# Position
plt.figure()
x = [t-data_pid['__time'][0] for t in data_pid['__time']]
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'].shift(-1086),"k--",label="Ref. $r_1$")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'].shift(-1086),label="CL")

# x = [t-data_ff['__time'][0] for t in data_ff['__time']]
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/feedback/positions[0]'],label="FF")
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/reference/positions[0]'],label="Ref. $r_1$")

plt.xlim((21,27))
plt.ylim((1.5,1.61))
plt.xlabel(r'$t$ [s]')
plt.ylabel(r'$q_1$ [rad]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)
plt.tight_layout()
base_file = "tracking_high_accel_pos"
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()


# Velocity
plt.figure()
x = [t-data_pid['__time'][0] for t in data_pid['__time']]
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'].shift(-1086),"k--",label="Ref. $r_1$")
plt.plot(x,data_pid['/velocity_joint_trajectory_controller/controller_state/feedback/velocities[0]'].shift(-1086),label="CL")

# x = [t-data_ff['__time'][0] for t in data_ff['__time']]
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/feedback/velocities[0]'],label="FF")
# plt.plot(x,data_ff['/velocity_joint_trajectory_controller/controller_state/reference/velocities[0]'],label="Ref. $r_1$")

plt.xlim((21,27))
plt.ylim(bottom=-0.025)
plt.xlabel(r'$t$ [s]')
plt.ylabel(r'$\dot{q}_1$ [rad]')
plt.legend(bbox_to_anchor=(0, 1.03, 1, 0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3)
plt.tight_layout()
base_file = "tracking_high_accel_vel"
plt.savefig(f"{base_file}.pdf", bbox_inches='tight')
plt.savefig(f"{base_file}.png", bbox_inches='tight')
plt.show()