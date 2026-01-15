**sample1:**

i2cget -y 1 0x11 0x0f

"main" firmware (no test) at https://github.com/DALSA-Lab/bioscara/blob/bce66ca3f7d215edcba7416d0361210beb87d8f3/Arduino/joint/joint.ino

using i2c-gpio overlay (hardware i2c disabled):

```plain
dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2
```

Only j1 connected

Observations:

- Yellow (chB): SDA, Blue (chA): SCL
- Visible crosstalk on every large transition

- AND: clock stretching (is it really? Check i2c specs)

- Time divisions are not correct. Measured on osci is 6 us period
- short SDA spike:
  - ACK after address?
  - COntroller releases SDA and waits for answer?
  - Has been osbserved multiple times (not a glitch)

**Sample 2**

same firmware and data request. This time 

```
dtparam=i2c_arm=on,i2c_arm_baudrate=100000
```

hardware i2c

 Observations:

- Same cross coupling obvs.
- slower