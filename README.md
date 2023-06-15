# CISCOR Lab Boom Module Project #

| Jonathan Boylan | jboylan@fsu.edu | (786) 566-7230 |

## Directory ##

### **boom_rpi** ###

Information on how to set up the boom module. Includes Raspberry Pi setup scripts, electrical wiring, and other configuration instructions.

### **matlab** ###

Contains BoomController class and examples of how to control the boom from MATLAB.

### **boom_encoders** ###

ROS2 Python package that reads the encoder data (from Teensy via Serial), and publishes it to ROS.

#### Params ####
| Name | Description | Default |
| ---- | ----------- | ------- |
| `serial_port` | Serial port used for communication with the Teensy | `'/dev/ttyACM0'` |
| `baud_rate` | Baud rate used in serial communication *(Needs to match CAN setup baud rate)* | `500000` |
| `publish_rate` | Publish rate of encoder information | `100` |
| `base_cycles_per_revolution` | Reduction from steps to revolutions for the base encoder | `161792` |
| `arm_cycles_per_revolution` | Reduction for the arm encoder | `!TODO` |

#### Publishers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/boom/orientation` | Position of the arm around the base | Float32 | Rev |
| `/boom/tilt` | Position of the arm with respect to ground | Float32 | Rev |

### **motor_driver** ### 

ROS2 Python package that communicates with the motors via CAN.

#### Config YAML ####
```
motor-name:
    id: <int> (0-M motors)
    control_mode: <int> (1-3)
    gear_ratio: <double>
    can_id: <int> (0-63)
```

##### Control Modes #####
1. Torque Control
2. Velocity Control *(+ FF Torque)*
3. Position Control *(+ FF Torque, Velocity)*

#### Subscribers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/motors/cmd` | Motor set position command | JointTrajectoryPoint | rev, rev/s, Nm |

*Note: JointTrajectoryPoint is a list of positions, velocities, and efforts (torques). The index of each value in the list corresponds to the id of the motor to actuate.*

#### Publishers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/motors/info` | Motor encoder readings | JointTrajectoryPoint | rev, rev/s, Nm |
| `/motors/voltage` | Motor voltage readings | Float32MultiArray | V |
| `/motors/current` | Motor current readings | Float32MultiArray | A |
| `/motors/temperature` | Motor temperature readings | Float32MultiArray | deg C |

#### Services ####
| Topic | Description |
| ----- | ----------- |
| `/motors/set_zero` | Set current position to zero for all motors |
| `/motors/clear_errors` | Clear errors on all the motors |

### **leg_kinematics** ###

ROS2 C++ Package that uses the leg inverse kinematics to convert the leg position from Cartesian space to Motor space.

#### Config YAML ####
```
leg-name:
  id: <int> (0-N legs)
  motor_ids: <int[]> (from motor_driver config)
  kinematics: <string>
```

##### Kinematic Models #####
| Name | Description |
| ---- | ----------- |
| `fivebar-2d` | 2 DOF Five bar linkage (ETQuad) |
| `fivebar-3d` *(TODO)* | 3 DOF Five bar linkage (STARQ) |
| `rr-2d` *(TODO)* | 2 DOF Revolute-Revolute linkage |

#### Subscribers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/legs/cmd` | Set leg position command | PointCloud | m |
| `/motors/info` | Motor driver encoder readings | JointTrajectoryPoint | rev |

*Note: PointCloud is a list of 3D points. The index of each point in the list corresponds to the id of the leg to actuate.*

#### Publishers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/legs/info` | Forward kinematics applied to the motor encoder readings | PointCloud | m |
| `/motors/cmd` | Command to motor driver | JointTrajectoryPoint | rev |

### **gait_publisher** ###

ROS2 C++ package that reads point trajectories from a file and publishes them as leg commands at a fixed rate.

#### Config YAML ####
```
gait-name:
  file_name: <string>
  publish_rate: <double>
```
*Note: `file_name` is relative to `gait_publisher/gaits/` folder*

#### Subscribers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/gait` | Name of the gait to run | String | - |

#### Publishers ####
| Topic | Description | Type | Units |
| ----- | ----------- | ---- | ----- |
| `/legs/cmd` | Leg position commands | PointCloud | m |

### **docker** ###

Contains scripts and Dockerfiles used for development purposes.