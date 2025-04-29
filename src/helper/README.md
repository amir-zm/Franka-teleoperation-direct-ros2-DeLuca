# helper

This folder contains low-level helper libraries used by the `franka_teleoperation` nodes to perform common math and data‐conversion tasks.  Each header implements a single responsibility; together they keep the main control loops clean and focused.

---

## 📂 File overview

| File                                             | Purpose                                                             |
| ------------------------------------------------ | ------------------------------------------------------------------- |
| **convertArrayToEigenMatrix.hpp**                | Map raw C‐arrays (e.g. `double[16]`) into fixed‐size Eigen matrices |
| **convertArrayToEigenVector.hpp**                | Map raw C‐arrays (e.g. `double[7]`) into fixed‐size Eigen vectors   |
| **coriolisTimesDqVector.hpp**                    | Compute the Coriolis/centrifugal term \(C(q,\dot q)\dot q\)         |
| **jacobianMatrix.hpp**                           | Compute the 6×7 geometric Jacobian (position+orientation)           |
| **orientationErrorByPoses.hpp**                  | Compute orientation error axis & angle from two homogeneous poses   |
| **posErrorByPoses.hpp**                          | Compute Cartesian position error between two poses                 |
| **rotationErUnifiedAngleAxis.hpp**               | Convert rotation error to unified angle‐axis vector                |
| **jointTorquesSent.hpp**                         | Pack/unpack `std_msgs::msg::Float64MultiArray` for torque commands  |
| **localCalculatedTorques.hpp**                   | Helper to package local impedance‐control torques for publishing    |
| **remoteCalculatedTorques.hpp**                  | Helper to unpack received torques and apply them to the robot       |
| **FrankaLocal.hpp / .cpp**                       | `FrankaLocal` class: real‐time loop for the master (local) side     |
| **FrankaRemote.hpp / .cpp**                      | `FrankaRemote` class: real‐time loop for the slave (remote) side    |

---

## 🔍 Detailed descriptions

### 1. convertArrayToEigenMatrix.hpp  
- **Function**:  
  ```cpp
  template <size_t R, size_t C>
  Eigen::Matrix<double,R,C> toEigenMatrix(const std::array<double,R*C>& data);
