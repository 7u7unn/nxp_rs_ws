# nxp_rs_moveit_config

Konfigurasi MoveIt 2 untuk robot NXP RS. Paket ini menyediakan dua alur yang
terpisah:

- `demo.launch.py`: MoveIt dengan `mock_components/GenericSystem`, tanpa CAN dan
  tanpa motor fisik;
- `hardware_moveit.launch.py`: MoveIt dengan plugin hardware
  `rs_control/RobstrideSystem` melalui SocketCAN.

`arm` adalah chain enam joint dari `base_link` ke `grasp_frame`. Gripper hanya
menggerakkan `joint_right-finger`; `joint_left-finger` adalah joint mimic.
`grasp_frame` berada di tengah kedua finger dan menjadi tip arm.

## 1. Requirement dan instalasi

### Platform dan dependency

- Ubuntu 22.04
- ROS 2 Humble
- Workspace ini (`/home/jundi/nxp_rs_ws`)
- Untuk demo: MoveIt 2, `ros2_control`, controller manager, RViz, dan Xacro
- Untuk hardware: seluruh dependency demo ditambah `rs_control`, SocketCAN, dan
  konfigurasi motor yang sudah dikalibrasi
- Untuk scene inspeksi inertia Gazebo: `gazebo_ros` (sudah dicakup oleh
  `rosdep` dari package `nxp_rs_description`)

Jika ROS 2 Humble belum terpasang, ikuti [panduan instalasi resmi untuk Ubuntu
22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
terlebih dahulu.

Dependency runtime dan test tercantum di `package.xml`. Jangan memasang library
Python MoveIt secara manual dengan `pip`; gunakan paket ROS dan `rosdep`.

Runtime utama yang dipasang oleh `rosdep` mencakup `moveit_msgs`,
`moveit_configs_utils`, `moveit_ros_move_group`, `moveit_ros_visualization`,
`moveit_simple_controller_manager`, `ros2_control`, `controller_manager`,
`joint_state_broadcaster`, `joint_trajectory_controller`, `rviz2`, `tf2_ros`,
`geometry_msgs`, `shape_msgs`, `rclcpp`, `xacro`, `robot_state_publisher`,
serta dependency `nxp_rs_description` (`joint_state_publisher_gui` dan
`gazebo_ros`) dan `rs_control`. Dependency lint/test juga diambil dari
`package.xml` saat `rosdep install` dijalankan.

### Pasang tool sistem

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest \
  can-utils
```

`can-utils` hanya dipakai untuk diagnostik CAN hardware. Jika `rosdep` belum
pernah diinisialisasi:

```bash
sudo rosdep init
rosdep update
```

Lewati `rosdep init` jika sudah pernah berhasil.

### Pasang dependency ROS dan build

```bash
cd /home/jundi/nxp_rs_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to nxp_rs_moveit_config
source install/setup.bash
```

`rosdep install` memasang dependency eksternal MoveIt (`moveit_msgs`,
`moveit_configs_utils`, `move_group`, RViz, dan controller manager),
`ros2_control`, controller trajectory, serta dependency `nxp_rs_description`
dan `rs_control`. Kedua paket lokal tersebut kemudian dibangun oleh `colcon`.

Di terminal baru, source ulang:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
```

## 2. Model dan konfigurasi

| Item | Lokasi / nilai |
| --- | --- |
| URDF + ros2_control simulasi | [`config/nxp_rs_moveit.urdf.xacro`](config/nxp_rs_moveit.urdf.xacro) |
| SRDF group | [`config/nxp_rs.srdf`](config/nxp_rs.srdf) |
| Batas joint MoveIt | [`config/joint_limits.yaml`](config/joint_limits.yaml) |
| Kinematika | [`config/kinematics.yaml`](config/kinematics.yaml) |
| Controller MoveIt | [`config/moveit_controllers.yaml`](config/moveit_controllers.yaml) |
| Scene lantai | [`config/floor_scene.yaml`](config/floor_scene.yaml) |
| Overlay tolerance hardware (opsional) | [`config/hardware_execution.yaml`](config/hardware_execution.yaml) |

Scene menambahkan collision box `nxp_rs_floor` dengan permukaan atas di `z=0`
frame `world`. Hanya kontak tetap `base_link`/floor yang diizinkan oleh scene;
link bergerak tetap harus bebas collision.

Setelah mengubah URDF, SRDF, mesh, atau YAML, hentikan dan jalankan ulang semua
node yang membaca model. Proses yang sedang berjalan menyimpan model lama.

## 3. Demo MoveIt tanpa hardware

Demo ini memakai mock ros2_control. Ia cocok untuk memeriksa planning group,
kinematika, collision scene, dan integrasi controller, tetapi bukan simulasi
fisika motor dan tidak membuka SocketCAN.

Dengan RViz:

```bash
ros2 launch nxp_rs_moveit_config demo.launch.py
```

Headless smoke test:

```bash
ros2 launch nxp_rs_moveit_config demo.launch.py rviz:=false
```

Di RViz, pilih group `arm`, gunakan start state aktual dari model, tetapkan
target pose/joint, lalu **Plan** dan **Execute**. Eksekusi hanya dikirim ke
`mock_components/GenericSystem`; tidak ada gerakan motor fisik.

## 4. Hardware MoveIt

### Preflight keselamatan

Sebelum mengaktifkan torque:

- hentikan semua `rs_bringup`, Python controller, jogger, MoveIt hardware, dan
  launch fisik lain;
- pastikan hanya ada satu `/controller_manager`, satu
  `/joint_state_broadcaster`, dan satu `robot_state_publisher`;
- dukung arm secara mekanis, kosongkan area gerak, dan siapkan emergency stop;
- verifikasi CAN, ID, arah, offset, limit, serta posisi awal aktual.

Jangan memakai `demo.launch.py` bersamaan dengan hardware launch dalam ROS domain
yang sama karena nama controller dan topic bertabrakan.

### Periksa SocketCAN

Launch tidak mengatur bitrate secara otomatis. Periksa dan, bila aman, aktifkan
interface:

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
```

Konfigurasi motor bawaan adalah
[`rs_control/config/robstride.yaml`](../rs_control/config/robstride.yaml). Gunakan
`config_file:=/absolute/path/to/robstride.yaml` jika deployment memakai file lain.

### Mulai read-only

Mulai dengan torque dan controller gerak nonaktif:

```bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  can_interface:=can0 \
  read_only:=true \
  enable_control:=false \
  rviz:=true
```

Verifikasi dari terminal lain:

```bash
ros2 node list
ros2 control list_hardware_components --verbose
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

Sebelum lanjut, `robstride_system` harus `active`, state interface harus
`available`, posisi joint harus stabil, dan RViz harus sama dengan pose fisik.
Di panel **MotionPlanning**, gunakan start state **Current**. Lakukan **Plan**
saja pada tahap ini; jangan **Execute**.

### Aktifkan kontrol posisi

Setelah preflight dan read-only berhasil, hentikan launch dengan `Ctrl+C`,
tunggu semua node keluar, lalu mulai satu instance baru:

```bash
ros2 launch nxp_rs_moveit_config hardware_moveit.launch.py \
  can_interface:=can0 \
  read_only:=false \
  enable_control:=true \
  rviz:=true
```

`read_only:=false` mengaktifkan motor saat hardware diaktifkan. `enable_control`
hanya menentukan apakah `arm_controller` dan `gripper_controller` di-spawn;
keduanya harus `true` untuk menerima trajectory.

Periksa controller sebelum command pertama:

```bash
ros2 control list_hardware_components --verbose
ros2 control list_controllers
```

Controller yang diharapkan adalah `joint_state_broadcaster`, `arm_controller`,
dan `gripper_controller`, semuanya `active`.

### Gerakan pertama

Gunakan satu joint dengan target hanya `0.01`--`0.02 rad` dari posisi yang
terukur, gunakan velocity/acceleration scaling terendah, periksa trajectory di
RViz, lalu **Execute**. Setelah setiap eksekusi periksa feedback:

```bash
ros2 topic echo --once /arm_controller/controller_state
ros2 topic echo --once /joint_states
```

Hentikan segera dengan emergency stop jika robot menarik ke pose yang salah,
bergetar, bergerak terlalu jauh, atau CAN keluar dari `ERROR-ACTIVE`. Setelah
uji selesai, stop gerak dan shutdown launch dengan `Ctrl+C` agar hardware
dinonaktifkan.

Launch hardware tidak memuat overlay tolerance ketat secara default. Gunakan
[`config/hardware_execution.yaml`](config/hardware_execution.yaml) hanya dalam
commissioning terkontrol setelah tuning; tolerance tidak memperbaiki gain,
tracking, atau collision safety.

## 5. Troubleshooting singkat

- **Controller gagal di-spawn:** pastikan tidak ada launch fisik/simulasi lain
  yang masih berjalan.
- **Start point berbeda dari robot:** tunggu feedback stabil, pilih start state
  **Current**, lalu plan ulang.
- **`PATH_TOLERANCE_VIOLATED` atau `GOAL_TOLERANCE_VIOLATED`:** bandingkan
  `desired`, `actual`, dan `error`; selidiki calibration, gain, beban, dan
  mekanik sebelum mengubah tolerance.
- **Collision aneh pada base/finger:** rebuild description dan MoveIt, restart
  semua node, lalu periksa mesh, frame, arah joint, dan clearance fisik.
- **Finger di luar rentang `0.0 m` sampai `0.0837560613 m`:** periksa
  kembali zero dan pemetaan finger; jangan memperlebar limit untuk menutupi
  error kalibrasi.

Laporan investigasi detail tersedia di
[`docs/execution_investigation_2026-09-10.md`](docs/execution_investigation_2026-09-10.md).

## 6. Testing

Test berikut tidak memerlukan CAN atau motor hidup:

```bash
cd /home/jundi/nxp_rs_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select nxp_rs_moveit_config
colcon test-result \
  --test-result-base build/nxp_rs_moveit_config --verbose
```
