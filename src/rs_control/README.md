# rs_control

`rs_control` adalah driver RobStride untuk robot NXP RS. Paket ini menyediakan
driver SocketCAN C++, plugin `ros2_control`, pembaca encoder, utilitas scan/ubah
ID motor, dan jogger posisi berbasis Python.

Topologi fisik yang didukung adalah enam joint arm dan satu aktuator finger:

| Aktuator | CAN ID | Joint |
| --- | ---: | --- |
| RobStride RS-00 | 1--4 | `joint-1` ... `joint-4` |
| RobStride RS-05 | 5--6 | `joint-5`, `joint-6` |
| RobStride RS-05 | 7 | `joint_right-finger` |
| Mimic (tanpa motor) | -- | `joint_left-finger` |

## 1. Requirement dan instalasi

### Platform

- Ubuntu 22.04
- ROS 2 Humble yang sudah terpasang dan dapat di-source dari
  `/opt/ros/humble/setup.bash`
- Kernel Linux dengan SocketCAN untuk akses hardware

Jika ROS 2 Humble belum terpasang, ikuti [panduan instalasi resmi untuk Ubuntu
22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
terlebih dahulu.

Dependensi ROS, C++, Python, dan test dideklarasikan di `package.xml` dan
dipasang oleh `rosdep`. Tidak perlu `pip install` untuk menjalankan paket ini.

### Pasang tool sistem

Jalankan sekali di mesin pengembangan:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-can \
  python3-yaml \
  python3-pytest \
  can-utils
```

`can-utils` hanya diperlukan untuk diagnostik seperti `candump`; paket runtime
lainnya diambil dari deklarasi ROS package.

Dependency yang dicakup `package.xml` adalah:

- ROS/C++: `diagnostic_msgs`, `hardware_interface`, `pluginlib`, `rclcpp`,
  `rclcpp_lifecycle`, `sensor_msgs`, dan `yaml_cpp_vendor`;
- ROS launch/control: `launch`, `launch_ros`, `ament_index_python`, `rclpy`,
  `trajectory_msgs`, `controller_manager`, `joint_state_broadcaster`,
  `joint_trajectory_controller`, `nxp_rs_description`,
  `robot_state_publisher`, `rviz2`, dan `xacro`;
- Python/test: `python3-can`, `python3-yaml`, `python3-pytest`,
  `ament_cmake_gtest`, dan `ament_cmake_pytest`.

Semua item di atas, kecuali tool tambahan `can-utils`, diproses oleh perintah
`rosdep install` di bawah.

Jika `rosdep` belum pernah diinisialisasi di mesin ini, jalankan sekali:

```bash
sudo rosdep init
rosdep update
```

Jika `rosdep init` sudah pernah berhasil, lewati perintah tersebut dan cukup
jalankan `rosdep update` saat diperlukan.

### Pasang seluruh dependency dan build

```bash
cd /home/jundi/nxp_rs_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to rs_control
source install/setup.bash
```

Perintah `rosdep install` memasang dependency eksternal seperti
`hardware_interface`, `pluginlib`, `rclcpp`, `rclpy`, `controller_manager`,
`joint_trajectory_controller`, `joint_state_broadcaster`,
`robot_state_publisher`, `rviz2`, `xacro`, `python3-can`, dan `python3-yaml`.
Karena build juga menyertakan `nxp_rs_description`, `rosdep` turut menyelesaikan
dependency `joint_state_publisher_gui` dan `gazebo_ros`. Paket lokal tersebut
dibangun bersama `rs_control` oleh `colcon`.

Setiap terminal baru harus menjalankan:

```bash
source /opt/ros/humble/setup.bash
source /home/jundi/nxp_rs_ws/install/setup.bash
```

## 2. Konfigurasi motor

File konfigurasi yang digunakan:

- [`config/robstride.yaml`](config/robstride.yaml): CAN interface, bitrate,
  host ID, CAN ID motor, arah, offset, skala, gain, dan mode `read_only`.
- [`config/joints.yaml`](config/joints.yaml): batas joint dalam koordinat ROS.

Konfigurasi bawaan menggunakan `can0`, bitrate `1000000`, host ID `255`, dan
`read_only: true`. Sebelum mengaktifkan torque, pastikan:

- setiap motor merespons pada CAN ID yang benar dan semua ID unik;
- `host_id` berbeda dari seluruh CAN ID motor;
- arah dan `position_offset` cocok dengan sumbu positif URDF;
- limit joint berada di dalam batas mekanis;
- pemetaan finger diverifikasi tanpa beban. Motor 7 dipetakan ke `0.0 m`
  (open, encoder `2.02584429 rad`) sampai `0.0837560613 m` (closed, encoder
  `-0.04 rad`). Posisi open sebelumnya terbaca `-0.0434560613 m`; posisi
  tersebut sekarang dijadikan zero dengan skala `0.040543259557 m/rad`.
  Jogger menggunakan seluruh rentang terkalibrasi
  secara default; berikan `gripper_limit_margin` positif untuk menjaga jarak
  dari hard stop;
- robot, kabel, dan area gerak aman serta emergency stop siap digunakan.

Edit file sumber sebelum build ulang, atau gunakan file lain secara eksplisit:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  config_file:=/absolute/path/to/robstride.yaml
```

## 3. Siapkan dan scan CAN

Pastikan interface sudah aktif sebelum menjalankan node:

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
```

Scan C++ (tidak mengaktifkan motor):

```bash
ros2 run rs_control rs_scan --interface can0
```

Scan Python dengan rentang ID yang dapat diatur:

```bash
ros2 run rs_control rs_python_scan \
  --interface can0 --start-id 1 --end-id 255
```

## 4. Baca state tanpa mengaktifkan motor

### Encoder reader

```bash
ros2 launch rs_control rs_encoder_bringup.launch.py
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

Node ini hanya membaca encoder dan memublikasikan posisi, kecepatan, effort,
dan temperatur. Ia tidak mengirim frame enable atau motion.

### Python reader/controller

Mode bawaan juga read-only:

```bash
ros2 launch rs_control rs_python_bringup.launch.py
```

Node memublikasikan `/joint_states` dan `/diagnostics`. Mode kontrol Python
menerima `trajectory_msgs/msg/JointTrajectory` di
`/rs_control/joint_trajectory`; aktifkan hanya setelah preflight selesai:

```bash
ros2 launch rs_control rs_python_bringup.launch.py \
  read_only:=false
```

Jika node melaporkan fault, simpan bukti diagnostiknya dengan `candump -tz
can0`, periksa catu daya, kabel, encoder, dan mekanik, lalu selesaikan fault
sebelum mencoba lagi.

## 5. `ros2_control` standard

### Read-only

```bash
ros2 launch rs_control rs_bringup.launch.py
ros2 control list_hardware_components --verbose
ros2 control list_controllers
```

Mode ini membaca state dan hanya menjalankan `joint_state_broadcaster`.

### Aktifkan kontrol posisi

Jalankan hanya setelah limit, arah, offset, CAN, dukungan mekanik, dan emergency
stop diverifikasi:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  read_only:=false enable_control:=true
```

Controller yang diharapkan aktif:

| Controller | Joint | Command |
| --- | --- | --- |
| `joint_state_broadcaster` | seluruh joint | state `/joint_states` |
| `arm_controller` | `joint-1` ... `joint-6` | position trajectory |
| `gripper_controller` | `joint_right-finger` | position trajectory |

Topic command standard adalah `/arm_controller/joint_trajectory` dan
`/gripper_controller/joint_trajectory`. Jalur ini berbeda dari topic Python
`/rs_control/joint_trajectory`.

Jangan menjalankan dua hardware bringup sekaligus. Hentikan encoder reader,
Python controller, MoveIt physical, dan node lain yang dapat mengirim command
sebelum memulai instance baru.

## 6. Integrasi MoveIt dan RL

MoveIt mengirim trajectory ke `arm_controller` dan `gripper_controller` melalui
`FollowJointTrajectory`. Proses RL dapat membaca `/joint_states` dan mengirim
target posisi joint melalui topic controller standard atau topic Python
`/rs_control/joint_trajectory`. Pastikan urutan joint, satuan (rad untuk arm,
meter untuk finger), frame, limit, dan skala action sama antara simulasi dan
hardware. Paket ini tidak menyediakan trainer PPO atau environment RL.

## 7. Ubah CAN ID satu motor

Gunakan utilitas ini ketika hanya satu motor yang terhubung atau diberi daya.
Tanpa `--yes`, utilitas hanya melakukan scan atau dry run:

```bash
# Lihat motor yang merespons
ros2 run rs_control rs_python_set_id --interface can0

# Dry run: rencanakan ID 2 tanpa menulis
ros2 run rs_control rs_python_set_id \
  --interface can0 --new-id 2

# Terapkan perubahan setelah memeriksa hasil dry run
ros2 run rs_control rs_python_set_id \
  --interface can0 --new-id 2 --yes
```

Jika beberapa motor terhubung, berikan `--old-id`. Utilitas menolak ID tujuan
yang sudah terpakai atau sama dengan `host_id`. Power-cycle motor dan scan ulang
setelah perubahan.

## 8. Jog manual

### Mulai hardware dan controller

Terminal 1:

```bash
ros2 launch rs_control rs_bringup.launch.py \
  read_only:=false enable_control:=true
```

Pastikan `joint_state_broadcaster`, `arm_controller`, dan
`gripper_controller` berstatus `active` sebelum melanjutkan.

### Mulai jogger

Terminal 2 harus berupa terminal interaktif:

```bash
ros2 run rs_control rs_jog_controller
```

Atau gunakan launch wrapper:

```bash
ros2 launch rs_control rs_jog.launch.py \
  selected_joint:=joint-4 \
  arm_step:=0.02 \
  command_rate_hz:=20.0 \
  max_arm_velocity:=0.2
```

Argumen lain (limit, acceleration, jerk, timeout, dan topic) dapat dilihat
dengan `ros2 launch rs_control rs_jog.launch.py --show-args`.

| Tombol | Fungsi |
| --- | --- |
| `1`--`6` | Pilih joint arm |
| `7` | Pilih `joint_right-finger` |
| `+` / `-` | Gerakkan joint terpilih |
| `[` / `]` | Kurangi / tambah step |
| `Space` atau `s` | Controlled stop, torque tetap aktif |
| `h` | Ambil ulang feedback sebagai hold target |
| `q` | Stop dan keluar dari jogger |

Jogger menolak command saat feedback lengkap berhenti lebih lama dari
`feedback_timeout_s`. Keluar dari jogger tidak mematikan torque; hentikan
bringup dengan `Ctrl+C` setelah gerak berhenti.

### RViz untuk hardware

```bash
ros2 launch rs_control rs_jog_rviz.launch.py
```

Gunakan launch ini hanya untuk menampilkan hardware yang sudah dibawa oleh
`rs_bringup`. Jangan gunakan `nxp_rs_description display.launch.py` bersamaan,
karena launch tersebut membuat publisher joint simulasi sendiri.

## 9. Testing

Test tidak memerlukan CAN adapter atau motor hidup:

```bash
cd /home/jundi/nxp_rs_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select rs_control --event-handlers console_direct+
```

Test mencakup packing protocol, validasi konfigurasi dan limit, konversi
kalibrasi, serta export interface `ros2_control`.
