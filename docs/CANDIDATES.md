# UAV Geliştirme Hızlı Başlangıç Rehberi (ROS2 + PX4 + MAVROS + QGroundControl)

## 1. Kısa Tanımlar
- **ROS 2**: Robot yazılımlarını modüler şekilde geliştirmeyi sağlayan çatı.
- **PX4**: Açık kaynaklı uçuş kontrol yazılımı (drone/autopilot).
- **MAVROS**: MAVLink protokolünü ROS 2 ile köprüleyen paket.
- **QGroundControl**: PX4 için yer kontrol istasyonu arayüzü.

---

## 2. Kaynak Link Şablonları

### ROS 2
- Resmi dokümantasyon: [https://docs.ros.org/en/humble](https://docs.ros.org/en/humble)
- Kurulum rehberi: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)
- ROS 2 Tutorials: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- ROS 2 Python API: [https://docs.ros.org/en/iron/p/rclpy/](https://docs.ros.org/en/iron/p/rclpy/)
- ROS 2 C++ API: [https://docs.ros.org/en/iron/p/rclcpp/](https://docs.ros.org/en/iron/p/rclcpp/)
- ROS 2 Türkçe Öğretici Playlist [https://youtube.com/playlist?list=PLq9doJkBcZp_Z9Q7mHG1WLnB5k94In9ZM&feature=shared](https://youtube.com/playlist?list=PLq9doJkBcZp_Z9Q7mHG1WLnB5k94In9ZM&feature=shared)
- ROS 2 İngilizce Öğretici Playlist [https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&feature=shared](https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&feature=shared)

### PX4
- Resmi dokümantasyon: [https://docs.px4.io/main/en/](https://docs.px4.io/main/en/)
- Docker konteynırları: [https://docs.px4.io/main/en/test_and_ci/docker](https://docs.px4.io/main/en/test_and_ci/docker)
- Geliştirici rehberi: [https://docs.px4.io/main/en/dev_setup/](https://docs.px4.io/main/en/dev_setup/)

### MAVROS
- Resmi repo: [https://github.com/mavlink/mavros](https://github.com/mavlink/mavros)
- ROS 2 uyumlu MAVROS info: [https://github.com/mavlink/mavros/tree/ros2](https://github.com/mavlink/mavros/tree/ros2)

### QGroundControl
- Resmi site: [https://qgroundcontrol.com/](https://qgroundcontrol.com/)
- Kullanıcı kılavuzu: [https://docs.qgroundcontrol.com/master/en/](https://docs.qgroundcontrol.com/master/en/)

---

## 3. MAVROS Servis Referansı (ROS 2)

| Servis Adı                        | Açıklama                                   | Örnek Kullanım |
|------------------------------------|--------------------------------------------|----------------|
| `/mavros/cmd/arming`               | Aracı arm/disarm eder                      | `ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{ value: true }"` |
| `/mavros/set_mode`                 | Uçuş modunu değiştirir                     | `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{ custom_mode: 'AUTO.MISSION' }"` |
| `/mavros/cmd/takeoff`               | Kalkış komutu                              | `ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{ altitude: 10 }"` |
| `/mavros/cmd/land`                  | İniş komutu                                | `ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"` |
| `/mavros/param/get`                 | Parametre okuma                            | `ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet "{ param_id: 'SYSID_THISMAV' }"` |
| `/mavros/param/set`                 | Parametre yazma                            | `ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{ param_id: 'SYSID_THISMAV', value: { integer: 2 } }"` |

> Daha fazla servis listesi: [MAVROS Wiki – Services](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)

---

## 4. Terminal Komutları ile Bağlantı

**Terminal 1 – PX4 (Docker ile, Gazebo Plane)**
```bash
xhost +
docker run -it --privileged --net=host \
    -v ~/dosya_yolu(ör: ros2_ws/src):/dosya_yolu:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    --name=px4_devenv px4io/px4-dev-simulation-focal:latest bash
cd PX4-Autopilot
make px4_sitl gazebo_plane
```

**Terminal 2 – MAVROS**
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch.xml fcu_url:="udp://:14540@127.0.0.1:14557"
``` 

**Terminal 3 – QGroundControl**
```bash
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

**Terminal 4 – ROS 2 Python Node**
```bash
colcon build --packages-select uav_control_py
source install/setup.bash
ros2 run uav_control_py arm_and_mission
```

## 5. Paket Oluşturma ve Paket Yapısı

### Paket Oluşturma
Paket oluşturmak için terminalde aşağıdaki komutları kullanabilirsiniz:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python uav_control_py
```

### Python Paketi Yapısı
```
uav_control_py/
├── package.xml
├── setup.py
└── uav_control_py/
    ├── __init__.py
    └── arm_and_mission.py
```

Örnek Python Node (ROS 2)

uav_control_py/arm_and_mission.py
```python
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode

class ArmAndMissionNode(Node):
    def __init__(self):
        super().__init__('arm_and_mission_node')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

    def arm_and_set_mode(self):
        if self.arm_cli.wait_for_service(timeout_sec=5.0):
            req = CommandBool.Request()
            req.value = True
            self.arm_cli.call_async(req)

        if self.mode_cli.wait_for_service(timeout_sec=5.0):
            req = SetMode.Request()
            req.custom_mode = "AUTO.MISSION"
            self.mode_cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ArmAndMissionNode()
    node.arm_and_set_mode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

setup.py
```python
from setuptools import setup

package_name = 'uav_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='İSİM',
    maintainer_email='mail@example.com',
    description='PX4 ile arm ve mission moduna alma',
    entry_points={
        'console_scripts': [
            'arm_and_mission = uav_control_py.arm_and_mission:main'
        ],
    },
)
```

package.xml
```xml
<package format="3">
  <name>uav_control_py</name>
  <version>0.0.1</version>
  <description>PX4 ile arm ve mission moduna alma</description>
  <maintainer email="mail@example.com">İSİM</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>mavros_msgs</depend>
</package>
```