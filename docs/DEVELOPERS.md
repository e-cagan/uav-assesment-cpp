# DEVELOPERS.md — UAV Assessment (C++)

**Repo:** [github.com/e-cagan/uav-assesment-cpp](https://github.com/e-cagan/uav-assesment-cpp)  
**Stack:** ROS 2 Humble · MAVROS · PX4 SITL (Gazebo) · GTest · Colcon  
**Amaç:** Adayların bir MAVROS tabanlı C++ istemci (`MavrosClient`) yazarak Arm/Disarm ve Takeoff/Land görevlerini başarması. E2E testler PX4 SITL üzerinde koşar.

---

## İçindekiler

- [Hızlı Başlangıç (TL;DR)](#hızlı-başlangıç-tldr)
- [Dizin Yapısı](#dizin-yapısı)
- [Geliştirme Ortamı](#geliştirme-ortamı)
  - [Docker ile (Önerilen)](#docker-ile-önerilen)
  - [Yerel kurulum (Opsiyonel)](#yerel-kurulum-opsiyonel)
- [Build](#build)
- [SITL & MAVROS’ı çalıştırma](#sitl--mavrosı-çalıştırma)
- [Testler](#testler)
  - [E2E görev testleri](#e2e-görev-testleri)
  - [Test ekleme](#test-ekleme)
  - [Yerelde test çalıştırma](#yerelde-test-çalıştırma)
  - [Sonuçların yorumlanması](#sonuçların-yorumlanması)
- [Aday API’si – MavrosClient nasıl yazılır](#aday-apisı--mavrosclient-nasıl-yazılır)
  - [Beklenen davranış](#beklenen-davranış)
  - [Örnek iskelet kod](#örnek-iskelet-kod)
- [Scriptler](#scriptler)
- [CI/CD](#cicd)
  - [Workflow özeti](#workflow-özeti)
  - [Artifact’lar ve özet](#artifactlar-ve-özet)
  - [GHCR imajı](#ghcr-imajı)
- [Hata ayıklama ve SSS](#hata-ayıklama-ve-sss)
- [Kodlama kuralları ve konvansiyonlar](#kodlama-kuralları-ve-konvansiyonlar)

---

## Hızlı Başlangıç

```sh
# 1) Repo’yu klonla
git clone https://github.com/e-cagan/uav-assesment-cpp.git
cd uav-assesment-cpp

# 2) Docker imajını kullanarak interaktif kabuğa gir
docker run --rm -it -v "$PWD":/ws -w /ws ghcr.io/e-cagan/uav-assessment-cpp-env:latest bash
# veya yerelde kurduysan: direkt shell'de devam et

# 3) Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to uav_tests_cpp
source install/setup.bash

# 4) E2E Test – Task 1 (Arm/Disarm)
REPO_ROOT=/ws ./src/uav_tests_cpp/tests/run_e2e.sh build/uav_tests_cpp/test_task1_arm_disarm

# 5) E2E Test – Task 2 (Takeoff/Land)
REPO_ROOT=/ws ./src/uav_tests_cpp/tests/run_e2e.sh build/uav_tests_cpp/test_task2_takeoff_land
```

> **Not:** E2E script, PX4 SITL + MAVROS’ı başlatır ve test binary’sini GTest olarak koşturur.

---

## Dizin Yapısı

```
uav-assesment-cpp/
├─ run_sitl.sh                  # PX4 SITL başlatır (headless)
├─ run_mavros.sh                # MAVROS node'unu başlatır (udp://:PORT@)
├─ src/
│  ├─ uav_control_cpp/          # Adayın implemente edeceği kütüphane
│  │  ├─ include/uav_control_cpp/
│  │  │  ├─ mavros_client.hpp   # Public API (init, arm, set_mode, ...)
│  │  │  └─ not_implemented.hpp # SKIP mekanizması için yardımcı tip
│  │  ├─ src/mavros_client.cpp  # Örnek/iskelet (aday burada çalışır)
│  │  ├─ CMakeLists.txt
│  │  └─ package.xml
│  └─ uav_tests_cpp/            # GTest + E2E sarmalayıcı
│     ├─ include/uav_tests_cpp/
│     │  ├─ mavros_fixture.hpp  # GTest fixture (ROS init/shutdown)
│     │  └─ skip_if.hpp         # SKIP makrosu (NotImplemented -> SKIP)
│     ├─ test/
│     │  ├─ test_task1_arm_disarm.cpp
│     │  └─ test_task2_takeoff_land.cpp
│     ├─ tests/run_e2e.sh       # SITL+MAVROS+GTest orkestrasyonu
│     ├─ CMakeLists.txt         # gtest tanımları + add_test(E2E)
│     └─ package.xml
├─ .github/workflows/ci.yml     # CI pipeline (E2E, özet, GHCR push)
├─ Dockerfile                   # Geliştirme/CI imajı
└─ README.md / DEVELOPERS.md / CANDIDATES.md   # Dokümanlar
```

---

## Geliştirme Ortamı

### Docker ile (Önerilen)

- **İmaj:** `ghcr.io/e-cagan/uav-assessment-cpp-env:latest`
- PX4 v1.12.3, Gazebo, ROS 2 Humble, MAVROS, GeographicLib dataset’leri ve colcon içerir.

```sh
docker run --rm -it \
  -v "$PWD":/ws -w /ws \
  ghcr.io/e-cagan/uav-assessment-cpp-env:latest bash
```

### Yerel kurulum (Opsiyonel)

- Ubuntu 22.04, ROS 2 Humble kurulumu, ros-humble-mavros, ros-humble-mavros-extras,
- Gazebo, GeographicLib dataset’leri (MAVROS’ın scripti ile), colcon ve build araçları.
- PX4 SITL (v1.12.x) kaynak/build.

> **Not:** CI imajı ile sürüm uyumunu korumak en güvenli yoldur.

---

## Build

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to uav_tests_cpp
source install/setup.bash
```

- `uav_control_cpp` lib’i derlenir ve `uav_tests_cpp` testleri linklenir.
- İzinler:  
  ```sh
  chmod +x run_sitl.sh run_mavros.sh src/uav_tests_cpp/tests/run_e2e.sh
  ```

---

## SITL & MAVROS’ı çalıştırma

**Manuel (debug):**
```sh
# 1) PX4 SITL (headless)
./run_sitl.sh &   # log: /tmp/sitl_e2e.log

# 2) MAVROS (PX4 ile UDP köprü)
./run_mavros.sh 14557 &   # log: /tmp/mavros_e2e.log

# 3) ROS durumu
source /opt/ros/humble/setup.bash
ros2 topic list | grep /mavros/state
```

**Portlar & gereksinimler**
- MAVROS’ın PX4’e bağlandığı varsayılan UDP port: **14557**
- OFFBOARD modu için setpoint’lerin > 2 Hz akması gerekir (öneri 20–50 Hz).

---

## Testler

### E2E görev testleri

- **Task 1 — Arm/Disarm:** `test_task1_arm_disarm.cpp`  
  `MavrosClient::init()`, `wait_for_mavros()`, `arm(true/false)`.
- **Task 2 — Takeoff/Land:** `test_task2_takeoff_land.cpp`  
  OFFBOARD’a hazırlık için `pump_setpoints()`, `set_mode("OFFBOARD")`, `arm(true)`, tırmanış ve `wait_alt_ge()`, ardından iniş (öncelik AUTO.LAND, yoksa setpoint ile alçaltma).

**SKIP mekaniği:**  
Aday `std::logic_error` (mesajında “not implemented/TODO” benzeri) veya `uav::NotImplemented` fırlatırsa test SKIP edilir. Böylece kademeli geliştirme yapılabilir.

---

### Test ekleme

- `src/uav_tests_cpp/test/` altında yeni bir `test_*.cpp` ekle.
- `CMakeLists.txt` içine:
  ```cmake
  ament_add_gtest(test_task3_something test/test_task3_something.cpp)
  ament_target_dependencies(test_task3_something rclcpp mavros_msgs geometry_msgs uav_control_cpp)
  add_test(NAME task3_something
    COMMAND /usr/bin/bash -lc
      "REPO_ROOT=${CMAKE_SOURCE_DIR}/../.. ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e2e.sh $<TARGET_FILE:test_task3_something>")
  ```
- Gerekirse `tests/run_e2e.sh` içine özel bekleme/ortam adımı ekleyebilirsin (genelde gerekmez).

---

### Yerelde test çalıştırma

```sh
# Tek binary
REPO_ROOT=/ws ./src/uav_tests_cpp/tests/run_e2e.sh build/uav_tests_cpp/test_task1_arm_disarm

# CTest ile (paket bağlamında)
colcon test --packages-select uav_tests_cpp --ctest-args -R "^task1_arm_disarm$" -VV
```

---

### Sonuçların yorumlanması

- **PASSED:** GTest başarılı.
- **FAILED:** GTest failure veya E2E script non-zero döndü (örn. MAVROS hazır değil).
- **SKIPPED:** Aday fonksiyonları “TODO/NotImplemented” olarak işaretledi.

---

## Aday API’si – MavrosClient nasıl yazılır

### Beklenen davranış

`include/uav_control_cpp/mavros_client.hpp`:

```cpp
void init();
bool wait_for_mavros(double timeout_s);
bool arm(bool value, double timeout_s);
bool set_mode(const std::string& mode, double timeout_s);
void pump_setpoints(double z, int count, std::chrono::milliseconds dt);
bool wait_alt_ge(double alt, double timeout_s);
```

**Temel ROS arayüzleri (MAVROS):**

- State topic: `/mavros/state` (`mavros_msgs::msg::State`)
- Local pose: `/mavros/local_position/pose` (`geometry_msgs::msg::PoseStamped`)
- Setpoint publisher: `/mavros/setpoint_position/local` (`geometry_msgs::msg::PoseStamped`)
- Arm servis: `/mavros/cmd/arming` (`mavros_msgs::srv::CommandBool`)
- Mode servis: `/mavros/set_mode` (`mavros_msgs::srv::SetMode`)

> **NOT:** OFFBOARD için setpoint’leri önceden akıt (en az birkaç yüz ms, 20–50 Hz) ve akışı kesme.

---

### Örnek iskelet kod

Bu sadece fikir vermek içindir; exception & zaman aşımı yönetimini kendi tarzına göre netleştir.

```cpp
#include "uav_control_cpp/mavros_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
using namespace std::chrono_literals;

MavrosClient::MavrosClient() {}

void MavrosClient::init() {
  node_ = std::make_shared<rclcpp::Node>("uav_control_client");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State &msg){ state_ = msg; });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped &msg){
        last_pose_ = msg; got_pose_.store(true);
      });

  sp_pub_   = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

  arm_cli_  = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_cli_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosClient::wait_for_mavros(double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->now() < deadline) {
    exec_->spin_some();
    // servislerin gelmesini bekle
    if (arm_cli_->wait_for_service(0ms) && mode_cli_->wait_for_service(0ms)) {
      // topic’ler akıyor mu? en azından state aldıysak kabul
      return true;
    }
    rclcpp::sleep_for(100ms);
  }
  return false;
}

bool MavrosClient::arm(bool value, double timeout_s) {
  if (!arm_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = value;
  auto fut = arm_cli_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) return false;
  return fut.get()->success;
}

bool MavrosClient::set_mode(const std::string& mode, double timeout_s) {
  if (!mode_cli_->wait_for_service(1s)) return false;
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;
  auto fut = mode_cli_->async_send_request(req);
  if (fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) return false;
  return fut.get()->mode_sent;
}

void MavrosClient::pump_setpoints(double z, int count, std::chrono::milliseconds dt) {
  // son poz varsa onu referans al; yoksa 0,0’dan gönder
  geometry_msgs::msg::PoseStamped sp;
  sp.header.frame_id = "map";
  sp.pose.position.x = got_pose_ ? last_pose_.pose.position.x : 0.0;
  sp.pose.position.y = got_pose_ ? last_pose_.pose.position.y : 0.0;
  sp.pose.position.z = z;
  sp.pose.orientation.w = 1.0;

  for (int i=0; i<count && rclcpp::ok(); ++i) {
    sp.header.stamp = node_->now();
    sp_pub_->publish(sp);
    exec_->spin_some();
    rclcpp::sleep_for(dt);
  }
}

bool MavrosClient::wait_alt_ge(double alt, double timeout_s) {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && node_->now() < deadline) {
    exec_->spin_some();
    if (got_pose_.load() && last_pose_.pose.position.z >= alt) return true;
    rclcpp::sleep_for(50ms);
  }
  return false;
}
```

---

## Scriptler

- **run_sitl.sh**  
  PX4 SITL’i Gazebo ile headless başlatır. Loglar: `/tmp/sitl_e2e.log`
- **run_mavros.sh <port>**  
  MAVROS’ı verilen UDP portu ile PX4’e bağlar. Log: `/tmp/mavros_e2e.log`  
  Örn: `./run_mavros.sh 14557`
- **src/uav_tests_cpp/tests/run_e2e.sh <gtest-bin>**
  - SITL’i başlatır
  - MAVROS’ı başlatır
  - `/mavros/state` görünene kadar bekler
  - Verilen GTest binary’sini çalıştırır
  - Temizlik yapar (px4, gzserver, mavros)
  - Başarısızlıkta ilgili log tail’lerini basar.

---

## CI/CD

### Workflow özeti

- Docker Buildx ile imaj build+load, colcon build.
- E2E testleri doğrudan GTest binary’leriyle çalıştırır (CTEST sarmalamasız).
- Her görev için durum tespiti yapıp GitHub Step Summary’ye rozetler basar.
- PR dışında GHCR’a latest ve SHA ile push eder.

### Artifact’lar ve özet

- `.ci_artifacts/`
  - `task*_gtest.log`, `*_sitl.log`, `*_mavros.log`, status dosyaları.
- Workflow sonunda Step Summary:
  - MAVROS smoke, Task 1, Task 2 PASS/FAIL/SKIP rozetleri
  - Log tail’leri collapsible `<details>` içinde.

### GHCR imajı

```sh
docker pull ghcr.io/<owner>/uav-assessment-cpp-env:latest
docker run -it --rm ghcr.io/<owner>/uav-assessment-cpp-env:latest bash
```

> CI sonunda ayrıca `:<commit-sha>` tag’i de basılır.

---

## Hata ayıklama ve SSS

- **AMENT_TRACE_SETUP_FILES: unbound variable**  
  `set -u` aktif shell’de `source /opt/ros/humble/setup.bash` öncesi hata verebilir.  
  Çözüm: `set +u; source ...; set -u` (E2E scriptte bu uygulanır).

- **CTEST “Cannot create directory /ws/build/…”**  
  Konteynerde `-w /ws` ile çalıştığından emin olun. E2E zaten gtest’i doğrudan çağırıyor; CTest güncel CI’da kullanılmıyor.

- **MAVROS hazır değil**  
  `/mavros/state` görünmüyorsa `run_mavros.sh` portu ve SITL logunu kontrol et. PX4 boot ettikten sonra MAVROS’ı başlatmak önemli.

- **OFFBOARD set edilemiyor**  
  Önce `pump_setpoints()` ile belirli bir frekansta (>= 20 Hz) veri akıt. Setpoint akışı yoksa PX4 OFFBOARD’u reddeder.

- **Altitude yükselmiyor**  
  `pump_setpoints(3.0, …)` çalışırken publish oranını ve poz çerçevesini doğrula (map/local). `wait_alt_ge()` periyodik spin yapmalı.

---

## Kodlama kuralları ve konvansiyonlar

- C++17 (ROS 2 Humble ile uyumlu).
- Ament/Colcon ile derleme, `uav_control_cpp` kütüphane, `uav_tests_cpp` test paketi.
- **SKIP Mekanizması:** Henüz yazmadığın fonksiyonlarda `throw uav::NotImplemented("...")` veya `std::logic_error("TODO: ...")`. Testler SKIP’e çevirir.
- **Zaman aşımı & bekleme:** Servis bekleme, spin ve sleep’lerde küçük granülerlik (50–100 ms) ve üst limit parametreleri.
- **Log/diagnostik:** Aday kodda `RCLCPP_INFO/WARN/ERROR` kullanabilirsin (testler stdout/stderr’i