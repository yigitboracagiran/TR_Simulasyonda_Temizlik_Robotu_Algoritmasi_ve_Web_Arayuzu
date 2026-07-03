# Simülasyonda Temizlik Robotu Algoritması ve Web Arayüzü

Bu proje, ROS tabanlı bir temizlik robotu simülasyonu için hazırlanmış kontrol algoritması ve web arayüzünden oluşur. Sistem; joystick kontrolü, web arayüzü üzerinden manuel kontrol, harita üzerinden hedef seçme, kamera görüntüsünü web arayüzünde gösterme ve otonom temizlik algoritması gibi işlevleri içerir.

## Hedef Ortam

| Bileşen | Sürüm / Notlar |
|---|---|
| İşletim Sistemi | Ubuntu 20.04 önerilir |
| ROS | ROS 1 Noetic önerilir |
| Programlama Dilleri | Python, JavaScript, HTML, CSS |
| Python Kütüphaneleri | `rospy`, `numpy`, `roslaunch`, `actionlib` |
| ROS Mesaj Paketleri | `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `move_base_msgs` |
| Web Bağlantısı | ROSLIB.js + rosbridge websocket |
| WebSocket Adresi | `ws://localhost:9090` |
| Harita Görseli | `resimler/map4.jpg` |

> [!NOTE]
> Bu README, mevcut repo yapısına göre hazırlanmıştır. Dosya yolları, ROS workspace konumunuz veya kullandığınız simülasyon paketine göre değiştirilebilir.

---

## İçindekiler

- [1. Program Hakkında](#1-program-hakkında)
- [2. Proje Yapısı](#2-proje-yapısı)
- [3. Kullanılan Dosyalar](#3-kullanılan-dosyalar)
- [4. Gereksinimler](#4-gereksinimler)
- [5. Kurulum](#5-kurulum)
- [6. ROS ve rosbridge Hazırlığı](#6-ros-ve-rosbridge-hazırlığı)
- [7. Web Arayüzünü Çalıştırma](#7-web-arayüzünü-çalıştırma)
- [8. Python Kontrol Kodunu Çalıştırma](#8-python-kontrol-kodunu-çalıştırma)
- [9. Kullanım Modları](#9-kullanım-modları)
- [10. Web Arayüzü Kontrolleri](#10-web-arayüzü-kontrolleri)
- [11. Otonom Temizlik Algoritması](#11-otonom-temizlik-algoritması)
- [12. ROS Topic Yapısı](#12-ros-topic-yapısı)
- [13. Olası Sorunlar](#13-olası-sorunlar)
- [14. Final Kontrol Listesi](#14-final-kontrol-listesi)
- [15. Tanıtım Videosu](#15-tanitim-videosu)
---

## 1. Program Hakkında

Bu proje, simülasyon ortamında çalışan bir temizlik robotunun farklı kontrol modlarıyla yönetilmesini amaçlar.

Program temel olarak üç bölümden oluşur:

1. **ROS Python kontrol algoritması**
   - Joystick kontrolünü yönetir.
   - Web arayüzünden gelen hız komutlarını işler.
   - Harita üzerinden seçilen hedef noktaya robotu göndermeye çalışır.
   - Otonom temizlik hareketi için yılan tipi tarama mantığı kullanır.

2. **Web arayüzü**
   - Robotun kamera görüntüsünü gösterir.
   - Harita görselini gösterir.
   - Robotun anlık konumunu harita üzerinde işaretler.
   - Manuel hareket butonları, mod değiştirme butonu ve acil durdurma butonu içerir.

3. **Harita ve görsel takip sistemi**
   - Harita görseli üzerinden tıklanan noktaları ROS topic olarak yayınlar.
   - Robotun geçtiği noktaları harita üzerinde boyar.
   - Robotun yönünü ok işaretiyle gösterir.

---

## 2. Proje Yapısı

Örnek repo yapısı aşağıdaki gibidir:

```text
TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu/
├── README.md
├── index.html
├── kodlar/
│   ├── app.js
│   ├── joy.py
│   └── style.css
├── launch/
│   └── joy.launch
└── resimler/
    └── map4.jpg
```

> [!IMPORTANT]
> `index.html` dosyası bazı JavaScript kütüphanelerini `kutuphaneler/` klasöründen çağırır. Bu klasör sisteminizde yoksa ilgili dosyaları eklemeniz veya `index.html` içindeki kaynak yollarını güncellemeniz gerekir.

---

## 3. Kullanılan Dosyalar

| Dosya / Klasör | Açıklama |
|---|---|
| `index.html` | Web arayüzünün ana HTML dosyasıdır. Harita, kamera alanı, yön butonları, mod butonu ve acil durdurma butonu burada yer alır. |
| `kodlar/app.js` | Web arayüzü ile ROS arasındaki bağlantıyı sağlar. ROSLIB ile topic yayınlama ve topic dinleme işlemlerini yapar. |
| `kodlar/joy.py` | ROS tarafındaki ana Python kontrol kodudur. Joystick, arayüz, hedef nokta ve otonom hareket işlemlerini yönetir. |
| `kodlar/style.css` | Web arayüzünün görünüm ayarlarını içerir. |
| `launch/joy.launch` | Joystick modunda çalıştırılacak launch dosyasıdır. |
| `resimler/map4.jpg` | Web arayüzünde gösterilen harita görselidir. |

---

## 4. Gereksinimler

Bu projenin çalışması için ROS 1 tabanlı bir sistem önerilir.

Gerekli temel paketler:

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  ros-noetic-rosbridge-server \
  ros-noetic-joy \
  ros-noetic-teleop-twist-joy \
  ros-noetic-navigation \
  ros-noetic-map-server \
  ros-noetic-move-base-msgs
```

Python için gerekli paket:

```bash
pip3 install numpy
```

ROS ortamını aktif etmek için:

```bash
source /opt/ros/noetic/setup.bash
```

Kalıcı yapmak için:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Kurulum

Repoyu bilgisayarınıza klonlayın:

```bash
cd ~
git clone https://github.com/yigitboracagiran/TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu.git
cd TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu
```

Python dosyasını çalıştırılabilir yapmak için:

```bash
chmod +x kodlar/joy.py
```

---

## 6. ROS ve rosbridge Hazırlığı

Web arayüzü, ROS ile `ws://localhost:9090` adresi üzerinden haberleşir. Bunun için `rosbridge_websocket` çalışmalıdır.

Önce ROS master başlatılır:

```bash
roscore
```

Yeni bir terminal açılarak rosbridge başlatılır:

```bash
source /opt/ros/noetic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

> [!NOTE]
> `app.js` içinde ROS bağlantısı şu adrese yapılır:
>
> ```javascript
> ros.connect('ws://localhost:9090');
> ```
>
> Eğer rosbridge farklı bir bilgisayarda çalışıyorsa bu adresi kendi IP adresinize göre değiştirmeniz gerekir.

---

## 7. Web Arayüzünü Çalıştırma

Repo ana dizinindeyken basit bir HTTP sunucusu başlatabilirsiniz:

```bash
cd ~/TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu
python3 -m http.server 8000
```

Ardından tarayıcıdan şu adresi açın:

```text
http://localhost:8000/index.html
```

Web arayüzünde aşağıdaki alanlar bulunur:

- Harita görseli
- Kamera görüntüsü alanı
- İleri, geri, sağ, sol ve dur butonları
- `Joystick Arayuz Otonom` mod değiştirme butonu
- `DUR!` acil durdurma butonu

---

## 8. Python Kontrol Kodunu Çalıştırma

`joy.py` dosyasında joystick launch dosyası göreceli yol ile çağrıldığı için Python kodunu `kodlar/` klasörünün içinden çalıştırmak daha uygundur.

```bash
cd ~/TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu/kodlar
source /opt/ros/noetic/setup.bash
python3 joy.py
```

> [!WARNING]
> Kod içinde bazı harita, koordinat ve oda sınırı değerleri sabit olarak tanımlanmıştır. Farklı bir harita veya simülasyon ortamı kullanıyorsanız bu değerleri kendi sisteminize göre güncellemeniz gerekir.

---

## 9. Kullanım Modları

Sistemde mod bilgisi `/modKontrolu` topic'i üzerinden `std_msgs/String` olarak yayınlanır.

| Değer | Mod | Açıklama |
|---|---|---|
| `-2` | Acil Durdurma | Robot durdurulur ve mod geçişleri kilitlenir. |
| `-1` | Pasif Mod | Herhangi bir aktif sürüş modu yoktur. |
| `0` | Joystick Modu | Joystick launch dosyası çalıştırılır. |
| `1` | Arayüz Modu | Web arayüzündeki butonlar ve harita tıklamaları kullanılır. |
| `2` | Otonom Mod | Otonom temizlik algoritması çalışır. |

Mod değiştirme işlemi web arayüzündeki `Joystick Arayuz Otonom` butonu üzerinden yapılır.

---

## 10. Web Arayüzü Kontrolleri

Web arayüzündeki yön butonları yalnızca arayüz modu aktifken çalışır.

| Buton | Görev |
|---|---|
| `▲` | Lineer hızı artırır. |
| `▼` | Lineer hızı azaltır. |
| `◀` | Açısal hızı artırır. |
| `▶` | Açısal hızı azaltır. |
| `D` | Lineer ve açısal hızı sıfırlar. |
| `DUR!` | Acil durdurma modunu açar veya kapatır. |
| `Joystick Arayuz Otonom` | Joystick, arayüz ve otonom modlar arasında geçiş yapar. |

Web arayüzü şu işlemleri yapar:

- `/camera/rgb/image_raw/compressed` topic'inden kamera görüntüsünü alır.
- `/cmd_vel1` topic'ine hız komutu gönderir.
- `/clicked_point` topic'i ile haritada tıklanan pikseli yayınlar.
- `/ok` topic'i ile robotun haritadaki anlık konumunu alır.
- Robotun geçtiği pikselleri harita üzerinde kırmızıya boyar.

---

## 11. Otonom Temizlik Algoritması

Otonom modda robot, oda içinde sütun sütun ilerleyen yılan tipi bir temizlik mantığıyla hareket eder.

Algoritmanın genel akışı:

1. Harita verisi `/map` topic'inden alınır.
2. Harita verisi matris yapısına dönüştürülür.
3. Bilinmeyen alanlar, boş alanlar ve engeller analiz edilir.
4. Robot yukarı veya aşağı yönde ilerleyerek bir sütunu tamamlar.
5. Engel veya duvar tespit edilirse engelin diğer tarafına geçmeye çalışır.
6. Sütun tamamlanınca yana geçer.
7. Son sütun tamamlandığında oda temizliği tamamlanmış kabul edilir.

> [!NOTE]
> Otonom algoritmadaki oda sınırları, başlangıç pikselleri ve orijin piksel değerleri haritaya özel olarak ayarlanmıştır. Yeni bir harita kullanıldığında bu değerler yeniden düzenlenmelidir.

---

## 12. ROS Topic Yapısı

Projede kullanılan başlıca topic'ler aşağıdaki gibidir:

| Topic | Mesaj Tipi | Açıklama |
|---|---|---|
| `/modKontrolu` | `std_msgs/String` | Aktif sürüş modunu belirler. |
| `/cmd_vel` | `geometry_msgs/Twist` | Robotun gerçek hız komutudur. |
| `/cmd_vel1` | `geometry_msgs/Twist` | Web arayüzünden gelen hız komutudur. |
| `/clicked_point` | `geometry_msgs/Point` | Web haritasında tıklanan piksel bilgisidir. |
| `/ok` | `geometry_msgs/Point` | Robotun web haritasındaki konum ve yön bilgisidir. |
| `/camera/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | Web arayüzünde gösterilen kamera görüntüsüdür. |
| `/odom` | `nav_msgs/Odometry` | Robotun anlık odometri bilgisidir. |
| `/map` | `nav_msgs/OccupancyGrid` | Harita bilgisidir. |

---

## 13. Olası Sorunlar

### Web arayüzü ROS'a bağlanmıyor

Aşağıdakileri kontrol edin:

- `roscore` çalışıyor mu?
- `rosbridge_websocket` çalışıyor mu?
- `app.js` içindeki websocket adresi doğru mu?
- Tarayıcı konsolunda bağlantı hatası var mı?

Kontrol için:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Kamera görüntüsü gelmiyor

Aşağıdaki topic'in yayınlanıp yayınlanmadığını kontrol edin:

```bash
rostopic list | grep camera
rostopic echo /camera/rgb/image_raw/compressed
```

Eğer kamera topic adı farklıysa `kodlar/app.js` dosyasındaki şu alanı değiştirin:

```javascript
name : '/camera/rgb/image_raw/compressed'
```

### Yön butonları robotu hareket ettirmiyor

Aşağıdakileri kontrol edin:

- Web arayüzü arayüz modunda mı?
- `/cmd_vel1` topic'ine veri gidiyor mu?
- `joy.py` çalışıyor mu?
- `/cmd_vel` topic'i robot tarafından dinleniyor mu?

Kontrol için:

```bash
rostopic echo /cmd_vel1
rostopic echo /cmd_vel
```

### Haritaya tıklayınca robot hedefe gitmiyor

Aşağıdakileri kontrol edin:

- `move_base` çalışıyor mu?
- `/map` topic'i yayınlanıyor mu?
- `/odom` topic'i yayınlanıyor mu?
- `map` frame'i ile robot frame'i arasında TF bağlantısı var mı?

Kontrol için:

```bash
rostopic echo /map
rostopic echo /odom
rosrun tf view_frames
```

### Joystick modu çalışmıyor

Aşağıdakileri kontrol edin:

- Joystick bilgisayara bağlı mı?
- `/joy` topic'i yayınlanıyor mu?
- `launch/joy.launch` dosyası doğru yapılandırılmış mı?

Kontrol için:

```bash
rostopic echo /joy
```

### Harita veya robot konumu yanlış yerde görünüyor

`joy.py` içindeki piksel-koordinat dönüşüm değerlerini kontrol edin. Özellikle şu değişkenler haritaya göre ayarlanmalıdır:

```python
orijinSutunPikseli
orijinSatirPikseli
baslangicSutunPikseli
baslangicSatirPikseli
odaSatirUstPiksel
odaSatirAltPiksel
OdaSutunSagPiksel
OdaSutunSolPiksel
```

---

## 14. Final Kontrol Listesi

Programı çalıştırmadan önce aşağıdakileri kontrol edin:

- [ ] ROS ortamı doğru şekilde source edildi.
- [ ] `roscore` çalışıyor.
- [ ] `rosbridge_websocket` çalışıyor.
- [ ] Simülasyon ortamı başlatıldı.
- [ ] `/map` topic'i yayınlanıyor.
- [ ] `/odom` topic'i yayınlanıyor.
- [ ] `/cmd_vel` robot tarafından dinleniyor.
- [ ] Kamera topic'i yayınlanıyor.
- [ ] `index.html` tarayıcıda açıldı.
- [ ] `kodlar/joy.py` çalıştırıldı.
- [ ] Web arayüzündeki mod butonu doğru çalışıyor.
- [ ] Acil durdurma butonu test edildi.
- [ ] Harita koordinatları kullanılan simülasyona göre ayarlandı.

---

<a id="15-tanitim-videosu"></a>

## 15. Tanıtım Videosu

Tanıtım videosunu direkt açmak için: https://github.com/yigitboracagiran/TR_Simulasyonda_Temizlik_Robotu_Algoritmasi_ve_Web_Arayuzu/assets/111417887/bf8bb40a-8892-492e-8b2d-298c19551939
