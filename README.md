# DriveSafe — Firmware ESP32

Implementasi lengkap Dokumen **C-251** (Capstone B-06, DTETI FT UGM):
*Sistem Deteksi Kecelakaan Kendaraan dan Respons Darurat Cerdas Berbasis IoT
dengan Perangkat Multisensor.*

---

## 1. Struktur berkas

Letakkan seluruh berkas dalam satu folder bernama `DriveSafe`, lalu buka
`DriveSafe.ino` di Arduino IDE.

| Berkas | Isi |
|---|---|
| `DriveSafe.ino` | Mesin keadaan utama (Gambar 4.2 & 4.3), dua task FreeRTOS, penyusunan SMS, CLI pengujian |
| `config.h` | Seluruh pin, threshold, timing, UUID, dan tipe data bersama |
| `sensor.h/.cpp` | Driver BNO055 mandiri (Wire), kalibrasi bias, estimasi orientasi |
| `detector.h/.cpp` | Fitur turunan, adaptive threshold, deteksi multi-tahap |
| `eventbuf.h/.cpp` | Circular buffer pre-impact 10 s + post-impact 5 s |
| `storage.h/.cpp` | NVS (kontak darurat, profil) + event log ke LittleFS |
| `gps.h/.cpp` | NEO-M8N via UART1, cache *last known good fix* |
| `gsm.h/.cpp` | SIM800L: `AT+CMGS`, `ATD`, retry, sanitasi GSM 7-bit |
| `bleif.h/.cpp` | BLE peripheral + parser JSON ringan |
| `ui.h/.cpp` | Buzzer non-blocking, LED status, tombol interrupt |

---

## 2. Pustaka yang perlu dipasang

Lewat **Library Manager** Arduino IDE:

- **TinyGPSPlus** oleh Mikal Hart

Sisanya sudah termasuk dalam **esp32 board package** oleh Espressif
(`Wire`, `Preferences`, `LittleFS`, `BLEDevice`). Tidak ada dependensi lain —
driver BNO055 dan parser JSON ditulis sendiri agar bebas dari masalah versi
pustaka.

**Pengaturan board:** ESP32 Dev Module · Partition Scheme *Default 4MB with
spiffs* · Flash 4MB · Upload speed 921600 · Serial Monitor 115200 baud.

---

## 3. Wiring

| Modul | Pin modul | Pin ESP32 | Catatan |
|---|---|---|---|
| BNO055 | SDA | GPIO21 | I2C 400 kHz, alamat 0x28 |
| | SCL | GPIO22 | |
| | VIN / 3V3 | 3V3 | |
| NEO-M8N | TX | GPIO16 | UART1 RX, 9600 bps |
| | RX | GPIO17 | UART1 TX |
| | VCC | 5V | |
| SIM800L | TX | GPIO26 | UART2 RX, 9600 bps |
| | RX | GPIO27 | UART2 TX |
| | RST | GPIO14 | opsional |
| | VCC | **3,7–4,2 V khusus** | jangan dari 5 V ESP32 |
| Buzzer aktif | + | GPIO25 | |
| Push button | satu kaki | GPIO33 | kaki lain ke GND, `INPUT_PULLUP` |
| Pembagi tegangan baterai | titik tengah | GPIO34 | 100 kΩ / 100 kΩ |

**Wajib:** kapasitor bulk 1000–2200 µF pada jalur VCC SIM800L. Modul menarik
arus puncak hingga 2 A saat transmisi dan tanpa buffer ini ESP32 akan *reset*
tepat pada saat pengiriman SMS darurat (Bab 7.3).

Seluruh GND disatukan. Arah pemasangan mengikuti panah pada *enclosure*
sehingga **sumbu X BNO055 sejajar arah gerak kendaraan** (Bab 8.2.4) — arah ini
menentukan klasifikasi tabrakan depan vs ditabrak dari belakang.

---

## 4. Penggunaan pertama

1. Unggah firmware, buka Serial Monitor pada 115200 baud.
2. Ketik `help` untuk daftar perintah.
3. Atur identitas dan kontak darurat:

```
owner Lisa Olivia
profile motor
add Ayah +6281234567890 1
add Ibu +6285678901234 2
contacts
```

4. Uji jalur GSM: `gsm` lalu `sms`.
5. Uji GPS di ruang terbuka: `gps` (butuh 30–90 detik untuk *cold start*).
6. **Rekam baseline** sebelum uji kecelakaan: `baseline 120`, lalu berkendara
   normal selama dua menit dengan manuver agresif. Sistem akan mencetak usulan
   nilai threshold empiris (lihat Bagian 8).
7. Uji rantai lengkap tanpa membanting perangkat: `sim`.

Tombol fisik: **tekan singkat** membatalkan alarm, **tekan ≥ 2 detik**
menghidupkan/mematikan mode pairing BLE.

---

## 5. Protokol BLE (untuk tim aplikasi mobile)

Nama perangkat: `DriveSafe-XXXX` (4 digit heksadesimal terakhir MAC).

Service `4f1a0000-9c1e-4a2b-8d33-7b21c0de5afe`

| Characteristic | UUID akhiran | Properti | Isi |
|---|---|---|---|
| CONFIG | `...0001` | Write | JSON konfigurasi → disimpan ke NVS |
| STATUS | `...0002` | Read, Notify | status perangkat tiap 1 detik |
| EVENT | `...0003` | Read, Notify | detail kejadian kecelakaan |
| CMD | `...0004` | Write | perintah singkat |

**Menulis konfigurasi** (boleh terpecah beberapa paket, firmware menunggu
sampai kurung kurawal seimbang):

```json
{"owner":"Lisa","profile":"motor","countdown":30,
 "contacts":[{"n":"Ayah","p":"+6281234567890","pr":1},
             {"n":"Ibu","p":"+6285678901234","pr":2}]}
```

`profile` menerima `sepeda` / `motor` / `mobil`. `pr` = 1 berarti kontak
prioritas utama yang menerima *missed call*.

**Notifikasi status** (untuk tab Beranda, Sub-bab 10.2.4):

```json
{"st":"MONITOR","bat":87,"sens":1,"gps":1,"sat":9,
 "gsm":68,"spd":42.3,"con":3,"cd":0,"prof":"motor"}
```

`st` bernilai `MONITOR`, `VERIFY`, `COUNTDOWN`, `ALERT`, `CANCELLED`,
`COOLDOWN`, atau `PAIRING`. `cd` adalah sisa hitung mundur dalam detik —
inilah angka yang ditampilkan pada layar "KECELAKAAN TERDETEKSI!!".

**Notifikasi kejadian** (Sub-bab 10.2.6–10.2.7):

```json
{"ph":"SENDING","ts":1789564985,"g":4.82,"j":3412.0,"r":1180.5,
 "roll":72.4,"pitch":11.2,"sev":1,"sevn":"Sedang",
 "type":5,"typen":"Terguling","lat":-7.765400,"lon":110.372100,
 "map":"maps.google.com/?q=-7.76540,110.37210"}
```

`ph` bernilai `SENDING` saat pengiriman dimulai dan `SENT` setelah selesai.

**Perintah** (tulis sebagai teks biasa ke CMD): `CANCEL`, `TEST_SMS`,
`RECAL`, `SIMCRASH`, `CLEARCON`.

---

## 6. Format SMS darurat

Disusun agar tetap di bawah 160 karakter GSM 7-bit sehingga terkirim sebagai
satu bagian:

```
DRIVESAFE - KECELAKAAN
Lisa Olivia (motor)
Terguling 4.8g Sedang
16/09 14:23 WIB
maps.google.com/?q=-7.76540,110.37210
```

SMS dikirim ke seluruh kontak terdaftar secara berurutan, disusul *missed
call* 5 detik ke kontak prioritas pertama. Kegagalan pengiriman diulang
otomatis maksimal dua kali.

---

## 7. Pemetaan kode terhadap dokumen

| Dokumen | Lokasi di kode |
|---|---|
| Pers. 4.7 kompensasi bias giroskop | `DriveSafeSensor::calibrateBias`, `read` |
| Pers. 4.8 filter orde pertama | `COMP_FILTER_ALPHA` di `updateOrientation` |
| Pers. 4.9–4.12 adaptive threshold | `CrashDetector::computeFeatures` |
| Pers. 4.13–4.15 G, J, R | `CrashDetector::computeFeatures` |
| Pers. 4.16 f1[n] | `stageEventDetection` |
| Pers. 4.17 f[n] = f1 ∧ (f2 ∨ f3) | `sensorTask`, kasus `ST_VERIFYING` |
| Pers. 4.21–4.22 roll & pitch | `DriveSafeSensor::updateOrientation` |
| Pers. 4.23 rollover & severity | `checkRolloverPersistent`, `classifySeverity` |
| Pers. 4.27–4.31 memori | `EventBuffer`, `DriveSafeStorage` |
| Tabel 4.1 LPF | `DriveSafeSensor::applyLpfConfig` |
| Tabel 4.2 / 5.6 threshold profil | `PROFILE_TABLE` di `config.h` |
| Tabel 4.4 struktur event log | `struct EventLog` |
| Tabel 5.7 threshold insiden | konstanta `SHOCK_*`, `ROLL_*`, `PITCH_*` |
| Sub-bab 5.10 spare time 30 s | `SPARE_OBSERVE/CANCEL/FINALIZE_MS` |
| Bab 8.3 alur komunikasi darurat | `runEmergencyProcedure` |
| Bab 8.5 / 10.2.1 pairing BLE | `bleif.cpp` |

Verifikasi numerik yang sudah dijalankan pada modul logika menghasilkan angka
yang persis sama dengan dokumen: buffer **36.000 byte** (Pers. 4.27) dan beban
memori **6,77 %** dari SRAM 520 KB (Pers. 4.30).

---

## 8. Catatan teknis terhadap dokumen

Delapan hal berikut ditemukan saat implementasi. Semuanya sudah ditangani di
kode, tetapi sebagian perlu dikoreksi juga di laporan sebelum sidang.

**1. Mode NDOF mengunci akselerometer pada ±4 g.** Pada mode fusion, firmware
internal BNO055 memaksa rentang ±4 g. Akibatnya klasifikasi severity Pers. 4.23
tidak akan pernah bisa membedakan *Moderate* (4–7 g) dari *Severe* (≥ 7 g)
karena pembacaan ter-*clipping* di 4 g. Karena itu `SENSOR_MODE` default
disetel ke **AMG** dengan rentang ±16 g, dan orientasi dihitung di ESP32
memakai Pers. 4.21–4.22 plus *complementary filter*. Ubah ke
`SENSOR_MODE_NDOF` di `config.h` jika tim tetap ingin fusion internal, dengan
konsekuensi tersebut.

**2. Bandwidth giroskop 116 Hz tidak tersedia di mode NDOF.** Tabel 4.1 memilih
ACC 62,5 Hz dan GYR 116 Hz, tetapi pada mode fusion BNO055 mengunci giroskop
di 32 Hz dan register `GYR_CONFIG_0` menjadi *read-only*. Nilai Tabel 4.1 hanya
dapat diterapkan pada mode AMG — satu alasan tambahan memilih AMG.

**3. Pers. 4.22 membuat syarat |θ| > 90° mustahil tercapai.** Fungsi
`arctan(ax / √(ay²+az²))` secara matematis terbatas pada rentang −90°…+90°,
sehingga *Pitch Angle Detection* pada Sub-bab 4.5.3 tidak akan pernah aktif
bila hanya memakai akselerometer. Kode mengatasinya dengan mengintegrasikan
giroskop sehingga pitch dapat merentang −180°…+180°. Sebaiknya dokumen
menambahkan penjelasan ini, atau menurunkan ambang pitch ke nilai < 90°.

**4. Threshold jerk 30–50 m/s³ terlalu rendah untuk Ts = 0,01 s.** Pada
100 Hz, perubahan percepatan 0,4 m/s² antar cuplikan saja sudah menghasilkan
J = 40 m/s³. Pengujian numerik menunjukkan polisi tidur ringan menghasilkan
J ≈ 745 m/s³ dan benturan 4,5 g mencapai ≈ 3.500 m/s³. Dengan nilai Tabel 4.2,
tahap 1 akan hampir selalu terpicu. Nilai dokumen tetap dipakai sebagai default
agar konsisten dengan laporan; gunakan perintah `baseline` untuk memperoleh
angka empiris lalu setel `JERK_THRESHOLD_SCALE` dan
`ROTJERK_THRESHOLD_SCALE` di `config.h`. Ini persis langkah yang sudah
direncanakan tim pada Tabel 10.1 tahap 2.

**5. Tabel 4.2 dan Tabel 5.6 memberi nilai G berbeda.** Tabel 4.2 menyebut
mobil > 3 g, motor > 2,5 g, sepeda > 2 g; Tabel 5.6 menghasilkan 4,5 g, 3,2 g,
dan 2,0 g. Kode memakai **Tabel 5.6** karena nilainya diturunkan dari
perhitungan Pers. 5.4–5.8 yang dapat ditelusuri. Salah satu tabel perlu
diselaraskan di dokumen.

**6. Jalur pairing disebut dua cara berbeda.** Bab 8.5 menjelaskan mode Access
Point Wi-Fi dengan server HTTP port 80, sedangkan Tabel 6.1 dan Sub-bab 10.2.1
menetapkan BLE dengan alasan efisiensi daya. Kode mengimplementasikan **BLE**
sesuai keputusan yang lebih baru. Bab 8.5 sebaiknya diperbarui agar tidak
kontradiktif.

**7. Modul GPS disebut NEO-M8N dan NEO-6M bergantian.** Bab 8.2.1 dan Bab 8.3
menulis NEO-6M, sedangkan Tabel 9.1 dan bab lain menulis NEO-M8N. Kode bekerja
untuk keduanya karena memakai NMEA standar, tetapi penamaan di dokumen perlu
diseragamkan.

**8. Klaim recall ≥ 85 % belum dapat diverifikasi dari firmware saja.**
Spesifikasi Tabel 6.2 no.1 membutuhkan data uji berlabel. Firmware sudah
menyimpan setiap kejadian ke `/events.csv` beserta nilai puncak dan *trigger
flags*, sehingga confusion matrix Tabel 4.3 dapat dihitung langsung dari berkas
tersebut setelah rangkaian pengujian Tabel 10.2 selesai.

---

## 9. Daftar perintah serial

```
status                    status sistem lengkap
live                      20 cuplikan fitur real-time
baseline <detik>          rekam manuver normal & usulkan threshold empiris
contacts                  daftar kontak darurat
add <nama> <nomor> <pri>  tambah kontak
del <index>               hapus kontak
profile <sepeda|motor|mobil>
owner <nama>              atur nama pemilik
countdown <detik>         atur durasi hitung mundur (5–120)
calib                     ulangi kalibrasi bias giroskop
sim                       simulasikan pemicu kecelakaan
cancel                    batalkan hitung mundur
sms                       kirim SMS uji ke kontak prioritas
gsm                       segarkan status modul GSM
gps                       data GPS terkini
pair / unpair             hidup/matikan advertising BLE
log                       event log terakhir
mem                       pemakaian memori
help                      bantuan
```

---

## 10. Checklist pengujian (Tabel 10.1 & 10.2)

| Tahap | Perintah / tindakan | Kriteria lulus |
|---|---|---|
| 1. Pembacaan BNO055 | `status`, `live` | laju ≥ 100 Hz, G ≈ 1,00 g saat diam |
| 2. Threshold empiris | `baseline 120` saat berkendara normal | tidak ada pemicu palsu setelah pengali disetel |
| 3. Modul GSM | `gsm`, `sms` | SMS diterima < 30 detik |
| 4. Modul GPS | `gps` di ruang terbuka | galat ≤ ±10 m, sat ≥ 6 |
| 5. Rantai deteksi | `sim` lalu diamkan perangkat | `VERIFY` → `COUNTDOWN` → `ALERT` |
| 6. Pembatalan | `sim` lalu tekan tombol pada detik ke-5 | status menjadi `CANCELLED`, SMS tidak dikirim |
| 7. Rollover | miringkan perangkat > 60° selama > 1,5 detik | langsung masuk `COUNTDOWN` |
| 8. Event log | `log`, `mem` | data tersimpan, beban memori ≈ 6,8 % |
| 9. Pairing | `pair`, sambungkan dari aplikasi | konfigurasi bertahan setelah perangkat dimatikan |
