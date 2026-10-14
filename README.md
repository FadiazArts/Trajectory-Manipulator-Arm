# FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D Simulation

[![Vercel Deployment](https://img.shields.io/badge/Vercel-Deployed-black?style=for-the-badge&logo=vercel)](https://trajectory-manipulator-arm.vercel.app)
[![GitHub Repo](https://img.shields.io/badge/GitHub-Repository-181717?style=for-the-badge&logo=github)](https://github.com/FadiazArts/Trajectory-Manipulator-Arm)

🌐 **Live Demo**: [https://trajectory-manipulator-arm.vercel.app](https://trajectory-manipulator-arm.vercel.app)

Aplikasi simulasi interaktif 3D untuk robot manipulator artikulasi 3-DOF dengan sistem aktuator hidrolik non-floating dan interpolasi trayektori multi-mode.

Proyek ini telah dimodularisasi secara profesional dengan memisahkan kode menjadi lapisan **HTML**, **CSS**, dan **JavaScript (ES Modules)** sesuai dengan standar rekayasa perangkat lunak modern (*Clean Architecture*, *Separation of Concerns*, dan *Single Responsibility Principle*).

---

## 📁 Struktur Direktori Proyek

```text
├── index.html                                        # Entry point HTML utama yang bersih dan semantik
├── simulasi_trayektori_manipulator_robotik_3d (3).html # Entry point alternatif (sinkron dengan index.html)
├── package.json                                      # Konfigurasi package & scripts runner
├── README.md                                         # Dokumentasi teknis & arsitektur proyek
│
├── css/                                              # Modular CSS Architecture
│   ├── variables.css                                 # Design tokens, palette warna, gradient, reset & font
│   ├── layout.css                                    # Grid shell layout, icon rail navigasi & KPI header
│   ├── controls.css                                  # Control panel, folder tabs, sliders, preset buttons
│   ├── viewport.css                                  # 3D canvas viewport, HUD camera tools, transport dock
│   ├── telemetry.css                                 # Bento telemetry dashboard, spline chart, ring gauges, SE(3) matrix
│   └── style.css                                     # Master stylesheet (mengimpor seluruh modul CSS)
│
└── js/                                               # Modular JavaScript (ES6+ Modules)
    ├── config.js                                     # Konstanta matematika (DEG2RAD, RAD2DEG), preset industri, kamera
    ├── state.js                                      # Central reactive application state store (Single Source of Truth)
    ├── kinematics.js                                 # Kinematika Maju (FK), Kinematika Balik (IK), SE(3) transform, koordinat
    ├── trajectory.js                                 # Interpolasi trayektori (Quintic C², Cubic C¹, LSPB, Cartesian Linear), CSV export
    ├── scene.js                                      # Inisialisasi Three.js, kamera, OrbitControls, pencahayaan, workspace envelope
    ├── robotModel.js                                 # Geometri PBR robot 3D, silinder hidrolik teleskopik non-floating, gripper, marker
    ├── chart.js                                      # Rendering spline chart 2D Canvas berkinerja tinggi dengan floating callout badge
    ├── ui.js                                         # Event handlers, interaksi input/slider, sinkronisasi DOM & telemetry live
    └── main.js                                       # App bootstrap, animation loop, window facades & resize handling
```

---

## 🧩 Arsitektur Modular

### 1. Lapisan Struktur (HTML)
- **`index.html`**:
  Berisi struktur semantik DOM aplikasi tanpa inline CSS atau script monolithic.
  Menggunakan elemen navigasi `<nav>`, header `<header>`, panel kontrol `<aside>`, viewport 3D `<main>`, dan dashboard telemetri `<aside>`. Seluruh aset visual dan logika dihubungkan melalui `css/style.css` dan `<script type="module" src="js/main.js">`.

### 2. Lapisan Gaya (CSS)
Dipisahkan menjadi 5 modul tematik dan 1 master stylesheet:
- **`css/variables.css`**: Definisi CSS Custom Properties (`:root`), palet warna referensi (Indigo, Sky, Mint, Cyan, Purple, Orange, Coral), gradasi, font Poppins, radius, bayangan, dan reset CSS.
- **`css/layout.css`**: Grid container utama (`.app-shell`), icon rail sebelah kiri (`.nav-rail`), dan top KPI header bar (`.top-kpi-bar`).
- **`css/controls.css`**: Kolom kontrol sebelah kiri (`.left-control-col`), folder tabs, input sliders dengan custom track & thumb, kartu perbandingan mini, dan industrial preset grid.
- **`css/viewport.css`**: Area 3D canvas viewport, toolbar preset kamera overlay (`.viewport-hud`), dan floating transport timeline dock (`.transport-dock`) dengan stepper nodes.
- **`css/telemetry.css`**: Kolom telemetri bento kanan (`.right-bento-col`), kartu koordinat TCP indigo, kanvas spline chart, pengukur multi-ring konsentris SVG, pilar hidrolik, 8-bar capsule equalizer, dan matriks transformasi homogen 4x4 $SE(3)$.
- **`css/style.css`**: Master stylesheet yang menggabungkan seluruh modul di atas melalui `@import`.

### 3. Lapisan Logika & Engine (JavaScript ES Modules)
Setiap file JavaScript memiliki satu tanggung jawab yang terisolasi (*Single Responsibility*):

| Modul | Tanggung Jawab Utama |
|---|---|
| **`js/config.js`** | Menyimpan konstanta konversi sudut (`DEG2RAD`, `RAD2DEG`), preset industri (Pick & Place, High Zenith, Vertical Weld, Wide Sweep), dan preset kamera. |
| **`js/state.js`** | Menyimpan state global aplikasi secara terpusat (*Single Source of Truth*), mencakup dimensi lengan (`L1`, `L2`, `L3`), posisi waypoint, status aktuator, dan konfigurasi animasi. |
| **`js/kinematics.js`** | Fungsi matematika murni untuk Kinematika Maju (*Forward Kinematics* - DH/Geometris), Kinematika Balik analitis (*Inverse Kinematics*) dengan validasi batas jangkauan (*reachability clamp*), dan konversi sistem koordinat robot ke Three.js. |
| **`js/trajectory.js`** | Mesin interpolasi waktu (Quintic Polinomial $C^2$, Cubic Polinomial $C^1$, LSPB / *Trapezoidal Velocity*, Cartesian Straight-line IK), kalkulasi kecepatan/percepatan, sampling progres, dan ekspor data CSV. Menggunakan pola *Observer* (`onTrajectoryComputed`) untuk decoupled eventing. |
| **`js/scene.js`** | Setup Three.js: scene, kamera perspektif, WebGL renderer (shadow map, filmic tone mapping), OrbitControls, pencahayaan multi-arah (Directional, Hemisphere, Ambient), lantai grid, triad koordinat RGB, dan visualisasi *workspace dome envelope*. |
| **`js/robotModel.js`** | Pembuatan assembly 3D robot lengan artikulasi: base pedestal, rotating turret trunnion, upper arm beam, forearm horn clevis, gripper paralel 2-jaw, dan **aktuator hidrolik teleskopik ganda non-floating** yang terkunci rapat secara matematis pada poros clevis pin. |
| **`js/chart.js`** | Mesin penggambaran grafik spline 2D Canvas dengan dukungan Retina High-DPI (`devicePixelRatio`), kurva kecepatan/posisi halus, area gradien, playhead vertikal interaktif, dan badge nilai floating teal (`1.802`). |
| **`js/ui.js`** | Logika interaksi antarmuka pengguna: manipulasi tab koordinat, drag slider dinamis, tombol preset, kontrol playback timeline (play, pause, scrub, reset, ping-pong, speed), hotkey keyboard spasi, dan sinkronisasi data telemetri live ke seluruh elemen dashboard. |
| **`js/main.js`** | Titik masuk utama aplikasi (*Bootstrap*): inisialisasi modul, lifecycle loop animasi (`requestAnimationFrame`), penanganan perubahan ukuran jendela browser (*window resize*), dan integrasi fungsi ke global `window` agar semua event handler inline tetap kompatibel 100%. |

---

## 🚀 Cara Menjalankan Aplikasi

Karena menggunakan modul ES6 modern (`<script type="module">`), aplikasi perlu disajikan melalui HTTP server lokal untuk menghindari batasan CORS peramban pada protokol `file://`.

### Opsi 1: Menggunakan Node.js / npx (Rekomendasi)
```bash
# Menggunakan npx serve
npx serve .

# Atau jalankan script package.json
npm start
```
Buka browser pada alamat yang ditampilkan (contoh: `http://localhost:3000`).

### Opsi 2: Menggunakan Python 3
```bash
python3 -m http.server 8000
```
Buka peramban pada `http://localhost:8000`.

### Opsi 3: Menggunakan Ekstensi VS Code "Live Server"
Klik kanan pada file `index.html` di VS Code dan pilih **"Open with Live Server"**.

---

## 🛠️ Keunggulan Arsitektur Setelah Modularisasi

1. **Maintainability & Readability**: File asli yang awalnya berupa 4.000 baris kode monolitik kini terorganisir rapi dalam modul-modul kecil berukuran 80–400 baris yang fokus pada satu tugas.
2. **Testability**: Logika matematika seperti `kinematics.js` dan `trajectory.js` dapat diuji (*unit test*) langsung menggunakan Node.js tanpa membutuhkan runtime browser atau dependensi eksternal.
3. **Decoupled Architecture**: Logika komputasi trayektori tidak lagi terikat keras ke logika rendering 3D atau canvas berkat penerapan *Observer Pattern*.
4. **Zero Regression**: Semua 31 fungsi interaksi tombol dan input yang dipanggil dari HTML tetap bekerja normal tanpa ada fitur yang rusak atau hilang.
5. **Industry Standards**: Mengikuti struktur aplikasi web modern berbasis ES Modules standar web tanpa memerlukan bundling yang rumit.
