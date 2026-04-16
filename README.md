# BLECompass + KrakenSDR Integration (Raspberry Pi 5)

This project enables real-time compass heading updates from a **Raspberry Pi 5** over **Bluetooth Low Energy (BLE)**.

It integrates:
- **IMU sensors** (magnetometer + accelerometer)
- **KrakenSDR DOA processing**
- **BLE communication**
- **Android compass visualization**

The system is designed for **radio direction finding (foxhunting)** and can run fully **headless** on the Raspberry Pi.

---

# 🚀 Features

- BLE communication between Raspberry Pi and Android device  
- Real-time heading updates  
- IMU-based heading (fallback mode)  
- KrakenSDR DOA-based heading  
- IMU + Kraken fusion for absolute direction  
- Headless operation (no monitor required)  
- GPIO shutdown button support  

---

# 📁 Project Files

## `KrakenSDR_ESPRIT.py`
- Modified KrakenSDR processing script  
- Implements DOA using the ESPRIT algorithm  
- Outputs real-time direction to:
  ```text
  live_doa.json
