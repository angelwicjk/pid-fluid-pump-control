# 🧪 PID Fluid Pump Control System

> A closed-loop Arduino-based system using PID control for multi-pump liquid dispensing and regulation.  
> Developed for precise flow rate management in real-time applications.

---

## 🚀 Features

- 🔧 PID control loop with dynamic tuning
- 🌡️ Real-time flow rate feedback via analog sensors
- 💧 Supports up to 4 pumps and solenoid valves
- 🧠 Integral gain adaptation and smoothing filter logic
- 📡 UART-based serial communication for data monitoring
- 🛠️ Yalın mimari ile yazılmış doğrudan register seviyesinde kontrol

---

## ⚙️ How It Works

The system reads real-time flow values and adjusts pump output using a PID controller.  
Depending on user input (via Serial), it activates a specific pump and its corresponding valve.  
The output voltage is carefully filtered and smoothed to avoid sudden spikes or oscillations.

### Key Technical Highlights:
- Integral term with saturation handling
- Derivative filtering using a configurable low-pass filter
- Output rate limiting (voltage change clamp)
- Modular control logic for multiple pump paths

---

## 🔌 Hardware Setup

| Component             | Description                            |
|----------------------|----------------------------------------|
| Arduino Mega         | Main MCU platform                      |
| Flow Sensor          | Analog flow input (scaled with offset) |
| Solenoid Valves (x4) | Controlled via digital pins            |
| Custom Pump Driver   | Voltage-controlled via I2C             |
| Power Supply         | 12V for pumps & valves                 |


Use any UART logger or external Python script for real-time visualization.

---

## 📄 License

This project is licensed under the **MIT License** – free for personal or academic use.  
Feel free to fork, modify, or contribute.

---

## 👤 Author

**Hüseyin Bertan Acar**  
📧 bertan_acr@hotmail.com  
📍 İzmir / Turkey  
🔗 [LinkedIn](https://www.linkedin.com/in/huseyin-bertan-acar/)

---

> “My goal is to develop maintainable, testable, and robust embedded systems.”


