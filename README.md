# 🔥 Solder Reflow Plate (Bare-Metal AVR)

This project is a firmware rewrite of the original [AfterEarthLTD/Solder-Reflow-Plate](https://github.com/AfterEarthLTD/Solder-Reflow-Plate).  
The original version was Arduino-based — this one is written in **pure AVR C 
(bare metal)** for better performance and control.

A custom **KiCad PCB** will be created for this version in the future.

---

## 🧰 Requirements

Before building, update your system:
```bash
sudo apt update && sudo apt upgrade -y
```

Install the AVR toolchain:
```bash
sudo apt install gcc-avr binutils-avr avr-libc avrdude make
```

---

## ⚙️ Build

Go to the `firmware/` directory and run:
```bash
make
```

This will compile the firmware and generate:
- `main.o` – object file 
- `main.elf` – compiled binary 
- `main.hex` – flashable firmware file

---

## 🔥 Flash to MCU

Connect your **USBasp** (or other AVR programmer) and run:
```bash
make flash
```
or manually:
```bash
avrdude -c usbasp -p m32 -U flash:w:main.hex
```

---

## 🧹 Clean Build Files
To remove all generated files:
```bash
make clean
```

---

## 🧩 Notes
- Target MCU: **ATmega32A**
- Clock source: **Internal or external depending on fuses**
- Code is written in **standard C (no Arduino framework)**

---

## 📘 License
This project is open-source and follows the same license as the original
repository.
