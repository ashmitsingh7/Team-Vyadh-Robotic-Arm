# Encoder Feedback System

High-resolution encoder feedback subsystem developed for the Team Vyadh robotic arm.

This subsystem was designed to enable precise joint feedback and positioning for IG32 DC encoded motors using ESP32-based processing.

---

## Objectives

The robotic arm required precise positional feedback for stable closed-loop control and repeatable robotic manipulation.

To achieve this, multiple encoder approaches were explored and implemented:

* magnetic encoder systems
* optical encoder systems
* hybrid encoder configurations

---

## Encoder Implementations

### 9048 CPR Magnetic Encoder System

Implemented encoder processing for IG32 motors equipped with 9048 CPR magnetic encoders using ESP32-based interrupt handling and feedback processing.

This configuration was used for precise robotic arm joint positioning while maintaining lightweight embedded processing.

---

### 232464 CPR Optical Encoder System

Developed support for extremely high CPR optical encoder systems reaching approximately 232,464 counts per revolution.

The objective was to experimentally evaluate ultra-high-resolution encoder feedback using only ESP32-based processing.

---

### Hybrid Encoder Architecture

A hybrid encoder feedback approach was explored using:

* motor encoders
* AS5600 magnetic absolute encoders

This enabled both relative and absolute joint positioning information for improved robotic arm feedback reliability.

---

## Mechanical Encoder Mounting

Custom CAD-designed encoder mounting hardware was developed to integrate the AS5600 magnetic absolute encoder into the robotic arm assembly.

The mounting system ensured:

* precise magnet alignment
* stable sensor positioning
* vibration resistance during arm motion

---

## Repository Structure

```text
firmware/ -> encoder firmware implementations
cad/ -> encoder mounting hardware
docs/ -> subsystem documentation
```

---

## Files

### Firmware

* IG32_9048COUNT.ino
* IG32_2LAKHCOUNT.ino
* magneticandopticalencoder.ino
* multi_encoder.ino

### CAD

* encoderholder_base.stl
* enco holder.stl

```
```
