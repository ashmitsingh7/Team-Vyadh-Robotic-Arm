# Team Vyadh - Robotic Arm

Competition-oriented robotic arm subsystem archive developed for Mars rover applications under Team Vyadh, VIT Vellore.

This repository documents the embedded, mechanical, and control subsystems developed for a multi-DOF robotic manipulator used in rover operations and field deployment.

---

# Robotic Arm Platform

## Competition Deployment

![img](images/people/WhatsApp%20Image%202026-06-06%20at%2012.55.33%20PM.jpeg)

![img](images/people/WhatsApp%20Image%202026-06-06%20at%2012.29.01%20PM.jpeg)

![img](images/people/WhatsApp%20Image%202026-06-06%20at%2012.29.01%20PM%20(1).jpeg)

---

# Arm Assembly & Mechanisms

![img](images/arm/WhatsApp%20Image%202026-06-06%20at%2012.29.00%20PM.jpeg)

![img](images/arm/WhatsApp%20Image%202026-06-06%20at%2012.29.00%20PM%20(1).jpeg)

![img](images/arm/WhatsApp%20Image%202026-06-06%20at%2012.55.32%20PM.jpeg)

![img](images/arm/WhatsApp%20Image%202026-06-06%20at%2012.55.32%20PM%20(1).jpeg)

---

# Team Vyadh Experience

Served as a Senior Core Member in the Robotic Arm Domain at Team Vyadh, contributing to the development, integration, and deployment of embedded and mechanical subsystems for a competition-oriented multi-DOF robotic manipulator designed for Mars rover applications.

Worked extensively across:
- embedded systems
- robotic arm control
- encoder feedback architectures
- mechanical subsystem design
- PCB and electronics integration
- competition deployment infrastructure

The work involved both low-level firmware development and real-world hardware deployment under dynamic operating conditions.

---

# Major Contributions

## Encoder Feedback Systems

Developed and experimented with high-resolution encoder feedback systems for IG32 DC encoded motors using ESP32-based processing.

Implemented:
- 9048 CPR magnetic encoder systems
- ~232,464 CPR optical encoder handling
- hybrid encoder architectures combining motor encoders and AS5600 magnetic absolute encoders

Designed custom encoder mounting hardware for precise sensor alignment and vibration-resistant operation.

---

## Wrist Differential Homing System

Developed a limit-switch-based homing mechanism for the wrist differential assembly.

Since the wrist differential lacked direct encoder feedback, a homing-based initialization system was implemented to establish repeatable zero-position calibration during startup.

Designed and fabricated custom mounting hardware for reliable switch triggering under robotic arm motion and vibration.

---

## Mechanical Mounting Infrastructure

Designed multiple custom robotic arm mounting systems for subsystem integration and deployment reliability.

This included:
- MD10C motor driver mounts
- PCB/protoboard retention systems
- wire management and routing structures

The PCB retention system was specifically designed with controlled sliding tolerances to prevent PCB displacement during robotic arm motion.

These mounts were deployed on the final competition robotic arm platform.

---

## Embedded Systems & Control

Worked on embedded subsystem development for robotic arm operation, including:
- ESP32-based encoder processing
- interrupt-driven feedback systems
- joint positioning logic
- calibration workflows
- modular embedded integration

Contributed to subsystem integration and testing under real deployment conditions.

---

# System Overview

The robotic arm was developed as a modular manipulation platform capable of:
- precise joint positioning
- encoder-based feedback control
- wrist homing and calibration
- modular electronics integration
- vibration-resistant operation

The project involved the integration of embedded firmware, custom mechanical hardware, and robotic subsystem design for real-world rover deployment.

---

# Technologies Used

## Embedded Systems
- ESP32
- Encoder interfaces
- Interrupt-driven feedback systems

## Robotics & Control
- Closed-loop feedback systems
- Joint calibration
- Differential wrist mechanisms

## Mechanical Design
- Fusion360
- 3D printed subsystem hardware
- Modular mounting systems

---

# Repository Structure

```text
encoder_system/ -> encoder feedback subsystem
homing_system/ -> wrist homing subsystem
mounts/ -> robotic arm mounting infrastructure
pcb/ -> PCB related assets
control/ -> robotic arm control systems
communication/ -> communication systems
images/ -> subsystem and deployment images
```

---

# Deployment Context

These subsystems were developed as part of Team Vyadh's rover robotic arm platform and were designed for real-world deployment under dynamic robotic operating conditions.

The repository serves as an engineering archive documenting subsystem development, experimentation, and deployment-oriented design decisions.