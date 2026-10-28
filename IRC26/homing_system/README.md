# Wrist Differential Homing System

Mechanical homing subsystem developed for the robotic arm wrist differential mechanism.

---

## Problem

The wrist differential mechanism did not contain direct encoder feedback.

Without a repeatable reference position, the wrist orientation could drift over time during operation and startup.

---

## Solution

A limit-switch-based homing mechanism was designed to establish a repeatable zero-position reference.

During startup:

* the wrist mechanism moves toward the homing position
* the limit switch is triggered
* the controller assigns this position as the calibrated zero reference

This enabled repeatable initialization and stable wrist positioning despite the absence of encoder feedback.

---

## Mechanical Design

A custom CAD-designed limit switch holder was developed to:

* accurately position the switch
* ensure reliable triggering
* tolerate vibration during robotic arm motion

---

## Files

### CAD

* final_limitswitchholder.stl

```
```
