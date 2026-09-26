# 🤖 OSCAR 1.0 — Autonomous Sumo Robot

**OSCAR 1.0** is a 4-wheel-drive autonomous Sumo Robot designed to **detect, chase, and push opponents out of the Dohyo while avoiding the ring boundary.**

## ⚙️ Hardware

| Component | Qty | Purpose |
|---|---:|---|
| Arduino Nano | 1 | Main controller |
| HC-SR04 | 3 | Opponent detection |
| Line Sensors | 3 | Edge detection |
| JGA25-370 Motors | 4 | Movement & pushing |
| L298N | 1 | Motor control |
| Battery + Buck Converter | 1 | Power |

> **No rear ultrasonic sensor and no servo motor are used.**

## 🧠 How It Works

**Edge → Detect → Attack → Search**

- ⚪ **Edge detected** → Avoid
- 🎯 **Opponent ≤ 35 cm** → Attack
- 📡 **Opponent ≤ 100 cm** → Track
- 🔍 **No opponent** → Search

The robot makes decisions every **20 ms** and starts after a **5-second delay**.

## 🔧 Mechanical Design

The chassis focuses on:

**Traction + Stability + Pushing Force**

It uses **4-wheel drive**, a curved front pushing structure, and a protective lower body.

## 🧪 Development

The project went through multiple iterations involving:

- Wiring & soldering
- Sensor failures
- Motor driver problems
- Mechanical issues
- Wheel problems
- Testing & debugging

> **“After 1,000 uploads, rewiring every single pin, and deeply questioning our life choices... IT FINALLY WORKS! 🔥”**

## 🚀 Control System

```text
SENSORS
   ↓
DECISION MAKING
   ↓
MOTOR CONTROL
   ↓
MOVEMENT
   ↓
PUSH
