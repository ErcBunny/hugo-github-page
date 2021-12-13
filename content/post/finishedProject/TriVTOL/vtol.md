---
title: "A 3-Rotor VTOL UAV"
date: 2021-12-13T13:20:21+08:00
draft: false
description: A project for The Freshman Year Innovation Program of HITSZ (2019)
image: /post/finishedProject/TriVTOL/cover.png
comments: true
license: false
hidden: false
categories:
    - Finished Project
tags:
    - VTOL
    - UAV
---

---
## VIDEO DEMO

{{< bilibili BV1H4411o7Gq 1 >}}

---
## SYSTEM OVERVIEW

* VTOL is achieved by adding a mixing unit to the F4 flight controller running Betaflight Y3 firmware

* The mixing unit takes in PWM outputs of a basic multicopter flight controller and yields signals for VTOL actuators

* We only implemented attitude control. Communication between boards is via PWM

![Illustration of the UAV | ①: Aileron, ②: Rudder & Elevator, ③ & ④: Tilt Servo, ⑤: Motor](/post/finishedProject/TriVTOL/illustration.png)

![System Block Diagram](/post/finishedProject/TriVTOL/overview.png)

![Essential Part of the F1 Mixing Unit Explained](/post/finishedProject/TriVTOL/math.png)

---
## DETAILED REPORT

* [View PDF (Chinese)](https://github.com/ErcBunny/sharedDocs/raw/main/Design%2C%20Control%20and%20Mixing%20of%203-Rotor%20VTOL%20UAVs.pdf)

> Contents of the detailed report are in Chinese as they were made originally to support the Chinese presentation. A rework and translation of illustrations, diagrams and the report is in progress, but don't expect it to be finished soon. Best regards~