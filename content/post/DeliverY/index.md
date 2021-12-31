---
title: "Autonomous Delivery Drone"
date: 2021-04-23T14:55:42+08:00
draft: false
description: National Undergraduate Engineering Training Integration Ability Competition (2021)
image: cover.png
comments: true
license: false
hidden: false
categories:
    - Finished_Project
tags:
    - UAV
     
---

---
## VIDEO DEMO

{{< bilibili BV1AV411J7UV 1 >}}

---
## PROJECT OVERVIEW

* Nothing interesting or related to cutting-edge research. Just an engineering level practice

* Some technical approaches:
    * Localization: optical flow, TOF sensor
    * Detection and navigation: YOLO + KCF + Hough Circle Transform on TX2
    * Flight controller: ACFLY ADRC
    * Communication between FC and TX2: velocity command through serial UART

---
## DETAILED REPORT

* [View Control and Electronic Circuit Design Scheme PDF (Chinese)](https://github.com/ErcBunny/sharedDocs/raw/main/Autonomous%20Delivery%20Drone%20electric-control%20report.pdf)

* [View Structure Design Scheme PDF (Chinese)](https://github.com/ErcBunny/sharedDocs/raw/main/Autonomous%20Delivery%20Drone%20hardware%20report.pdf)

> Contents of the detailed report are in Chinese as they were made originally to support the Chinese presentation. A rework and translation of illustrations, diagrams and the report is in progress, but don't expect it to be finished soon. Best regards~