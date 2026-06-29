# Disaster Recovery Network Simulation (ns-3)

A network simulation built with **ns-3** to evaluate communication coverage in disaster recovery scenarios using **mobile ad-hoc networks (MANETs)** and the **DSDV routing protocol**.

The simulation models mobile rescue units communicating with fixed Access Points (APs) and measures network coverage under different mobility and routing configurations.

---

## Overview

Natural disasters often compromise existing communication infrastructures.

This project simulates a disaster recovery scenario where mobile wireless nodes move across a geographical area and communicate with multiple emergency Access Points through an ad-hoc wireless network.

The objective is to evaluate how routing and mobility affect the percentage of reachable emergency stations over time.

---

## Features

* 📡 Mobile Ad-Hoc Network (MANET)
* 📶 IEEE 802.11b wireless communication
* 🚶 Random Waypoint Mobility Model
* 🛰 DSDV routing protocol
* 📊 Automatic network coverage analysis
* 📈 Gnuplot visualization of simulation results

---

## Simulation Parameters

| Parameter         |           Value |
| ----------------- | --------------: |
| Mobile Nodes      |              50 |
| Access Points     |              10 |
| Simulation Area   |   5000 x 5000 m |
| Routing Protocol  |            DSDV |
| Mobility Model    | Random Waypoint |
| Wireless Standard |    IEEE 802.11b |

Simulation parameters can be customized through command-line arguments.

---

## Metrics

The simulator evaluates:

* Number of reachable Access Points
* Network coverage percentage
* Packet delivery over time
* Communication timeline between mobile nodes and APs

---

## Technologies

* C++
* ns-3
* IEEE 802.11b
* DSDV Routing
* Gnuplot

---

## Running the Simulation

```bash
./waf --run disaster-recovery-simulation
```

Example:

```bash
./waf --run "disaster-recovery-simulation --nWifis=50 --nAPs=10"
```

---

## Example Output

```text
Reachable APs: 9 / 10

Network Coverage: 90%
```

The simulator also generates:

* `disaster-recovery-simulation.png`
* `disaster-recovery-simulation.plt`

showing the packet reception timeline for each Access Point.

---

## Learning Objectives

This project was developed to explore:

* Wireless network simulation
* Mobile Ad-Hoc Networks (MANET)
* Routing algorithms
* Disaster recovery communication systems
* Performance evaluation using ns-3
