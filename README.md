# Two-Stage Stochastic Generation Expansion Planning with Hydrogen and Storage

A small-scale implementation of a **two-stage stochastic MILP** for power system expansion planning, including thermal rehabilitation, battery storage, and a full hydrogen vector (electrolyzer, fuel cell, hydrogen storage). 

Developed as part of research on system flexibility and published/submitted in 2026.

![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)
![PuLP](https://img.shields.io/badge/PuLP-2.x-orange.svg)
![Google Colab](https://img.shields.io/badge/Run%20in-Colab-brightgreen.svg)

## 📋 Overview

This project implements a **two-stage stochastic mixed-integer linear programming (MILP)** model for Generation Expansion Planning (GEP) that co-optimizes:

- Rehabilitation of existing thermal generators (capacity & ramp rate)
- Battery energy storage capacity
- Hydrogen assets: electrolyzer, fuel cell, and hydrogen storage

**Key Features:**
- Solved using open-source **PuLP** + **CBC** solver (runs in Google Colab)
- Two scenarios, two time steps (demonstration case)
- Novel **logistic demand envelope** flexibility sensitivity metric
- Fully reproducible and open-source

## 📊 Main Results (Test Case)

- **Optimal Investments**: Rehabilitation of G2 + **80 MW Fuel Cell**
- **Battery & H₂ Storage**: 0 MW/MWh (due to short horizon & low variability)
- **Expected Operational Cost**: **11,022 k$**
- Fuel cell primarily used for **upward reserve** provision

## 🚀 Quick Start

### Run in Google Colab (Recommended)

[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/farazaghajani-eng/flexibility-pocket-robust-ml-/blob/main/hydro_flex.py)

### Local Installation

```bash
git clone https://github.com/farazaghajani-eng/flexibility-pocket-robust-ml-.git
cd flexibility-pocket-robust-ml-
pip install pulp pandas matplotlib seaborn
python hydro_flex.py
