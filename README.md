# TOPA - UAV Positioning and Optimization Framework with Obstacle-Aware Modeling Ussing GEKKO

## Overview

TOPA (Trajectory Optimization for Positioning with obstacle Awareness) is a comprehensive framework for optimizing UAV (Unmanned Aerial Vehicle) positioning in wireless communication networks while considering physical obstacles. The project combines network simulation using ns-3 with mathematical optimization techniques to find optimal UAV positions that maximize coverage while maintaining line-of-sight (LoS) conditions.

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Configuration](#configuration)
- [Examples](#examples)
- [Technical Details](#technical-details)
- [Contributing](#contributing)
- [License](#license)
- [Citation](#citation)
- [Contact](#contact)

## Features

- **Obstacle-Aware Positioning**: Automatically calculates optimal UAV positions considering building obstacles and LoS conditions
- **ns-3 Network Simulation**: Comprehensive network performance evaluation using ns-3 simulator
- **Multi-User Support**: Handles multiple User Equipment (UE) with different requirements
- **Propagation Models**: Implements both LoS and NLoS propagation models (ITU-R 1411)
- **Real-time Throughput Monitoring**: Tracks network performance metrics during simulation
- **3D Positioning**: Full 3D coordinate system for UAVs, UEs, and obstacles
- **Optimization Engine**: GEKKO-based optimization for finding minimal transmission power and optimal positions

## Architecture

The project consists of two main components:

1. **ns-3 Evaluation Module**: Network simulation and performance evaluation
2. **Optimization Module**: Mathematical optimization for UAV positioning

```
TOPA/
├── ns-3-evaluation/
│   ├── PositioningV13.cc    # Main ns-3 simulation
│   ├── coords.cc/h          # Coordinate management
│   ├── loss_calc.h          # Loss calculation utilities
│   └── data files           # AP and UE positions
└── optimization/
    ├── app.py               # Main optimization engine
    ├── obstacle.py          # Obstacle modeling
    └── data files           # UE and obstacle definitions
```

## Prerequisites

### For ns-3 Evaluation Module
- ns-3 simulator (version 3.38 or higher)
- C++ compiler with C++11 support
- Python 3.x for analysis scripts

### For Optimization Module
- Python 3.7+
- NumPy
- Shapely
- GEKKO optimization package

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/kamranshafafi/TOPA.git
cd TOPA
```

### 2. Install ns-3 (if not already installed)

```bash
# Download and install ns-3
wget https://www.nsnam.org/releases/ns-3.38/ns-allinone-3.38.tar.bz2
tar xjf ns-allinone-3.38.tar.bz2
cd ns-allinone-3.38/ns-3.38
./waf configure --enable-examples
./waf
```

### 3. Install Python Dependencies

```bash
pip install -r requirements.txt
```

Create a `requirements.txt` file with:
```
numpy>=1.19.0
shapely>=1.7.0
gekko>=1.0.0
matplotlib>=3.3.0
```

### 4. Copy ns-3 Files

Copy the ns-3 evaluation files to your ns-3 scratch directory:
```bash
cp ns-3-evaluation/* /path/to/ns-3.38/scratch/UAV/
```

## Project Structure

### ns-3 Evaluation Module Files

- **PositioningV13.cc**: Main simulation file implementing:
  - WiFi 802.11ac configuration
  - Building and obstacle modeling
  - LoS/NLoS channel conditions
  - Throughput measurement
  - Power adjustment for NLoS links

- **coords.cc/h**: Coordinate management system:
  - 3D coordinate class
  - File I/O for coordinate data
  - Coordinate list management

- **AP.txt**: Access Point coordinates (x, y, z format)
- **ue.txt**: User Equipment coordinates

### Optimization Module Files

- **app.py**: Main optimization engine featuring:
  - GEKKO-based optimization
  - SNR-based radius calculation
  - Angle constraint enforcement
  - Iterative power optimization

- **obstacle.py**: Obstacle modeling:
  - Building height extraction
  - Polygon-based obstacle representation
  - Distance and angle calculations

- **UEs.txt**: UE definitions (id, x, y, z, SNR)
- **obstacle.txt**: Building corner coordinates

## Usage

### Running the Optimization Module

1. Define your UEs in `UEs.txt`:
```
id,x,y,z,snr
0,-10,0,1,11
1,10,0,1,11
```

2. Define obstacles in `obstacle.txt`:
```
5,5,20
5,-5,20
-5,-5,20
-5,5,20
```

3. Run the optimization:
```bash
cd optimization
python app.py
```

Output will show:
- Minimum transmission power required
- Optimal UAV position (x, y, z)

### Running ns-3 Simulation

1. Prepare coordinate files in the scratch directory
2. Compile and run:
```bash
cd /path/to/ns-3.38
./waf --run "UAV/PositioningV13"
```

3. Monitor output for:
- Channel conditions (LoS/NLoS)
- Path loss calculations
- Real-time throughput
- Average throughput

## Configuration

### Key Parameters in ns-3 Simulation

```cpp
#define CHANNEL_BW 160           // Channel bandwidth in MHz
#define CHANNEL_NUMBER 50        // WiFi channel number
#define PacketSize 1024          // Packet size in bytes
#define Simulation_Stop_Time 130 // Simulation duration
```

### Optimization Parameters

```python
# In app.py
K = -20 * log10((4 * pi) / (3 * 10 ** 8)) - 20 * log10(5250 * 10 ** 6) - (-85)
# Frequency: 5250 MHz (Channel 50)
# Reference power: -85 dBm
```

## Examples

### Example 1: Basic UAV Positioning

```python
# Define 4 UEs in a square pattern
# UEs.txt:
0,-10,0,1,11
1,10,0,1,11
2,0,10,1,14
3,0,-10,1,14

# Define a building obstacle
# obstacle.txt:
5,5,20
5,-5,20
-5,-5,20
-5,5,20

# Run optimization
python app.py
# Output: PT=15, Position=[0.5, 0.5, 25.3]
```

### Example 2: ns-3 Throughput Evaluation

```bash
# Configure AP position
echo "-0.49743364333, -10.1373293337, 25.215211983" > AP.txt

# Configure UE positions
echo "0,20,1" > ue.txt
echo "0,-15,1" >> ue.txt

# Run simulation
./waf --run "UAV/PositioningV13"
```

## Technical Details

### Propagation Models

1. **LoS Model**: ITU-R 1411 LoS propagation model
2. **NLoS Model**: ITU-R 1411 NLoS over-rooftop propagation model

### Optimization Constraints

- Coverage constraint: All UEs must be within communication range
- Height constraint: UAV must fly above building height
- Angle constraint: Maintain minimum elevation angle to avoid obstacles

### Performance Metrics

- Throughput (Mbps)
- Path loss (dB)
- SNR (dB)
- Transmission power (dBm)

## Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Coding Standards

- C++ code should follow ns-3 coding style
- Python code should follow PEP 8
- Include appropriate comments and documentation
- Add unit tests for new features

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use TOPA in your research, please cite:

```bibtex
@INPROCEEDINGS{10539557,
  author={Shafafi, Kamran and Coelho, André and Campos, Rui and Ricardo, Manuel},
  booktitle={2023 IEEE 9th World Forum on Internet of Things (WF-IoT)}, 
  title={Joint Traffic and Obstacle-Aware UAV Positioning Algorithm for Aerial Networks}, 
  year={2023},
  volume={},
  number={},
  pages={1-6},
  keywords={Wireless communication;Aggregates;Urban areas;Line-of-sight propagation;Quality of service;Autonomous aerial vehicles;Throughput;Unmanned Aerial Vehicles;Aerial Networks UAV Placement;Positioning Algorithm;LoS communications;Obstacle Detection},
  doi={10.1109/WF-IoT58464.2023.10539557}}
```

## Contact

- **Author**: Kamran Shafafi
- **Email**: kamranshafafi@gmail.com
- **Project Link**: [https://github.com/kamranshafafi/TOPA](https://github.com/kamranshafafi/TOPA)

## Acknowledgments

- ns-3 development team for the network simulator
- GEKKO developers for the optimization framework
- Contributors and testers

---

**Note**: This project is under active development. Please check for updates regularly.
- GEKKO developers for the optimization framework
- Contributors and testers

---

**Note**: This project is under active development. Please check for updates regularly.
