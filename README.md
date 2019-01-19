# Filter Bank Multicarrier (FBMC) Transmitter and Receiver
This project contains code for a transmitter and receiver which
- use GNU Radio as a framework
- transfer packetized data analogous to IEEE802.11a WLAN
- use Filter Bank Multicarrier (FBMC) as a modulation scheme
- synchronize packets in frequency domain
- can adapt their spectrum usage to coexisting, independent wireless systems.

## Video Presentation
The system was presented at SDRA 2018: [YouTube Video](https://www.youtube.com/watch?v=2y5V_O9y9V0)

## Installation of gr-fbmc1
```bash
git clone https://github.com/KyrellGod/gr-fbmc.git
cd gr-ieee802-11
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```
## Usage
- TBD
