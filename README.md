# Filter Bank Multicarrier (FBMC) Transmitter and Receiver
This project contains code for a transmitter and receiver which
- use GNU Radio as a framework
- transfer packetized data analogous to IEEE 802.11a WLAN
- use Filter Bank Multicarrier (FBMC) as a modulation scheme
- synchronize to data packets in frequency domain
- can adapt their spectrum usage to coexisting, independent wireless systems.

## Presentation
The system was presented at SDRA 2018: [YouTube Video](https://www.youtube.com/watch?v=2y5V_O9y9V0)

## Installation of gr-fbmc
```bash
git clone https://github.com/maxpenner/gr-fbmc.git
cd gr-fbmc1
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```
## Usage
Most parameters for system configuration can be found in ```lib/utils```.
### Run in Simulation
The system can be started in a simulation environment without using any radio hardware. To start a simulation with a transmitter and a receiver in the same flowgraph, the files ```examples/freq_sync/freq_simulation.grc``` (frequency domain sync) or ```examples/time_sync/time_simulation.grc``` (time domain sync) can be used. To feed data to the transmitter *(127.0.0.1 : 8888)* the file ```examples/udp_source.grc``` is provided.
### Run on Hardware
The system was tested with two USRPs N210 and an Intel i7-4770 at the receiver side.

On one computer connected to an USRP N210, start ```examples/transmitter.grc```. The transmitter opens a socket *(127.0.0.1 : 8888)* which can be used to feed a binary video stream to the modulation chain. VLC player can be used for this purpose.

On a second computer also connected to an USRP N210, start ```examples/freq_sync/freq_receiver.grc``` or ```examples/time_sync/time_receiver.grc```. If data arrives at the receiver and decoding is successful, the decoded data is output through a socket *(127.0.0.1 : 8889)*.

## Publications
A complete description of the algorithms used for frequency synchronization can be found here [https://core.ac.uk/download/pdf/130519465.pdf](https://core.ac.uk/download/pdf/130519465.pdf) **(Analysis of frequency domain frame detection
and synchronization in OQAM-OFDM systems)**.

## Troubleshooting
After installation the simulations should run without further adjustments. If radio hardware is used the N210 should be prefered due to its precise crystal oscillator. Otherwise, too many packets will be lost during transmission to enable a stable video transmission.

If no N210 is available, the packet size at both the transmitter and the receiver must be reduced to not more than 20 symbols. This way the negative influence of the carrier frequency offset is mitigated.

If you have any questions, feel free to contact me *(maxim.penner@ikt.uni-hannover.de)*.
