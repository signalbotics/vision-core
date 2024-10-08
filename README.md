
# Vision Core SDK

Welcome to the Vision Core SDK! This SDK provides state-of-the-art libraries for mono depth estimation, stereo depth estimation, segmentation, and firmware support for Nreal glasses. It's optimized for use on embedded devices such as NVIDIA Jetson, making it perfect for real-time applications in robotics, AR/VR, and more.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
  - [Mono Depth Estimation](#mono-depth-estimation)
  - [Stereo Depth Estimation](#stereo-depth-estimation)
  - [Segmentation](#segmentation)
  - [Firmware for Nreal Glasses](#firmware-for-nreal-glasses)
- [Environment Setup](#environment-setup)
- [Building from Source](#building-from-source)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Mono Depth Estimation**: Accurate depth maps from a single image using state-of-the-art neural networks.
- **Stereo Depth Estimation**: Real-time depth estimation using stereo vision for applications requiring depth perception.
- **Segmentation**: High-performance image segmentation for identifying objects and environments.
- **Firmware for Nreal Glasses**: Customized firmware to enhance Nreal glasses with advanced vision capabilities.

## Installation

### Prerequisites

- **Ubuntu 20.04/22.04**: This SDK supports both Ubuntu 20.04 (Focal) and Ubuntu 22.04 (Jammy).
- **OpenCV**: Ensure that OpenCV (version >= 4.5) is installed.
- **CUDA**: Required for running models on NVIDIA GPUs.

### Installing Vision Core SDK

To install Vision Core SDK via APT, follow these steps:

1. **Add the APT repository**:

   ```bash
   echo "deb http://packages.signalbotics.com jammy main" | sudo tee /etc/apt/sources.list.d/vision-core.list
   wget -O - http://packages.signalbotics.com/public.key | sudo apt-key add -
   sudo apt update
   ```

2. **Install the SDK**:
   ```bash
   sudo apt install vision-core
   ```

## Usage

### Mono Depth Estimation

To perform mono depth estimation, use the provided example:

```bash
cd /usr/local/vision_core/examples/mono
mkdir build && cd build && cmake .. && make -j$(nproc)
./mono_demo_ros --use_topic # use ros image topic eg. zed camera
or
./mono_demo_ros # use /dev/video0 camera
```

### Stereo Depth Estimation

Run the stereo depth estimation example as follows:

```bash
cd /usr/local/vision_core/examples/stereo_depth
./stereo_depth_demo --left <left_image> --right <right_image>
```

### Segmentation

To perform image segmentation, use the segmentation demo:

```bash
cd /usr/local/vision_core/examples/segmentation
./segmentation_demo --input <input_image>
```

### Firmware for Nreal Glasses

To update or install firmware on Nreal glasses:

```bash
cd /usr/local/vision_core/firmware/nreal
./install_firmware.sh
```

## Environment Setup

The SDK requires specific environment variables for optimal performance. These are automatically set during installation:

- `VISION_CORE_HOME`: Set to `/usr/local/vision_core`
- `PATH`: Includes `$VISION_CORE_HOME/bin`

If you need to manually set up the environment:

```bash
export VISION_CORE_HOME=/usr/local/vision_core
export PATH=$PATH:$VISION_CORE_HOME/bin
```

## Building from Source

To build the Vision Core SDK from source:

1. **Clone the repository**:

   ```bash
   git clone https://github.com/yourusername/vision-core.git
   cd vision-core
   ```

2. **Install dependencies**:

   ```bash
   sudo apt install build-essential cmake libopencv-dev
   ```

3. **Build the SDK**:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   ```

## Contributing

Contributions are welcome! Please fork the repository, create a branch, and submit a pull request.

1. Fork the repo.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a pull request.

## License

This project is licensed under the Attribution-NonCommercial 4.0 International - see the [LICENSE](LICENSE.md) file for details.