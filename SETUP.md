# Setting up and running the EROAS simulation

This runs the original EROAS navigation (sonar-image gap search + ST-CBF) on
the DAVE multibeam sonar in Gazebo Harmonic, inside a Docker container. The
host only needs the NVIDIA driver, Docker and the NVIDIA Container Toolkit.

## Requirements

- Ubuntu 22.04 or 24.04 with a desktop (the Gazebo window and RViz open on it)
- NVIDIA GPU, GTX 10xx (Pascal) or newer, 4 GB VRAM or more
- About 25 GB free disk space
- Internet access for the first build

## 1. NVIDIA driver

Check whether it is already installed:

```bash
nvidia-smi
```

If that prints your GPU, go to step 2. Otherwise:

```bash
sudo ubuntu-drivers install
sudo reboot
```

Use driver 535 or newer; the container's CUDA 12.6 needs at least 525.

## 2. Docker

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin

# run docker without sudo, then log out and back in
sudo usermod -aG docker $USER
```

After logging back in, check:

```bash
docker run --rm hello-world
```

## 3. NVIDIA Container Toolkit (GPU inside Docker)

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Check that a container sees the GPU:

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

## 4. Build

From this folder (copy it anywhere on the machine first, not onto a FAT/exFAT
USB drive):

```bash
./run_eroas.sh setup
```

This builds the Docker image `eroas:jazzy` (ROS 2 Jazzy, Gazebo Harmonic, CUDA;
several GB of downloads), creates the container `eroas` with this folder
mounted at `/root/eroas_ws/src/EROAS`, and builds the workspace in it. It only
needs to run once. After changing code, rebuild with `./run_eroas.sh build`.

## 5. Run

```bash
./run_eroas.sh start
```

The script starts the simulation, waits for RexROV2 to spawn (about 20 s), then
opens the Gazebo window and RViz with the sonar fan image. The vehicle sets off
by itself toward the World A waypoints.

| Command | What it does |
|---|---|
| `./run_eroas.sh start` | simulation + Gazebo window + RViz, World A |
| `./run_eroas.sh start --world world_b` | another world (`world_a_figure5`, `world_b`, `blue_blocks`) |
| `./run_eroas.sh start --headless` | no windows |
| `./run_eroas.sh start navigation:=port` | any launch argument as `name:=value` |
| `./run_eroas.sh teleop` | stop the EROAS nodes and drive by keyboard |
| `./run_eroas.sh logs` | follow the launch log |
| `./run_eroas.sh stop` | stop the simulation |
| `./run_eroas.sh down` | stop the simulation and the container |
| `./run_eroas.sh shell` | shell inside the container |

Teleop keys: `w/s` forward/back, `a/d` turn, `q/e` strafe, `r/f` up/down,
`space` stop, `i/k` sonar tilt up/down 5°, `o` tilt back to 0°, `x` quit.
Run `./run_eroas.sh stop` and `start` again to bring EROAS back.

Logs inside the container: `/root/runs/launch.log`, `gui.log`, `rviz.log`.

## Troubleshooting

**`Failed to initialize NVML: Unknown Error` / sonar not starting.** The
container lost the GPU (happens after some host driver or systemd reloads):
`docker restart eroas`, then `./run_eroas.sh start`.

**No RexROV2 in the Gazebo window.** The window was opened before the vehicle
spawned. `./run_eroas.sh start` always waits for the spawn; if you start
Gazebo by hand, launch with `gui:=false` and open `gz sim -g` after
`Entity creation successful` appears in the log.

**Windows don't open (`could not connect to display`).** Run
`xhost +SI:localuser:root` on the host, and check `echo $DISPLAY` is set in
the terminal you start from.

**Newer GPU (RTX 40xx or later) and the sonar fails.** Build for your GPU:
`EROAS_CUDA_ARCH=89 ./run_eroas.sh build` (89 = RTX 40xx; see NVIDIA's
compute capability table).

**Slow simulation / sonar below 4 Hz.** The DAVE sonar is GPU heavy. Close
other GPU programs; a 4 GB GTX 1050 Ti manages about 5 Hz.

## Using an existing container

On the original machine the container is `eroas_gpu` (built by hand, without
the source mount). The script works with it too:

```bash
EROAS_CONTAINER=eroas_gpu ./run_eroas.sh start
```

Code changes there are copied in with `docker cp` (to both `src/` and
`install/`), since that container does not mount this folder.
