## Prerequisites

- Docker Engine + Compose plugin (`docker compose`)
- `make`


## Quickstart

```bash
git clone <REPO_URL> ~/ros2_ws
cd ~/ros2_ws

make build      # headless by default (DESKTOP=0)
make up
make shell
```

### Optional: build with desktop tools (rviz/rqt)
```bash
make build DESKTOP=1
make recreate
make x11        # Run once per local desktop session if you want GUI apps from the container
```
