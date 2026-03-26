SHELL := /bin/bash

IMAGE := ros2-pi:jazzy-desktop
UID := $(shell id -u)
GID := $(shell id -g)

.PHONY: build up down shell logs restart recreate x11 ros_build ros_clean

build:
	docker build -t $(IMAGE) \
	  --build-arg USER_UID=$(UID) \
	  --build-arg USER_GID=$(GID) \
	  .

up:
	docker compose up -d

down:
	docker compose down

shell:
	docker compose exec -u ubuntu ros2 bash

logs:
	docker compose logs -f --tail=200 ros2

restart:
	docker compose restart ros2

recreate:
	docker compose down
	docker compose up -d

# Run once per local desktop session to allow GUI apps from the container.
x11:
	@if ! command -v xauth >/dev/null 2>&1; then \
	  echo "xauth is required for GUI forwarding (install package: xauth)"; \
	  exit 1; \
	fi
	@touch $$HOME/.docker.xauth
	@xauth nlist $${DISPLAY:-:0} | sed -e 's/^..../ffff/' | xauth -f $$HOME/.docker.xauth nmerge - >/dev/null 2>&1 || true
	@chmod 600 $$HOME/.docker.xauth
	@xhost +SI:localuser:$$(id -un) >/dev/null 2>&1 || true
	@xhost +SI:localuser:root >/dev/null 2>&1 || true
	@echo "X11 access configured for Docker GUI apps."

ros_build:
	colcon build --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON

ros_clean:
	rm -rf build install log
