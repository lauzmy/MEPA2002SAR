SHELL := /bin/bash

IMAGE := ros2-pi:jazzy-desktop

# Desktop is OFF by default. Enable with: make build DESKTOP=1
DESKTOP ?= 0

UID := $(shell id -u)
GID := $(shell id -g)

build:
	docker build -t $(IMAGE) \
	  --build-arg USER_UID=$(UID) \
	  --build-arg USER_GID=$(GID) \
	  --build-arg INSTALL_DESKTOP=$(DESKTOP) \
	  .

up:
	docker compose up -d

down:
	docker compose down

shell:
	docker compose exec ros2 bash

logs:
	docker compose logs -f --tail=200 ros2

restart:
	docker compose restart ros2

recreate:
	docker compose down
	docker compose up -d

# Run once per local desktop session if you want GUI apps from the container
x11:
	xhost +local:docker >/dev/null 2>&1 || true

ros_build:
	colcon build --cmake-args -DBUILD_TESTING=ON
	source install/setup.sh
