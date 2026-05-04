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
	sudo chmod a+rw /dev/i2c-1
	docker compose exec -u ubuntu -e DISPLAY=$${DISPLAY:-:0} ros2 bash

logs:
	docker compose logs -f --tail=200 ros2

restart:
	docker compose restart ros2

recreate:
	docker compose down
	docker compose up -d

# Run once per SSH session on the Pi to allow GUI apps from the container.
# With SSH X11 forwarding (ssh -X), run this on the Pi – xauth cookies are
# copied from the SSH temp XAUTHORITY file into ~/.docker.xauth for the container.
# NOTE: xhost must be run on your LOCAL LAPTOP (where the X server runs), not here.
x11:
	@if [ "$$(id -u)" = "0" ]; then \
	  echo "ERROR: Do not run 'make x11' with sudo. Run as your normal desktop user."; \
	  exit 1; \
	fi
	@DISP=$${DISPLAY:-:0}; \
	 echo "Using DISPLAY=$$DISP, HOME=$$HOME, XAUTHORITY=$${XAUTHORITY:-~/.Xauthority}"; \
	 rm -rf $$HOME/.docker.xauth; \
	 touch $$HOME/.docker.xauth; \
	 chmod 777 $$HOME/.docker.xauth; \
	 XAUTH_SRC=$${XAUTHORITY:-$$HOME/.Xauthority}; \
	 echo "Copying all cookies from $$XAUTH_SRC (with FamilyWild):"; \
	 xauth -f "$$XAUTH_SRC" nlist | sed 's/^..../ffff/' | xauth -f $$HOME/.docker.xauth nmerge -; \
	 echo "xauth cookies written: $$(xauth -f $$HOME/.docker.xauth list | wc -l)"; \
	 xauth -f $$HOME/.docker.xauth list
	@echo "X11 access configured for Docker GUI apps."

ros_build:
	colcon build --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON

ros_clean:
	rm -rf build install log
