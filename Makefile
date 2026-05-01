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
	docker compose exec -u ubuntu -e DISPLAY=$${DISPLAY:-:0} ros2 bash

logs:
	docker compose logs -f --tail=200 ros2

restart:
	docker compose restart ros2

recreate:
	docker compose down
	docker compose up -d

# Run once per local desktop session to allow GUI apps from the container.
# Do NOT run with sudo – must run as the desktop user who owns the X session.
# When using SSH X11 forwarding (ssh -X), $DISPLAY is set automatically in your session.
x11:
	@if [ "$$(id -u)" = "0" ]; then \
	  echo "ERROR: Do not run 'make x11' with sudo. Run as your normal desktop user."; \
	  exit 1; \
	fi
	@DISP=$${DISPLAY:-:0}; \
	 echo "Using DISPLAY=$$DISP, HOME=$$HOME"; \
	 rm -f $$HOME/.docker.xauth; \
	 touch $$HOME/.docker.xauth; \
	 chmod 777 $$HOME/.docker.xauth; \
	 XAUTH_SRC=$${XAUTHORITY:-$$HOME/.Xauthority}; \
	 echo "Using XAUTHORITY=$$XAUTH_SRC"; \
	 if [ -f "$$XAUTH_SRC" ]; then \
	   xauth -f "$$XAUTH_SRC" nlist 2>/dev/null | sed 's/^..../ffff/' | xauth -f $$HOME/.docker.xauth nmerge - 2>/dev/null; \
	   echo "xauth cookies written: $$(xauth -f $$HOME/.docker.xauth list | wc -l)"; \
	   xauth -f $$HOME/.docker.xauth list; \
	 else \
	   echo "WARNING: No XAUTHORITY file found at $$XAUTH_SRC"; \
	 fi
	@DISPLAY=$${DISPLAY:-:0} xhost +localhost 2>/dev/null && echo "xhost: localhost TCP connections allowed" || echo "xhost: failed – ensure DISPLAY is set"
	@DISPLAY=$${DISPLAY:-:0} xhost +local: 2>/dev/null && echo "xhost: local socket connections allowed" || true
	@echo "X11 access configured for Docker GUI apps."

ros_build:
	colcon build --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON

ros_clean:
	rm -rf build install log
