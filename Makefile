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
	 touch $$HOME/.docker.xauth; \
	 chmod 600 $$HOME/.docker.xauth; \
	 XAUTH_SRC=$${XAUTHORITY:-$$HOME/.Xauthority}; \
	 if xauth nlist "$$DISP" 2>/dev/null | grep -q .; then \
	   xauth nlist "$$DISP" | sed 's/^..../ffff/' | xauth -f $$HOME/.docker.xauth nmerge -; \
	   echo "xauth cookies written: $$(xauth -f $$HOME/.docker.xauth list | wc -l)"; \
	 elif xauth -f "$$XAUTH_SRC" nlist "$$DISP" 2>/dev/null | grep -q .; then \
	   xauth -f "$$XAUTH_SRC" nlist "$$DISP" | sed 's/^..../ffff/' | xauth -f $$HOME/.docker.xauth nmerge -; \
	   echo "xauth cookies written (from $$XAUTH_SRC): $$(xauth -f $$HOME/.docker.xauth list | wc -l)"; \
	 else \
	   echo "WARNING: No xauth cookies found for $$DISP – falling back to xhost +local:"; \
	 fi
	@DISPLAY=$${DISPLAY:-:0} xhost +local: 2>/dev/null && echo "xhost: local connections allowed" || echo "xhost: failed – ensure DISPLAY is set"
	@echo "X11 access configured for Docker GUI apps."

ros_build:
	colcon build --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON

ros_clean:
	rm -rf build install log
