FROM mcr.microsoft.com/devcontainers/cpp:1-debian-12

ARG USERNAME=vscode

# python3-venv is required for PlatformIO
# nodejs is required for SonarQube
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
    nodejs \
    python3-venv \
    && apt autopurge -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/udev/rules.d/ \
    && curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
    -o /etc/udev/rules.d/99-platformio-udev.rules \
    && chmod 644 /etc/udev/rules.d/99-platformio-udev.rules \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G plugdev $USERNAME \
    && groupadd --system --gid 986 rs232 \
    && usermod -a -G rs232 $USERNAME 
