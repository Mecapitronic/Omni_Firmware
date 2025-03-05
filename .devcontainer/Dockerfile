FROM mcr.microsoft.com/devcontainers/cpp:1-debian-12

ARG USERNAME=vscode

# python3-venv is required for PlatformIO
# nodejs is required for SonarQube
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
    python3-venv \
    nodejs \
    && apt autopurge -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G plugdev $USERNAME

