FROM mcr.microsoft.com/devcontainers/cpp:1-debian-12

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# python3-venv is required for PlatformIO
# nodejs is required for SonarQube
# clangd for language server
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
    clangd \
    clang-format \
    python3-venv \
    nodejs \
    && apt autopurge -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# platformio (cannot be added to PATH so we symlink)
# https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules
RUN curl -fsSL -o pio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 pio.py \
    && rm pio.py \
    && curl -fLo /etc/udev/rules.d/99-platformio-udev.rules --create-dirs \
    https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G plugdev $USERNAME \
    && ln -s /home/vscode/.platformio/penv/bin/platformio /usr/bin/platformio

