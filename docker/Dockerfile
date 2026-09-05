# syntax=docker/dockerfile:1
# =============================================================================
# AiSprayer Multi-Stage Production Dockerfile (Platform-Aware, Zero-PyTorch)
# Targets: RK3588 (linux/arm64) and Generic Linux (linux/amd64)
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# Stage 1: Build Environment (Python Virtualenv Builder)
# ─────────────────────────────────────────────────────────────────────────────
FROM ubuntu:24.04 AS builder

ARG TARGETARCH
ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive

# Install minimal toolchain for python venv setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build Python Virtual Environment (Zero-PyTorch, pure pre-built wheels)
WORKDIR /build
COPY docker/requirements-slim.txt ./docker/
COPY third_party/*.whl ./third_party/

RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip -i https://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com && \
    /opt/venv/bin/pip install --no-cache-dir -r docker/requirements-slim.txt -i https://mirrors.aliyun.com/pypi/simple/ --extra-index-url https://pypi.org/simple --trusted-host mirrors.aliyun.com

# Install matching local hardware SDK wheel (pyorbbecsdk2)
RUN find third_party/ -name "*.whl" -name "*linux_$(uname -m)*" -exec /opt/venv/bin/pip install --no-deps {} \;


# ─────────────────────────────────────────────────────────────────────────────
# Stage 2: Lean Production Runtime Image (Zero C++ source, Zero PyTorch)
# ─────────────────────────────────────────────────────────────────────────────
FROM ubuntu:24.04 AS runtime

LABEL maintainer="PixelBotLab <https://github.com/PixelBotLab/AiSprayer>"
LABEL description="AiSprayer Edge AI Spraying Robot Service (RK3588 Accelerated, Zero-PyTorch)"

ENV DEBIAN_FRONTEND=noninteractive
ENV APP_ROOT=/app
ENV PATH="/opt/venv/bin:${APP_ROOT}/bin:$PATH"
ENV PYTHONPATH="${APP_ROOT}/app/src:${APP_ROOT}/tools:${PYTHONPATH:-}"
ENV LD_LIBRARY_PATH="${APP_ROOT}/lib:${APP_ROOT}/third_party/install/lib:/usr/local/lib:/usr/lib:${LD_LIBRARY_PATH:-}"
ENV PYTHONUNBUFFERED=1

# Install only essential runtime libraries (fast, lightweight, avoiding heavy -dev tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    libssl3 \
    libsqlite3-0 \
    libyaml-cpp0.8 \
    libtinyxml2-10 \
    libopencv-core406t64 \
    libopencv-imgproc406t64 \
    libopencv-imgcodecs406t64 \
    libopencv-calib3d406t64 \
    libopencv-features2d406t64 \
    libopencv-flann406t64 \
    libatomic1 \
    libgomp1 \
    libgl1 \
    libomp5 \
    libopenblas0 \
    libdrm2 \
    libusb-1.0-0 \
    udev \
    ca-certificates \
    curl \
    procps \
    net-tools \
    iproute2 \
    lsof \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${APP_ROOT}

# Copy Python virtual environment from builder
COPY --from=builder /opt/venv /opt/venv

# Copy pre-packaged install distribution bundle (Zero C++ source code)
COPY install/ ${APP_ROOT}/
COPY docker/entrypoint.sh /entrypoint.sh

# Configure hardware acceleration runtime libraries and executable permissions
RUN if [ -f "${APP_ROOT}/lib/librknnrt.so" ]; then \
        cp "${APP_ROOT}/lib/librknnrt.so" /usr/lib/ 2>/dev/null || true; \
    fi && ldconfig && \
    chmod +x /entrypoint.sh \
        ${APP_ROOT}/bin/* \
        ${APP_ROOT}/env.sh \
        ${APP_ROOT}/run.sh 2>/dev/null || true

# Pre-create runtime directories
RUN mkdir -p ${APP_ROOT}/data/calib \
             ${APP_ROOT}/data/template_group \
             ${APP_ROOT}/app/logs \
             ${APP_ROOT}/logs

# Expose standard service ports:
# 8000:  FastAPI Core REST API & WebSocket
# 5173:  Web UI
# 18080: C++ Camera REST API
# 8554:  ZLMediaKit RTSP Server
# 8008:  ZLMediaKit HTTP-FLV / WebRTC
# 1935:  ZLMediaKit RTMP Server
EXPOSE 8000 5173 18080 8554 8008 1935

ENTRYPOINT ["/entrypoint.sh"]
CMD ["all"]
