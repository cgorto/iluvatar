# Multi-stage Dockerfile for iluvatar-server and iluvatar-camera
#
# Builds both headless binaries. The simulator (Bevy/GPU) runs natively on the host.
#
# Usage:
#   docker compose build
#   docker compose up

# ---- Build stage ----
FROM rust:1.87-bookworm AS builder

WORKDIR /build

# Copy workspace manifest and lock file first for layer caching
COPY Cargo.toml Cargo.lock ./

# Copy all crate manifests (needed for cargo to resolve workspace)
COPY crates/iluvatar-core/Cargo.toml crates/iluvatar-core/Cargo.toml
COPY crates/iluvatar-camera/Cargo.toml crates/iluvatar-camera/Cargo.toml
COPY crates/iluvatar-server/Cargo.toml crates/iluvatar-server/Cargo.toml
COPY crates/iluvatar-simulator/Cargo.toml crates/iluvatar-simulator/Cargo.toml

# Create dummy source files so cargo can resolve dependencies
RUN mkdir -p crates/iluvatar-core/src && echo "" > crates/iluvatar-core/src/lib.rs && \
    mkdir -p crates/iluvatar-camera/src && echo "fn main() {}" > crates/iluvatar-camera/src/main.rs && \
    mkdir -p crates/iluvatar-server/src && echo "fn main() {}" > crates/iluvatar-server/src/main.rs && \
    mkdir -p crates/iluvatar-simulator/src && echo "fn main() {}" > crates/iluvatar-simulator/src/main.rs

# Pre-build dependencies (cached unless Cargo.toml/Cargo.lock change)
RUN cargo build --release -p iluvatar-server -p iluvatar-camera 2>/dev/null || true

# Copy actual source code
COPY crates/ crates/

# Touch source files to invalidate the dummy builds
RUN touch crates/iluvatar-core/src/lib.rs && \
    touch crates/iluvatar-camera/src/main.rs && \
    touch crates/iluvatar-server/src/main.rs

# Build the real binaries
RUN cargo build --release -p iluvatar-server -p iluvatar-camera

# ---- Runtime stage ----
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/target/release/iluvatar-server /app/iluvatar-server
COPY --from=builder /build/target/release/iluvatar-camera /app/iluvatar-camera

WORKDIR /app
