#!/bin/bash

# AiSprayer App Dependencies Script (Forwarder to env.sh)
# Note: Dependency creation and environment management has been consolidated into env.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/env.sh" "$@"
