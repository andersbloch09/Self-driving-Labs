#!/bin/bash

# Function to determine which docker compose command to use
get_docker_compose_cmd() {
  if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    echo "docker compose"
  elif command -v docker-compose >/dev/null 2>&1; then
    echo "docker-compose"
  else
    echo "Error: Neither 'docker compose' nor 'docker-compose' is available" >&2
    exit 1
  fi
}

DOCKER_COMPOSE_CMD=$(get_docker_compose_cmd)

echo "Checking if Supabase is already running..."

# Check if the Supabase DB container is running
if docker ps --format '{{.Names}}' | grep -q '^supabase-db$'; then
  echo "✅ Supabase is already running."
else
  echo "🚀 Starting Supabase services..."
  $DOCKER_COMPOSE_CMD -f docker-compose.yml up -d
fi
