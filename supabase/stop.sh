#!/bin/bash

echo "Checking if Supabase is running..."

# Check if any Supabase container is running
if docker ps --format '{{.Names}}' | grep -q '^supabase-'; then
  echo "Stopping Supabase services..."
  docker-compose -f docker-compose.yml down
else
  echo "Supabase is already stopped."
fi
