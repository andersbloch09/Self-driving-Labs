#!/bin/bash

echo "Checking if Supabase is already running..."

# Check if the Supabase DB container is running
if docker ps --format '{{.Names}}' | grep -q '^supabase-db$'; then
  echo "✅ Supabase is already running."
else
  echo "🚀 Starting Supabase services..."
  docker-compose -f docker-compose.yml up -d
fi
