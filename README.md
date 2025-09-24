# Supabase Docker

This is a minimal Docker Compose setup for self-hosting Supabase. Follow the steps [here](https://supabase.com/docs/guides/hosting/docker) to get started.

This project uses a `.env` file to configure Supabase and related services, change the supabase/.env.example to supabase/.env.

Once ready to build the devcontainer image, run:
```bash
docker buildx build -f .devcontainer/Dockerfile.sdl -t sdl_dev .