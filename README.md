# Supabase Docker

This is a minimal Docker Compose setup for self-hosting Supabase. Follow the steps [here](https://supabase.com/docs/guides/hosting/docker) to get started.

This project uses a `.env.example` file to define required environment variables. To run the application, copy it to `.env` and fill in your own values.


Once ready to build the devcontainer image, run:
```bash
docker buildx build -f .devcontainer/Dockerfile.sdl -t sdl_dev .