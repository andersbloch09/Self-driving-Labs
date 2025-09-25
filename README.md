# Self-Driving Labs

This repository contains a project for self-driving labs, focusing on automated laboratory systems and intelligent experimentation workflows.

This project uses a `.env.example` file to define required environment variables. To run the application, copy it to `.env` and fill in your own values.


Once ready to build the devcontainer image, run:
```bash
docker buildx build -f .devcontainer/Dockerfile.sdl -t sdl_dev .