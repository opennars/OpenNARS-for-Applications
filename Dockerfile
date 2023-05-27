# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Update and upgrade the system packages
RUN apt-get update && apt-get upgrade -y

# Install build-essential, etc...
RUN apt install -y build-essential vim python3 python3-pip

# Copy the entire current directory content into /app inside the docker image
WORKDIR /app
COPY . /app

# Ensure that build.sh is executable and then execute it
RUN chmod +x /app/build.sh
RUN /app/build.sh

# Set a command to keep the container running indefinitely
CMD ["tail", "-f", "/dev/null"]

