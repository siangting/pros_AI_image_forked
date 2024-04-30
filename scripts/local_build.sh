#!/bin/bash

export set=false
export IMG_NAME="pros_ai_image"
export ECR_URL="ghcr.io/otischung"
export TODAY=$(date +%Y%m%d)
export DOCKER_CLI_EXPERIMENTAL=enabled

# Function to display a usage message
usage() {
    echo "Usage: $0 -j <threads>"
    exit 0
}

# Parse command-line options
while [ "$#" -gt 0 ]; do
    case "$1" in
        -j)
            threads="$2"
            set=true
            shift 2
            ;;
        -h)
            usage
            shift
            exit 0
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            usage
            ;;
    esac
done

# The regular expression matches digits only
# Ref: https://www.cyberciti.biz/faq/bash-see-check-input-number-integer-in-linux-script/
if ! [[ "$threads" =~ ^[0-9]+$ ]] && ! [[ "$threads" =~ ^[-][0-9]+$ ]]; then
    echo "$0 - $threads is NOT an integer. Ignoring."
    set=false
fi

if [ "$set" = true ]; then
    docker build --platform linux/amd64 \
        --tag $ECR_URL/$IMG_NAME:latest \
        --tag $ECR_URL/$IMG_NAME:$TODAY \
        --build-arg THREADS=$threads \
        -f ./Dockerfile .
else
    docker build --platform linux/amd64 \
        --tag $ECR_URL/$IMG_NAME:latest \
        --tag $ECR_URL/$IMG_NAME:$TODAY \
        -f ./Dockerfile .
fi
