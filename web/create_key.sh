#!/usr/bin/env bash

basedir=$(dirname "$0")

if [[ ! -d "$basedir/../private" ]]; then
  mkdir "$basedir/../private"
fi

file="$basedir/../private/server.pem"

if [[ -f "$file" ]]; then
  echo "$file already exists"
  exit 1
fi

openssl req -new -x509 -keyout "$file" -out "$file" -days 365 -nodes
