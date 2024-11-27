"""
Use curl to download artifacts when the Ansible download fails.
Usage: Set these 2 variables and run the script:
        yaml_file = ""
        data_dir = ""
"""

import os
import yaml
import hashlib
import subprocess


def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def file_exists(file_path):
    return os.path.isfile(file_path)


def download_file(url, dest_path, checksum):
    if file_exists(dest_path):
        print(f"File {dest_path} already exists. Skipping download.")
        return

    os.makedirs(os.path.dirname(dest_path), exist_ok=True)

    print(f"Downloading {url} to {dest_path}...")
    curl_command = ["curl", "-L", url, "-o", dest_path]
    subprocess.run(curl_command, check=True)


def resolve_variables(data_dir):
    return os.path.expanduser(data_dir)


def download_files_from_yaml(yaml_file, data_dir):
    data_dir = resolve_variables(data_dir)
    content = load_yaml(yaml_file)

    for task in content:
        if task.get("ansible.builtin.get_url"):
            url = task["ansible.builtin.get_url"]["url"]
            dest = task["ansible.builtin.get_url"]["dest"]
            checksum = task["ansible.builtin.get_url"]["checksum"]

            dest_path = dest.replace("{{ data_dir }}", data_dir)
            download_file(url, dest_path, checksum)


if __name__ == "__main__":
    yaml_file = ""
    data_dir = ""

    download_files_from_yaml(yaml_file, data_dir)
