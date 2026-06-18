 import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin

BASE_URL = "http://172.17.23.189:8080/sitl_final_package/"
OUTPUT_DIR = "sitl_final_package"

def download_recursive(url, outdir):
    os.makedirs(outdir, exist_ok=True)
    response = requests.get(url)
    if response.status_code != 200:
        print(f"Failed to access {url} — status code: {response.status_code}")
        return
    soup = BeautifulSoup(response.text, 'html.parser')
    for a in soup.find_all('a', href=True):
        href = a['href']
        if href in ('../', '/'):
            continue
        full_url = urljoin(url, href)
        local_path = os.path.join(outdir, href)
        if href.endswith('/'):
            # It's a subdirectory—recurse into it
            download_recursive(full_url, local_path)
        else:
            print(f"Downloading file: {full_url}")
            res = requests.get(full_url)
            with open(local_path, 'wb') as f:
                f.write(res.content)

if __name__ == "__main__":
    download_recursive(BASE_URL, OUTPUT_DIR)
    print("All files downloaded with directory structure preserved.")
