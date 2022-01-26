#!/usr/bin/env python3
# Jack Fetkovich
# Download CTRE libraries by parsing Maven .json file
from sys import argv
import requests


def import_json(url):
  '''
  Imports JSON file from specified URL
  '''
  r = requests.get(url, allow_redirects=True)
  return r.json()

def construct_download_urls(json_file:dict, archs):
  '''
  Constructs download URLs from downloaded JSON 
  @param json_file a JSON file in the form of a dictionary object
  @return: list of download links
  '''
  dependencies = json_file['cppDependencies']
  download_links = []
  for d in dependencies:
    base_url = json_file['mavenUrls'][0] + d['groupId'].replace('.', '/') + "/" + d['artifactId'] + "/" + d['version'] +'/' + d['artifactId'] + '-' + d['version'] + '-'
    download_links.append(base_url + d['headerClassifier'] + '.zip')

    # Some packages are available which are not included int
    # the .json list, explicitly list the archs which we want
    # to look for rather than believing the json info
    for a in archs:
      download_links.append(base_url + a + '.zip')
    
  return download_links
  
def download_files(download_urls):
  '''
  Downloads files from specified URLs
  @param download_urls a list of urls to download from
  @return: 
  '''
  successful = 0
  failed = 0
  failed_files = []
  for url in download_urls:
    r = requests.get(url)
    if r.status_code != 200:
      failed += 1
      failed_files.append(url)
      continue
    file_name = "." + url[url.rindex('/'):]
    
    with open (file_name, 'wb') as f:
      f.write(r.content)

    successful +=1
  
  if failed != 0:
    print(f"{failed} FILES FAILED TO DOWNLOAD:")
    for file in failed_files:
      print(f"\t{file}")
  
  print(f"{successful} files downloaded successfully")
  
def main():
  url = argv[1]
  json_file = import_json(url)
  archs = ['linuxathena', 'linuxx86-64', 'linuxaarch64bionic', 'linuxraspbian']
  download_links = construct_download_urls(json_file, archs)
  download_files(download_links)

main()
