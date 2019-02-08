"""
Deletes the cache
"""
import os

name = "example_08/trajectory_cache.pickle"

try:
    os.remove(name)
except FileNotFoundError as e:
    print(e)