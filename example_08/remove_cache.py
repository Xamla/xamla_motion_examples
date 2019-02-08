"""
Deletes the cache
"""
import os

def remove_cache():
    name = "example_08/trajectory_cache.pickle"

    try:
        os.remove(name)
    except FileNotFoundError as e:
        print(e)

if __name__ == '__main__':
    remove_cache()