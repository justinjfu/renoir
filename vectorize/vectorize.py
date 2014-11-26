#!/usr/bin/python
import numpy
import argparse

def __parse_args():
    arg_parser = argparse.ArgumentParser(description="Image vectorizer")
    arg_parser.add_argument("img_file", help="image file")
    arg_parser.add_argument("--verbose", "-v", dest='verbose', action='store_true', help="Verbose mode")

    args = arg_parser.parse_args()
    return args

def main():
    args = __parse_args()
    print args.verbose

if __name__ == "__main__":
    main()
