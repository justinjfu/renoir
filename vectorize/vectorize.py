#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import argparse

def __parse_args():
    arg_parser = argparse.ArgumentParser(description="Image vectorizer")
    arg_parser.add_argument("img_file", help="image file")
    arg_parser.add_argument("--verbose", "-v", dest='verbose', action='store_true', help="Verbose mode")
    args = arg_parser.parse_args()
    return args

def dist(p1, p2):
    """
    Returns euclidean distance between 2 points
    """
    diff = p1-p2
    return np.sqrt(np.dot(diff,diff))

def scan_neighbors(pic, point, scan_range = 1):
    """
    Returns the nearest point in the picture, or None if nothing is found
    """
    X, Y = pic.shape
    candidate_points = []
    for i in range(-scan_range, scan_range+1):
        i_coord = point[0]+i
        if i_coord < 0 or i_coord >= X:
            continue
        for j in range(-scan_range, scan_range+1):
            j_coord = point[1]+j
            if j_coord<0 or j_coord>=Y:
                continue
            if i==0 and j==0:
                continue
            if pic[i_coord,j_coord] == 1:
                candidate_points.append(np.array((i_coord,j_coord)))

    if not candidate_points:
        return None

    best_point = min(candidate_points, key=lambda pt: dist(pt, point))
    return best_point

def select_initial(pic):
    """
    Select an arbitrary point in the picture
    """
    X, Y = np.where(pic==1)
    return np.array((X[0], Y[0]))

LIFT = 'lift!123'
def generate_segments(pic):
    """
    Returns an iterator that plans a path through the image
    Objects in an iterator can either be a list of 2 points [[x1,y1],[x2,y2]] or an instance of LIFT
    """
    pic = np.copy(pic) #don't modify original
    while np.sum(pic) != 0:
        point = select_initial(pic)
        pic[point[0],point[1]] = 0
        while True:
            next_point = scan_neighbors(pic, point)
            if next_point is not None:
                yield [point, next_point]
                point = next_point
                pic[point[0],point[1]] = 0
            else:
                yield LIFT
                break

def test():
    img = np.zeros((5,5))
    for i in range(5):
        img[i,i] = 1
        img[4-i,i] = 1

    #plt.imshow(img, interpolation='nearest', cmap='gray')
    #plt.show()

    for segment in generate_segments(img):
        print segment

def main():
    #args = __parse_args()
    #print args.verbose
    test()

if __name__ == "__main__":
    main()
