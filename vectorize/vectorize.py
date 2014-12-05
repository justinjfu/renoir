#!/usr/bin/python
import numpy as np
import scipy as sp
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

def read_image(filename, show=True, thresh = 120):
    import scipy.misc as spm
    #import pdb;pdb.set_trace()
    img = spm.imread(filename, flatten = True)
    #img = spm.lena()
    img = spm.imresize(img, (100,100))
    img = spm.imfilter(img, ftype='find_edges')
    img = (img > thresh)
    img = img+0

    #remove outer edge
    img[:,0] = 0
    img[:,99] = 0
    img[0,:] =0
    img[99,:] = 0

    if np.sum(img) > (100*100)/2:
        img = 1-img
    if show:
        plt.imshow(img, interpolation='nearest', cmap='Greys')
        plt.show()
    return img


def score_nearest(point):
    def score(candidate_points):
        return min(candidate_points, key=lambda pt: dist(pt, point))
    return score

def score_momentum(point, prev_point, K=1.0):
    if prev_point is None:
        return score_nearest(point)
    else:
        direc = point-prev_point
        prev_momentum_norm = direc/(np.sqrt(np.dot(direc,direc)))
        def key_momentum(pt):
            diff = pt-point
            norm = np.sqrt(np.dot(diff, diff))
            momentum_norm = diff/norm
            cosine = np.dot(prev_momentum_norm, momentum_norm)
            return -K*cosine + norm
        def score(candidate_points):
            return min(candidate_points, key=key_momentum)
        return score


def scan_neighbors(pic, point, score_func, scan_range = 1):
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

    #score
    best_point = score_func(candidate_points) #min(candidate_points, key=lambda pt: dist(pt, point))
    return best_point

def select_initial(pic):
    """
    Select an arbitrary point in the picture
    """
    X, Y = np.where(pic==1)
    return np.array((X[0], Y[0]))

LIFT = 'lift!123'
def generate_segments(pic, params):
    """
    Returns an iterator that plans a path through the image
    Objects in an iterator can either be a list of 2 points [[x1,y1],[x2,y2]] or an instance of LIFT
    """
    scan_range = params.get('scan_range', 2)
    pic = np.copy(pic) #don't modify original
    while np.sum(pic) != 0:
        point = select_initial(pic)
        prev_point = None
        pic[point[0],point[1]] = 0
        while True:
            next_point = scan_neighbors(pic, point, score_momentum(point, prev_point, K=30.0), scan_range=scan_range)
            if next_point is not None:
                yield [point, next_point]
                prev_point = point
                point = next_point
                pic[point[0],point[1]] = 0
            else:
                yield LIFT
                break

def filter_lifts(segment_iter):
    #Filter duplicate lifts
    lift = False
    for segment in segment_iter:
        if lift and segment is LIFT:
            continue
        lift = segment is LIFT
        yield segment

def remove_single(segment_iter):
    last_segment = None
    num_segments = 0
    for segment in segment_iter:
        if segment is not LIFT:
            if last_segment:
                yield last_segment
            last_segment = segment 
            num_segments += 1
        else:
            if num_segments > 1:
                yield last_segment
            num_segments = 0
            last_segment = None
            yield LIFT
    if last_segment:
        yield last_segment

def test():
    img = np.zeros((5,5))
    for i in range(5):
        img[i,i] = 1
        img[4-i,i] = 1

    img = read_image('cartman.jpg', show=True)

    for segment in filter_lifts(remove_single(generate_segments(img, {'scan_range':2}))):
        print segment

def main():
    #args = __parse_args()
    #print args.verbose
    test()
    #read_image('hi')

if __name__ == "__main__":
    main()
