import scipy.spatial.distance
import numpy as np
import cv2
import math


def order_coordinates(c_list):
    assert len(c_list)==4
    sortlist = []
    c = np.array(c_list)
    #min lentgh vector is point 0
    sortlist.append(c_list[np.argmin(c[:,0]*c[:,1])])
    
    
    #image center
    u0 = (cols)/2.0
    #calculate the angle
    nfac = 1/np.linalg.norm(c,axis=-1)
    norm_vec = np.dot(np.diag(nfac),c)
    angle = np.arccos(np.dot(norm_vec,[1,0]))*180/np.pi
    #min angle as point 1
    sortlist.append(c_list[np.argmin(angle)])
    #max angle is point 2
    sortlist.append(c_list[np.argmax(angle)])
    #last is point 3
    sortlist.append(c_list[np.argmax(c[:,0]*c[:,1])])
    return sortlist

def wrap_perspective(orig,coordinatelist,factor=1):
    #https://stackoverflow.com/questions/38285229/calculating-aspect-ratio-of-perspective-transform-destination-image
    (rows,cols,_) = orig.shape

    #image center
    u0 = (cols)/2.0
    v0 = (rows)/2.0



    #detected corners on the original image (sorted)
    sortedlist = order_coordinates(coordinatelist)
    p = [(int(x/factor),int(y/factor)) for (x,y) in sortedlist]

    #widths and heights of the projected image
    w1 = scipy.spatial.distance.euclidean(p[0],p[1])
    w2 = scipy.spatial.distance.euclidean(p[2],p[3])

    h1 = scipy.spatial.distance.euclidean(p[0],p[2])
    h2 = scipy.spatial.distance.euclidean(p[1],p[3])

    w = max(w1,w2)
    h = max(h1,h2)

    #visible aspect ratio
    ar_vis = float(w)/float(h)

    #make numpy arrays and append 1 for linear algebra
    m1 = np.array((p[0][0],p[0][1],1)).astype('float32')
    m2 = np.array((p[1][0],p[1][1],1)).astype('float32')
    m3 = np.array((p[2][0],p[2][1],1)).astype('float32')
    m4 = np.array((p[3][0],p[3][1],1)).astype('float32')

    #calculate the focal disrance
    k2 = np.dot(np.cross(m1,m4),m3) / np.dot(np.cross(m2,m4),m3)
    k3 = np.dot(np.cross(m1,m4),m2) / np.dot(np.cross(m3,m4),m2)

    n2 = k2 * m2 - m1
    n3 = k3 * m3 - m1

    n21 = n2[0]
    n22 = n2[1]
    n23 = n2[2]

    n31 = n3[0]
    n32 = n3[1]
    n33 = n3[2]

    f = math.sqrt(np.abs( (1.0/(n23*n33)) * ((n21*n31 - (n21*n33 + n23*n31)*u0 + n23*n33*u0*u0) + (n22*n32 - (n22*n33+n23*n32)*v0 + n23*n33*v0*v0))))

    A = np.array([[f,0,u0],[0,f,v0],[0,0,1]]).astype('float32')

    At = np.transpose(A)
    Ati = np.linalg.inv(At)
    Ai = np.linalg.inv(A)

    #calculate the real aspect ratio
    ar_real = math.sqrt(np.dot(np.dot(np.dot(n2,Ati),Ai),n2)/np.dot(np.dot(np.dot(n3,Ati),Ai),n3))

    if ar_real < ar_vis:
        W = int(w)
        H = int(W / ar_real)
    else:
        H = int(h)
        W = int(ar_real * H)

    pts1 = np.array(p).astype('float32')
    pts2 = np.float32([[0,0],[W,0],[0,H],[W,H]])

    #project the image with the new w/h
    M = cv2.getPerspectiveTransform(pts1,pts2)

    dst = cv2.warpPerspective(orig,M,(W,H))
    return dst