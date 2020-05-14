import numpy
from numpy.core.umath_tests import inner1d
#import cython

def _isCCWTurn(o, a, b):
    return ((a.item(0) - o.item(0))*(b.item(1) - o.item(1)) - (a.item(1) - o.item(1))*(b.item(0) - o.item(0)))>0

def convexHull( points):
    """Returns points on 2D convex hull of an array of points in CCW order. The function uses Andrew's monotone chain convex hull algorithm. 
    Only the first two entrys of every point list are considered."""
    points=numpy.array(points)
    points=points[points[:,0].argsort(),] #points are being sorted by the first value in the point arrays.
    lower=numpy.ones(numpy.shape(points))*numpy.nan #lower hull
    upper=lower.copy() #upper
    lenLower=lenUpper=0
    # Build lower hull 
    for p in points:
        for lenLower in range(lenLower,0,-1):
            if _isCCWTurn(lower[lenLower-2,:], lower[lenLower-1,:], p):
                break
        lower[lenLower,:]=p
        lenLower+=1
    # Build upper hull
    for p in points[::-1]:
        for lenUpper in range(lenUpper,0,-1):
            if _isCCWTurn(upper[lenUpper-2,:], upper[lenUpper-1,:], p):
                break
        upper[lenUpper,:]=p
        lenUpper+=1
    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of the first list is omitted because it is repeated at the beginning of the other list. 
    return numpy.vstack([lower[:lenLower-1,:], upper[:lenUpper,:]])

def isPointInsideConvexHull(cHPs, testPoint):
    """The function tests whether a given point is inside a convex hull. The hull's points must be given in CCW order to get the correct results! If the points are given in CW order, also points that lie on the boundary are classified as 'inside'. """
    if numpy.shape(cHPs)[0]<=3: # if the lists consists only of the starting point, another point and once more the starting point, you get a line as convex hull. Therefore, the testPoint won't fit in. 
        return False
    relCHPs=numpy.subtract(cHPs,testPoint)
    a=relCHPs[1:,0]* relCHPs[:-1,1]-relCHPs[0:-1,0]* relCHPs[1:,1]
    return all(a<0)

def shortestVectorsToConvexHull(cHPs, testPoint):
    """The function computes the shortest vectors that connect the test point with the lines between the points specified in cHPs"""
    relCHPs=numpy.subtract(cHPs,testPoint)
    P1P2=numpy.diff(cHPs,axis=0)
    lambda1=(inner1d(relCHPs[:-1,:],P1P2))/numpy.sum(P1P2**2,axis=1)
    return relCHPs[:-1,:]-lambda1[:,numpy.newaxis]*P1P2


def zAxisToNegGVectorRot(gVec):
    """ This function returns a rotation matrix with which points may be transformed into a coordinate system whose z-vector is parallel to the gravitation vector."""
    gVec/=-numpy.linalg.norm(gVec)
    if gVec.item(0)==gVec.item(1)==0:
        return numpy.eye(3)
    cosRotAngle=gVec.item(2)
    sinRotAngle=(1-cosRotAngle**2)**0.5
    rotAxis=numpy.array([gVec[1],-gVec[0],0])
    rotAxis/=numpy.linalg.norm(rotAxis)
    return numpy.linalg.inv(numpy.eye(3)*cosRotAngle + numpy.cross(rotAxis,numpy.eye(3))*sinRotAngle + numpy.outer(rotAxis,rotAxis)*(1-cosRotAngle))

def gVectorFromGroundContacts(groundContacts):
    """ In lack of an acceleration sensor, this function is able to compute the g-vector dependent on the positions of the ground contacts. The result, however, is only valid for a plane ground (less plane -> less valid)! """
    A = numpy.column_stack(( groundContacts[:,0:2],numpy.ones(groundContacts.shape[0])))
    temp,*_=(numpy.linalg.lstsq(A,groundContacts[:,2]))
    temp[2]=-1
    temp/=-numpy.linalg.norm(temp)
    if temp.item(2)>0:
        temp*=-1
    return temp

def directionToMove(groundContacts,cOM,threshold):
    """ If the center of mass of the robot comes too close to the boundary of the convex hull that is spanned by the foot points, a vector is returned that points in the direction where a better stability would could be reached."""
    if numpy.shape(groundContacts)[0]<3:
        return None
    gVec=gVectorFromGroundContacts(groundContacts)
    R=zAxisToNegGVectorRot(gVec)
    cOMGrav=numpy.dot(R,cOM)
    groundContactsGrav=numpy.dot(R,groundContacts.T).T
    convexHullGrav=convexHull( groundContactsGrav )
    if not isPointInsideConvexHull(convexHullGrav,cOMGrav):
        return None
    shortestVecs=shortestVectorsToConvexHull(convexHullGrav, cOMGrav)
    dists=(numpy.sum(shortestVecs**2,axis=1))**0.5
    normEnergies=(dists+shortestVecs[:,2])
    tempInd,*_=numpy.nonzero(normEnergies<threshold)
    if numpy.shape(tempInd)[0]==0:
        return numpy.array([0,0,0])
    else:
        directionToMove=numpy.dot(-shortestVecs[tempInd,0:2].T,(threshold-normEnergies[tempInd]).T)
        return numpy.append(directionToMove,0)
    
def NESM(cHPs, cOM):
    """ This gives the minimum normalized energy that is needed to push the center of mass over the lines that built up the support polygon. 
    The formula for the required normalized energy h_i that is needed to push the center of mass over line i is given in Garcia2002: 
    h_i=|R_i|*(1-cos(theta))*cos(psi) 
    with R_i being the vector between the center of mass and the line,
    theta being the angle between R_i and the vertical axis
    and psi being the angle between the line and the horizontal plane."""
    hs=[]
    for point1, point2 in zip(cHPs[:-1,:], cHPs[1:,:]):
        dp = point2-point1
        ndpq=numpy.linalg.norm(dp)**2
        temp1=numpy.dot((cOM-point1), dp)/ndpq
        temp2=numpy.linalg.norm(point1-cOM+dp*temp1)
        h=numpy.sqrt(1-dp[2]**2/ndpq)*temp2*(1+(point1[2]-cOM[2]+dp[2]*temp1)/temp2)
        hs.append(h)
    return min(hs)

