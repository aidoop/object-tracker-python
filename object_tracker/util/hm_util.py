import numpy as np
import math


class HMUtil:
    def convertXYZABCtoHM(func):
        def wrapper(*args, **kwargs):
            funcout = func(*args, **kwargs)
            [x, y, z, a, b, c] = funcout

            ca = math.cos(a)
            sa = math.sin(a)
            cb = math.cos(b)
            sb = math.sin(b)
            cc = math.cos(c)
            sc = math.sin(c)
            H = np.array([[cb*cc, cc*sa*sb - ca*sc, sa*sc + ca*cc*sb, x], [cb*sc, ca *
                                                                           cc + sa*sb*sc, ca*sb*sc - cc*sa, y], [-sb, cb*sa, ca*cb, z], [0, 0, 0, 1]])
            return H
        return wrapper

    def convertHMtoXYZABC(func):
        def wrapper(*args, **kwargs):
            H = args[0]
            x = H[0, 3]
            y = H[1, 3]
            z = H[2, 3]
            if (H[2, 0] > (1.0 - 1e-10)):
                b = -math.pi/2
                a = 0
                c = math.atan2(-H[1, 2], H[1, 1])
            elif H[2, 0] < -1.0 + 1e-10:
                b = math.pi/2
                a = 0
                c = math.atan2(H[1, 2], H[1, 1])
            else:
                b = math.atan2(-H[2, 0], math.sqrt(H[0, 0]
                                                   * H[0, 0]+H[1, 0]*H[1, 0]))
                c = math.atan2(H[1, 0], H[0, 0])
                a = math.atan2(H[2, 1], H[2, 2])
            funcout = func([x, y, z, a, b, c])
            return funcout
        return wrapper

    @staticmethod
    @convertXYZABCtoHM
    def convertXYZABCtoHMDeg(xyzabc):
        [x, y, z, a, b, c] = xyzabc
        a = a*math.pi/180
        b = b*math.pi/180
        c = c*math.pi/180
        return [x, y, z, a, b, c]

    @staticmethod
    @convertXYZABCtoHM
    def convertXYZABCtoHMRad(xyzabc):
        return xyzabc

    @staticmethod
    @convertHMtoXYZABC
    def convertHMtoXYZABCDeg(xyzabc):
        [x, y, z, a, b, c] = xyzabc
        return [x, y, z, a*180/math.pi, b*180/math.pi, c*180/math.pi]

    '''
    HM =        R(3x3)     d(3x1)
                0(1x3)     1(1x1)

    HMInv =     R.T(3x3)   -R.T(3x3)*d(3x1)
                0(1x3)     1(1x1)

                (R^-1 = R.T)
    '''
    @staticmethod
    def inverseHM(H):
        rot = H[0:3, 0:3]
        trs = H[0:3, 3]

        HMInv = np.zeros([4, 4], dtype=np.float64)
        HMInv[0:3, 0:3] = rot.T
        HMInv[0:3, 3] = (-1.0)*np.dot(rot.T, trs)
        HMInv[3, 0:4] = [0.0, 0.0, 0.0, 1.0]
        return HMInv

    @staticmethod
    def invH(H):
        Hout = H.T
        Hout[3, 0:3] = np.zeros([[0, 0, 0]])
        Hout[0:3, 3] = (Hout[0:3, 0:3]*H[0:3, 3])*(-1)
        return Hout

    @staticmethod
    def makeHM(rot, trans):
        HM = np.zeros([4, 4], dtype=np.float64)
        HM[0:3, 0:3] = rot
        HM[0:3, 3] = trans
        HM[3, 0:4] = [0.0, 0.0, 0.0, 1.0]
        return HM


###############################################################################
# Test Codes
###############################################################################
if __name__ == '__main__':

    # prepare test data - xyzabc and homogeneous matrix
    xyzabc = [-0.09064515960284498, -0.6685702677827611, 0.07567205103501873, -
              176.08612962588248, -0.6780892276157752, 130.42940636697082]

    hm = np.array(([[-0.6484945560472715,  0.7588883777053825,     -0.05952512881754177,   -0.09061736124264554],
                    [0.7611366063389998,    0.6475911239041057,     -
                        0.03601114732090388,   -0.6687733681469619],
                    [0.01121950390181825,   -0.06865978753469798,   -
                        0.9975770427931304,    0.07609749637689898],
                    [0,                     0,                      0,                      1]]), dtype=np.float64
                  )

    # 0. print original xyzabc and hm
    print('Test Data')
    print("XYZABC = ")
    print(xyzabc)
    print("HM = ")
    print(hm)
    print()

    # 1.  convert XYZABC(degree) to a homogeneous matrix
    print('XYZABC --> HM')
    hmcal = HMUtil.convertXYZABCtoHMDeg(xyzabc)
    print('Converted HM')
    print(hmcal)
    print()

    # 2. convert a homogeneous matrix to XYZABC(degree)
    print('Converted XYZABC')
    xyzabc_calc = HMUtil.convertHMtoXYZABCDeg(hmcal)
    print(xyzabc_calc)
    print()

    # 3. test inverseHM
    hmpinv = np.linalg.pinv(hm)
    print('Pseudo Inverse')
    print(hmpinv)
    print('Check if inverse is available')
    print(np.dot(hmpinv, np.array(
        [-0.09064515960284498, -0.6685702677827611, 0.07567205103501873, 1], dtype=np.float64).T))
    print()

    hminv = HMUtil.inverseHM(hm)
    print('Homogeneous Inverse')
    print(hminv)
    print('Check if inverse is available')
    print(np.dot(hminv, np.array(
        [-0.09064515960284498, -0.6685702677827611, 0.07567205103501873, 1], dtype=np.float64).T))
