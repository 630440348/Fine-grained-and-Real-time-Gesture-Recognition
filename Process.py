import numpy as np
import math

Rad = 180/3.14
fs = 100
Kp = 2.0 * fs
Ki = 0.2 * fs
halfT = 0.05 * fs

def ProcessFormula1(MeasureData, k, b):
    return MeasureData*k+b

def ProcessFormula5(MeasureData, TrueData):

    sumMD = 0
    sumTD = 0
    sumMDTD = 0
    sumMDpow = 0
    sumTDpow = 0
    sumMDTDpow = 0
    n = len(MeasureData)

    for i in MeasureData:
        sumMD += i
        sumMDpow += math.pow(i, 2)

    for i in TrueData:
        sumTD += i
        sumTDpow += math.pow(i, 2)

    for (i, j) in zip(MeasureData, TrueData):
        sumMDTD += (i*j)
        sumMDTDpow += math.pow((i*j), 2)

    k = n*sumMDTD-sumTD*sumMD
    k = k/(n*sumMDTDpow-math.pow(sumTD, 2))

    b = sumMDpow*sumTD - sumMD*sumMDTD
    b = b/(n*sumMDpow-math.pow(sumMD, 2))

    return k, b

def ProcessFormula7(g_x, g_y, g_z):
    c_x=0
    c_y=0
    c_z=0
    for i in g_x:
        c_x+=i

    for i in g_y:
        c_y+=i

    for i in g_z:
        c_x+=i

    return c_x/len(g_x), c_y/len(g_y), c_z/len(g_z)




def ProcessFormula14(data, length):

    for i in range(2, length-2, 5):
        data[i] = 1/70(69*data[i]+4*data[i+1]-6*data[i+2]+46*data[i+3]-data[i+4])
        data[i+1] = 1/35(2*data[i]+27*data[i-1]+12*data[i+2]-8*data[i+3]+2*data[i+4])
        data[i+2] = 1/35(-3*data[i]+12*data[i+1]+17*data[i+2]+12*data[i+3]-3*data[i+4])
        data[i+3] = 1/35(2*data[i]-8*data[i+1]+12*data[i+2]+27*data[i+3]+2*data[i+4])
        data[i+4] = 1/70(-data[i]+4*data[i+1]-4*data[i+2]+4*data[i+3]+69*data[4])


    return data




def ProcessFormula15(data, length, deltat):

    speed_v = [0]
    move_s = 0
    for i in range(1, length-1, 1):
        speed_v.append(speed_v[i-1] +((data[i-1]+4*data[i]+data[i+1])/6)*deltat)

    for i in range(1, length - 1, 1):
        move_s = move_s + ((speed_v[i-1]+4*speed_v[i]+speed_v[i+1])/6)*deltat

    return move_s



def ProcessFormula16(W_x, W_y, W_z, s_x, s_y, s_z):
    return W_x+s_x, W_y+s_y, W_z+s_z


def ProcessFormula17(d_realp, d_realn, dmea_p, dmea_n):

    u_xp = (dmea_p-d_realp)/d_realp
    u_xn = (dmea_n-d_realn)/d_realn
    return u_xp, u_xn

def ProcessFormula18(s_nxp, s_nxn, u_xp, u_xn):
    s_uxp = s_nxp/(1+u_xp)
    s_uxn = s_nxn/(1+u_xn)
    return  s_uxp, s_uxn

def ProcessFormula19(movex, s_uxp, s_uxn, s_nx):

    if(s_nx>0):
        return movex+s_uxp
    else:
        return movex+s_uxn



def ProcessFormula21(move_x, move_y, move_z, move_mx, move_my, move_mz):

    a = math.pow(move_x-move_mx, 2)
    b = math.pow(move_y-move_my, 2)
    c = math.pow(move_z-move_mz, 2)

    return math.sqrt(a+b+c)

def ProcessFormula22(move, handlength):

    if(move>handlength):
        return 1
    else:
        return 0

def ProcessFormula23(a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z, r1, r2):

    d_bc = math.sqrt((math.pow(b_x-c_x,2))+(math.pow(b_y-c_y,2))+(math.pow(b_z-c_z,2)))
    d_ab = math.sqrt((math.pow(a_x-b_x,2))+(math.pow(a_y-b_y,2))+(math.pow(c_z-b_z,2)))

    b_fx = b_x+(r1*(b_x-c_x))/d_bc
    b_fy = b_y+(r1*(b_y-c_y))/d_bc
    b_fz = b_z+(r1*(b_z-c_z))/d_bc

    a_fx = a_x + (r2 * (a_x - b_x)) / d_ab
    a_fy = a_y + (r2 * (a_y - b_y)) / d_ab
    a_fz = a_z + (r2 * (a_z - b_z)) / d_ab

    return b_fx, b_fy, b_fz, a_fx, a_fy, a_fz


def ProcessGyroscope(g_x, g_y, g_z, a_x, a_y, a_z,q0,q1,q2,q3, exInt, eyInt, ezInt):



    norm = math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z)
    ax = a_x / norm
    ay = a_y / norm
    az = a_z / norm


    vx = 2.0 * (q1 * q3 - q0 * q2)
    vy = 2.0 * (q0 * q1 + q2 * q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

    ex = (ay * vz - az * vy)
    ey = (az * vx - ax * vz)
    ez = (ax * vy - ay * vx)

    exInt = exInt + ex * Ki
    eyInt = eyInt + ey * Ki
    ezInt = ezInt + ez * Ki


    gx = g_x + Kp * ex + exInt
    gy = g_y + Kp * ey + eyInt
    gz = g_z + Kp * ez + ezInt

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT

    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 = q0 / norm
    q1 = q1 / norm
    q2 = q2 / norm
    q3 = q3 / norm

    return q0,q1,q2,q3,exInt,eyInt,ezInt

def GetAngle_roll_pitch_yaw(q0, q1, q2, q3):

    Angle_roll = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * Rad
    Angle_pitch = math.asin(-2 * q1 * q3 + 2 * q0 * q2) * Rad
    Angle_yaw = math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * Rad
    return Angle_roll, Angle_pitch, Angle_yaw
