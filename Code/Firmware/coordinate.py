from math import pi
from math import sin
from math import cos

m = 2 #accuracy of likeless of readings
lim = 180 # maximum accurate LiDAR reading



# ==================================================================================
#                             ROUTE 1 : -180 < a < -90
# ==================================================================================
def route1(a,F,B,R,L,zone):

    b = (-a - 90)*pi/180   #conversion to radians for correct use in sin/cos
    c = (180 + a)*pi/180

    if (R > lim and L > lim) or (B > lim and F > lim):
        return "Coordinate cannot be calculated: R + L unknown or B + F unknown."

    if zone == 1:

        # X COORDINATE =============================
        if R > lim:   # if R is unreadable, use F
            x = F*cos(c)
        else:
            if abs(F*cos(b) - L*cos(c)) < m or L > lim: # if F and L both hitting bottom, don't use F
                x = R*cos(b)
            else:
                x = F*cos(c)

        # Y COORDINATE ====================
        if L > lim:  # if L is unreadable, use F
            y = F*cos(b)
        else:
            if abs(R*cos(b) - F*cos(c)) < m or R > lim: # if F and R both hitting side, don't use F
                y = L*cos(c)
            else:
                y = F*cos(b)


    if zone == 2:

        # X COORDINATE ====================
        if B > lim: # if B unreadable, use L
            x = 300 - L*cos(b)
        else:
            if abs(F*cos(b) - L*cos(c)) < m or F > lim: # if F and L both hitting bottom, don't use L for x
                x = 300 - B*cos(c)
            else:
                x = 300 - L*cos(b)

        # Y COORDINATE ====================
        if F > lim: # if F unreadable, use B
            y = B*cos(c)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or B > lim: # if L and B both hitting side, don't use L for y
                y = F*cos(b)
            else:
                y = B*cos(c)


    if zone == 3:

        # X COORDINATE ====================
        if L > lim: # if L unreadable, use B
            x = 300 - B*cos(c)
        else:
            if abs(R*cos(c) - B*cos(b)) < m or R > lim: # if R and B both hitting top, don't use B for x
                x = 300 - L*cos(b)
            else:
                x = 300 - B*cos(c)
        # Y COORDINATE ====================
        if R > lim: # if R unreadable, use B
            y = 300 - B*cos(b)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or L > lim: # if L and B both hitting side, don't use B for y
                y = 300 - R*cos(c)
            else:
                y = 300 - B*cos(b)


    if zone == 4:

        # X COORDINATE ====================
        if F > lim: # if F unreadable, use R
            x = R*cos(b)
        else:
            if abs(R*cos(b) - B*cos(b)) < m or B > lim: # if R and B both hitting top, don't use R
                x = F*cos(c)
            else:
                x = R*cos(b)

        # Y COORDINATE ====================
        if B > lim: # if B unreadable, use R
            y = 300 - R*cos(c)
        else:
            if abs(R*cos(b) - F*cos(c)) < m or F > lim: # if R and F both hitting side, don't use R for y
                y = 300 - B*cos(b)
            else:
                y = 300 - R*cos(c)


    return round(x),round(y)    # only really need to nearest cm
    # ==============================================================================



# ==================================================================================
#                              ROUTE 2 : -90 < a < 0
# ==================================================================================
def route2(a,F,B,R,L, zone):

    b = a*pi/180   #conversion to radians for correct use in sin/cos
    c = (90 + a)*pi/180
    d = (-a)*pi/180

    if (R > lim and L > lim) or (B > lim and F > lim):
        return "Coordinate cannot be calculated: R + L unknown or B + F unknown."

    if zone == 1:
        # X COORDINATE ====================
        if B > lim: # if B unreadable, use R
            x = R*cos(c)
        else:
            if abs(F*cos(c) - R*cos(b)) < m or F > lim: # if R and F hitting bottom, don't use R for x
                x = B*cos(b)
            else:
                x = R*cos(c)
        # Y COORDINATE ====================
        if F > lim: # if F unreadable, use R
            y = R*cos(b)
        else:
            if abs(B*cos(b) - R*cos(c)) < m or B > lim: # if R and B hitting side, don't use R for y
                y = F*cos(c)
            else:
                y = R*cos(b)


    if zone == 2:

        # X COORDINATE ====================
        if L > lim: # if L unreadable, use F
            x = 300 - F*cos(b)
        else:
            if abs(R*cos(b) - F*cos(c)) < m or R > lim: # if F and R both hitting bottom, don't use F for x
                x = 300 - L*cos(c)
            else:
                x = 300 - F*cos(b)
        # Y COORDINATE ====================
        if R > lim: # if R unreadable, use F
            y = F*cos(c)
        else:
            if abs(L*cos(c) - F*cos(b)) < m or L > lim: # if F and L both hitting side, don't use F for y
                y = R*cos(b)
            else:
                y = F*cos(c)


    if zone == 3:

        # X COORDINATE ====================
        if F > lim: # if F unreadable, use L
            x = 300 - L*cos(c)
        else:
            if abs(B*cos(c) - L*cos(b)) < m or B > lim: # if B and L both hitting top, don't use L for x
                x = 300 - F*cos(b)
            else:
                x = 300 - L*cos(c)

        # Y COORDINATE ====================
        if B > lim: # if B unreadable, use L
            y = 300 - L*cos(b)
        else:
            if abs(L*cos(c) + F*cos(b)) < m or F > lim: # if L and F both hitting side, don't use L for y
                y = 300 - B*cos(c)
            else:
                y = 300 - L*cos(b)


    if zone == 4:

        # X COORDINATE ====================
        if R > lim: # if R unreadable, use B
            x = B*cos(b)
        else:
            if abs(B*cos(c) - L*cos(b)) < m or L > lim: # if B and L both hitting top, don't use B for x
                x = R*cos(c)
            else:
                x = B*cos(b)
        # Y COORDINATE ====================
        if L > lim: # if L unreadable, use B
            y = B*cos(c)
        else:
            if abs(B*cos(b) - R*cos(c)) < m or R > lim: # if B and R both hitting side, don't use B for y
                y = 300 - L*cos(b)
            else:
                y = B*cos(c)


    return round(x),round(y)    # only really need to nearest cm
    # ==============================================================================


# ==================================================================================
#                               ROUTE 3 : 0 < a < 90
# ==================================================================================
def route3(a,F,B,R,L, zone):

    b = a*pi/180   #conversion to radians for correct use in sin/cos
    c = (90 - a)*pi/180

    if (R > lim and L > lim) or (B > lim and F > lim):
        return "Coordinate cannot be calculated: R + L unknown or B + F unknown."


    if zone == 1:

        # X COORDINATE ====================
        if L > lim:  # if L is unreadable, use B
            x = B*cos(b)
        else:
            if abs(B*cos(c) - R*cos(b)) < m or R > lim: # if B and R both hitting bottom, don't use B for x.
                x = L*cos(c)
            else:
                x = B*cos(b)

        # Y COORDINATE ====================
        if R > lim: # if R is unreadable, use B
            y = B*cos(c)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or L > lim: # if B and L both hitting side, don't use B for y
                y = R*cos(b)

            else:
                y = B*cos(c)


    if zone == 2:

        # X COORDINATE ====================
        if F > lim: # if F unreadable, use R
            x = 300 - R*cos(c)
        else:
            if abs(B*cos(c) - R*cos(b)) < m or B > lim: # if B and R both hitting bottom, don't use R for x
                x = 300 - F*cos(b)
            else:
                x = 300 - R*cos(c)
        # Y COORDINATE ====================
        if B > lim: # if B unreadable, use R
            y = R*cos(b)
        else:
            if abs(F*cos(b) - R*cos(c)) < m or F > lim: # if F and R both hitting side, don't use R for y
                y = B*cos(c)
            else:
                y = R*cos(b)


    if zone == 3:

        # X COORDINATE ====================
        if R > lim: # if R unreadable, use F
            x = 300 - F*cos(b)
        else:
            if abs(L*cos(b) - F*cos(c)) < m or L > lim: # if L and F both hitting top, don't use F for x
                x = 300 - R*cos(c)
            else:
                x = 300 - F*cos(b)
        # Y COORDINATE ====================
        if L > lim: # if L unreadable, use F
            y = 300 - F*cos(c)
        else:
            if abs(F*cos(b) - R*cos(c)) < m or R > lim: # if F and R both hitting side, don't use F for y
                y = 300 - L*cos(b)
            else:
                y = 300 - F*cos(c)


    if zone == 4:

        # X COORDINATE ====================
        if B > lim: # if B unreadable, use L
            x = L*cos(c)
        else:
            if abs(L*cos(b) - F*cos(c)) < m or F > lim: # if L and F both hitting top, don't use L for x
                x = B*cos(b)
            else:
                x = L*cos(c)
        # Y COORDINATE ====================
        if F > lim: # if F unreadable, use L
            y = 300 - L*cos(b)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or B > lim: # if L and B both hitting side, don't use L for y
                y = 300 - F*cos(c)
            else:
                y = 300 - L*cos(b)



    return round(x),round(y)    # only really need to nearest cm
    # ==============================================================================


# ==================================================================================
#                               ROUTE 4 : 90 < a < 180
# ==================================================================================
def route4(a,F,B,R,L,zone):

    b = (a - 90)*pi/180   #conversion to radians for correct use in sin/cos
    c = (180 - a)*pi/180

    if (R > lim and L > lim) or (B > lim and F > lim):
        return "Coordinate cannot be calculated: R + L unknown or B + F unknown."

    if zone == 1:

        # X COORDINATE ====================
        if F > lim:  # if F unreadable, use L
            x = L*cos(b)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or B > lim: # if L and B both hitting bottom, don't use L for x
                x = F*cos(c)
            else:
                x = L*cos(b)
        # Y COORDINATE ====================
        if B > lim: # if B unreadable, use L
            y = L*cos(c)
        else:
            if abs(F*cos(c) - L*cos(b)) < m or F > lim: # if F and L both hitting side, don't use L for y
                y = B*cos(b)
            else:
                y = L*cos(c)


    if zone == 2:

        # X COORDINATE ====================
        if R > lim: # if R unreadable, use B
            x = 300 - B*cos(c)
        else:
            if abs(L*cos(c) - B*cos(b)) < m or L > lim: # if L and B both hitting bottom, don't use B for x
                x = 300 - R*cos(b)
            else:
                x = 300 - B*cos(c)
        # Y COORDINATE ====================
        if L > lim: # if L unreadable, use B
            y = B*cos(b)
        else:
            if abs(B*cos(c) - R*cos(b)) < m or R > lim: # if B and R both hitting side, don't use B for y
                y = L*cos(c)
            else:
                y = B*cos(b)


    if zone == 3:

        # X COORDINATE ====================
        if B > lim: # if B unreadable, use R
            x = 300 - R*cos(b)
        else:
            if abs(F*cos(b) - R*cos(c)) < m or F > lim: # if F and R both hitting top, don't use R for x
                x = 300 - B*cos(c)
            else:
                x = 300 - R*cos(b)
        # Y COORDINATE ====================
        if F > lim: # if F unreadable, use R
            y = 300 - R*cos(c)
        else:
            if abs(R*cos(b) - B*cos(c)) < m or B > lim: # if B and R both hitting side, don't use R for y
                y = 300 - F*cos(b)
            else:
                y = 300 - R*cos(c)


    if zone == 4:

        # X COORDINATE ====================
        if L > lim: # if L unreadable, use F
            x = F*cos(c)
        else:
            if abs(F*cos(b) - R*cos(c)) < m or R > lim: # if F and R both hitting top, don't use F for x
                x = L*cos(b)
            else:
                x = F*cos(c)
        # Y COORDINATE ====================
        if R > lim: # if R unreadable, use F
            y = 300 - F*cos(b)
        else:
            if abs(F*cos(c) - L*cos(b)) < m or L > lim: # if F and L both hitting side, don't use F for y
                y = 300 - R*cos(c)
            else:
                y = 300 - F*cos(b)



    return round(x),round(y)    # only really need to nearest cm
    # ===============================================================================
