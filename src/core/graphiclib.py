import math as math

def rgb_to_lab(inputColor):
   num = 0
   RGB = [0, 0, 0]

   for value in inputColor:
       if value > 0.04045 :
           value = ( ( value + 0.055 ) / 1.055 ) ** 2.4
       else :
           value = value / 12.92

       RGB[num] = value * 100
       num = num + 1

   XYZ = [0, 0, 0,]

   X = RGB [0] * 0.4124 + RGB [1] * 0.3576 + RGB [2] * 0.1805
   Y = RGB [0] * 0.2126 + RGB [1] * 0.7152 + RGB [2] * 0.0722
   Z = RGB [0] * 0.0193 + RGB [1] * 0.1192 + RGB [2] * 0.9505
   XYZ[ 0 ] = round( X, 4 )
   XYZ[ 1 ] = round( Y, 4 )
   XYZ[ 2 ] = round( Z, 4 )

   XYZ[ 0 ] = float( XYZ[ 0 ] ) / 95.047         # ref_X =  95.047
   XYZ[ 1 ] = float( XYZ[ 1 ] ) / 100.0          # ref_Y = 100.000
   XYZ[ 2 ] = float( XYZ[ 2 ] ) / 108.883        # ref_Z = 108.883

   num = 0
   for value in XYZ :

       if value > 0.008856 :
           value = value ** ( 0.3333333333333333 )
       else :
           value = ( 7.787 * value ) + ( 16 / 116 )

       XYZ[num] = value
       num = num + 1

   Lab = [0, 0, 0]

   L = ( 116 * XYZ[ 1 ] ) - 16
   a = 500 * ( XYZ[ 0 ] - XYZ[ 1 ] )
   b = 200 * ( XYZ[ 1 ] - XYZ[ 2 ] )

   Lab [ 0 ] = round( L, 4 )
   Lab [ 1 ] = round( a, 4 )
   Lab [ 2 ] = round( b, 4 )

   return Lab

def color_distance(color1, color2):
    color1 = rgb_to_lab(color1)
    color2 = rgb_to_lab(color2)

    c1 = math.sqrt(color1[1]**2+color1[2]**2)
    c2 = math.sqrt(color2[1]**2+color2[2]**2)
    dc = c1-c2
    dl = color1[0]-color2[0]
    da = color1[1]-color2[1]
    db = color1[2]-color2[2]
    dh = math.sqrt((da*da)+(db*db)-(dc*dc))
    first = dl
    second = dc/(1+0.045*c1)
    third = dh/(1+0.015*c1)

    return math.sqrt(first**2+second**2+third**2)
