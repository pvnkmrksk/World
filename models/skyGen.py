from pandac.PandaModules import EggData
from pandac.PandaModules import EggPolygon
from pandac.PandaModules import EggTexture
from pandac.PandaModules import EggVertex
from pandac.PandaModules import EggVertexPool
from pandac.PandaModules import Filename
from pandac.PandaModules import Point2D
from pandac.PandaModules import Point3D

import math
import os

nrows = 6
ncols = 16
uvscale = 2.8
texture = 'models/clouds_null.png'

sqrt2 = math.sqrt( 2.0 )

def makePoly( g, vlist ):
    _poly = EggPolygon( )

    for _v in vlist:
        _poly.addVertex( _v )

    g.addChild( _poly )

def makeVertex( x, y, z, u, v ):
    _v = EggVertex( )

    _v.setPos( Point3D( x, y, z ) )
    _v.setUv( Point2D( u, v ) )

    vp.addVertex( _v )
    return _v

# egg data
vp = EggVertexPool( 'sky' )

data = EggData( )
data.addChild( vp )
data.setCoordinateSystem( 1 ) # CS_zup_right

# vertices
zenith = makeVertex( 0.0, 0.0, sqrt2 - 1.0, 0.0, 0.0 )
vertices = { }
for j in range( 1, nrows + 1 ):
    for i in range( ncols ):
        phi = math.radians( 360.0 / ncols * i )
        theta = math.radians( 45.0 / nrows * j )

        x = sqrt2 * math.sin( theta ) * math.cos( phi )
        y = sqrt2 * math.sin( theta ) * math.sin( phi )
        z = sqrt2 * math.cos( theta ) - 1.0

        factor = uvscale
        u = x * factor
        v = y * factor

        vertices[ ( j, i ) ] = makeVertex( x, y, z, u, v )

# triangles ( first row )
for i in range( ncols ):
    v1 = zenith
    v2 = vertices[ ( 1, ( i + 1 ) % ncols ) ]
    v3 = vertices[ ( 1, i ) ]

    makePoly( data, [ v1, v2, v3 ] )

# quads ( other rows )
for j in range( 1, nrows ):
    for i in range( ncols ):
        i0 = i
        i1 = ( i + 1 ) % ncols
        j0 = j
        j1 = j + 1

        v1 = vertices[ ( j0, i0 ) ]
        v2 = vertices[ ( j0, i1 ) ]
        v3 = vertices[ ( j1, i1 ) ]
        v4 = vertices[ ( j1, i0 ) ]

        makePoly( data, [ v1, v2, v3, v4 ] )

# texture
tex = EggTexture( 'clouds', texture )
tex.setMagfilter( EggTexture.FTLinearMipmapLinear )
tex.setMinfilter( EggTexture.FTLinearMipmapLinear )

n = data.getFirstChild( )
while n:
    if type( n ) == EggPolygon:
        n.addTexture( tex )
    n = data.getNextChild( )

# save
data.writeEgg( Filename( 'sky.egg' ) )

# egg2bam
os.system( 'egg2bam sky.egg -o sky.bam' )
