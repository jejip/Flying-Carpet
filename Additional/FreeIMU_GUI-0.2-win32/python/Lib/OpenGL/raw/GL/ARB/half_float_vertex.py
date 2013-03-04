'''OpenGL extension ARB.half_float_vertex

Automatically generated by the get_gl_extensions script, do not edit!
'''
from OpenGL import platform, constants, constant, arrays
from OpenGL import extensions
from OpenGL.GL import glget
import ctypes
EXTENSION_NAME = 'GL_ARB_half_float_vertex'
_DEPRECATED = False
GL_HALF_FLOAT = constant.Constant( 'GL_HALF_FLOAT', 0x140B )


def glInitHalfFloatVertexARB():
    '''Return boolean indicating whether this extension is available'''
    return extensions.hasGLExtension( EXTENSION_NAME )
