'''OpenGL extension DFX.multisample

Automatically generated by the get_gl_extensions script, do not edit!
'''
from OpenGL import platform, constants, constant, arrays
from OpenGL import extensions
from OpenGL.GL import glget
import ctypes
EXTENSION_NAME = 'GL_DFX_multisample'
_DEPRECATED = False
GL_MULTISAMPLE_3DFX = constant.Constant( 'GL_MULTISAMPLE_3DFX', 0x86B2 )
glget.addGLGetConstant( GL_MULTISAMPLE_3DFX, (1,) )
GL_SAMPLE_BUFFERS_3DFX = constant.Constant( 'GL_SAMPLE_BUFFERS_3DFX', 0x86B3 )
glget.addGLGetConstant( GL_SAMPLE_BUFFERS_3DFX, (1,) )
GL_SAMPLES_3DFX = constant.Constant( 'GL_SAMPLES_3DFX', 0x86B4 )
glget.addGLGetConstant( GL_SAMPLES_3DFX, (1,) )
GL_MULTISAMPLE_BIT_3DFX = constant.Constant( 'GL_MULTISAMPLE_BIT_3DFX', 0x20000000 )


def glInitMultisampleDFX():
    '''Return boolean indicating whether this extension is available'''
    return extensions.hasGLExtension( EXTENSION_NAME )
