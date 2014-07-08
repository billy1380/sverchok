# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

from math import *
from itertools import zip_longest

import bpy
from bpy.props import (EnumProperty, FloatProperty,
                       IntProperty, BoolVectorProperty)

from node_tree import SverchCustomTreeNode, StringsSocket
from data_structure import (updateNode, match_long_repeat,
                            SvSetSocketAnyType, SvGetSocketAnyType)


class ScalarMathNode(bpy.types.Node, SverchCustomTreeNode):
    ''' ScalarMathNode '''
    bl_idname = 'ScalarMathNode'
    bl_label = 'function'
    bl_icon = 'OUTLINER_OB_EMPTY'


# Math functions from http://docs.python.org/3.3/library/math.html
# maybe this should be distilled to most common with the others available via Formula2 Node
# And some constants etc.
# Keep 4, columns number unchanged and only add new with unique number
    
    mode_items = [
        ("SINE",            "Sine",         "", 1),
        ("COSINE",          "Cosine",       "", 2),
        ("TANGENT",         "Tangent",      "", 3),
        ("ARCSINE",         "Arcsine",      "", 4),
        ("ARCCOSINE",       "Arccosine",    "", 5),
        ("ARCTANGENT",      "Arctangent",   "", 6),
        ("SQRT",            "Squareroot",   "", 11),
        ("NEG",             "Negate",       "", 12),
        ("DEGREES",         "Degrees",      "", 13),
        ("RADIANS",         "Radians",      "", 14),
        ("ABS",             "Absolute",     "", 15),
        ("CEIL",            "Ceiling",      "", 16),
        ("ROUND",           "Round",        "", 17),
        ("ROUND-N",         "Round N",      "", 18),
        ("FMOD",            "Fmod",         "", 19),
        ("MODULO",          "modulo",       "", 20),
        ("FLOOR",           "floor",        "", 21),
        ("EXP",             "Exponent",     "", 30),
        ("LN",              "log",          "", 31),
        ("LOG1P",           "log1p",        "", 32),
        ("LOG10",           "log10",        "", 33),
        ("ACOSH",           "acosh",        "", 40),
        ("ASINH",           "asinh",        "", 41),
        ("ATANH",           "atanh",        "", 42),
        ("COSH",            "cosh",         "", 43),
        ("SINH",            "sinh",         "", 44),
        ("TANH",            "tanh",         "", 45),
        ("ADD",              "+",           "", 50),
        ("SUB",              "-",           "", 51),
        ("MUL",              "*",           "", 52),
        ("DIV",              "/",           "", 53),
        ("INTDIV",           "//",          "", 54),
        ("POW",              "**",          "", 55),
        ("PI",               "pi",          "", 60),
        ("E",                "e",           "", 61),
        ("PHI",              "phi",         "", 62),
        ("TAU",              "tau",         "", 63),
        ("MIN",              "min",         "", 70),
        ("MAX",              "max",         "", 71),
        ("-1",               "x-1",         "", 80),
        ("+1",               "x+1",         "", 81),
        ("*2",               "x*2",         "", 82),
        ("/2",               "x/2",         "", 83),
        ("POW2",             "x**2",        "", 84),
        ]

    fx = {
        'SINE':       sin,
        'COSINE':     cos,
        'TANGENT':    tan,
        'ARCSINE':    asin,
        'ARCCOSINE':  acos,
        'ARCTANGENT': atan,
        'SQRT':       lambda x: sqrt(fabs(x)),
        'NEG':        lambda x: -x,
        'DEGREES':    degrees,
        'RADIANS':    radians,
        'ABS':        fabs,
        'FLOOR':      floor,
        'CEIL':       ceil,
        'EXP':        exp,
        'LN':         log,
        'LOG1P':      log1p,
        'LOG10':      log10,
        'ACOSH':      acosh,
        'ASINH':      asinh,
        'ATANH':      atanh,
        'COSH':       cosh,
        'SINH':       sinh,
        'TANH':       tanh,
        'ROUND':      round,
        '+1':         lambda x: x+1,
        '-1':         lambda x: x-1,
        '*2':         lambda x: x*2,
        '/2':         lambda x: x/2,
        'POW2':       lambda x: x**2,

    }

    fxy = {
        'ADD':      lambda x, y : x+y,
        'SUB':      lambda x, y : x-y,
        'DIV':      lambda x, y : x/y,
        'INTDIV':   lambda x, y : x//y,
        'MUL':      lambda x, y : x*y,
        'POW':      lambda x, y : x**y,
        'ROUND-N':  lambda x, y : round(x, int(y)),
        'FMOD':     lambda x, y : fmod(x, y),
        'MODULO':   lambda x, y : x % y,
        'MIN':      lambda x, y : min(x, y),
        'MAX':      lambda x, y : max(x, y)
    }

    constant = {
        'PI':       pi,
        'TAU':      pi*2,
        'E':        e,
        'PHI':      1.61803398875,
    }
    
    int_prop ={
        'ROUND-N':  ("x","i_y"),
        }
        
        
    # items_ is a really bad name but changing it breaks old layouts 
    items_ = EnumProperty(name="Function", description="Function choice",
                          default="SINE", items=mode_items,
                          update=updateNode)
    x = FloatProperty(default=1, name='x', update=updateNode)
    y = FloatProperty(default=1, name='y', update=updateNode)
    
    # only used for round-n, for completeness right now.
    # perhaps make it switchable via draw buttons ext
    i_x = IntProperty(default=1, name='x', update=updateNode)
    i_y = IntProperty(default=1, name='y', update=updateNode)
    
    # boolvector to control prop type
    def change_prop_type(self, context):
        inputs = self.inputs
        if inputs:
            inputs[0].prop_name = 'i_x' if self.prop_types[0] else 'x'
        if len(inputs)>1:
            if not self.items_ in self.int_prop: 
                inputs[1].prop_name = 'i_y' if self.prop_types[1] else 'y'
            else:
                inputs[1].prop_name = 'i_y'
            
    prop_types = BoolVectorProperty(size=2, default=(False, False),
                                    update=change_prop_type)
        
    def draw_buttons(self, context, layout):
        layout.prop(self, "items_", "Functions:")

    def draw_buttons_ext(self, context, layout):
        layout.label(text="Change property type")
        for i,s in enumerate(self.inputs):
            row = layout.row()
            row.label(text=s.name)
            t = "To int" if self.prop_types[i] else "To float"
            row.prop(self, "prop_types", index=i, text=t, toggle=True)
            
    def init(self, context):
        self.inputs.new('StringsSocket', "X").prop_name = 'x'
        self.outputs.new('StringsSocket', "float")

    def draw_label(self):
        nrInputs = len(self.inputs)
        if nrInputs == 0:
            label = self.items_
        elif nrInputs == 1:
            x_label = 'X' if self.inputs[0].links else str(round(self.x, 3))
            label = " ".join((self.items_, x_label))
        elif nrInputs == 2:
            x_label = 'X' if self.inputs[0].links else str(round(self.x, 3))
            y_label = 'Y' if self.inputs[1].links else str(round(self.y, 3))
            label = " ".join((self.items_ , x_label, ", ", y_label))
        return label
        
    def update(self):

        # inputs
        nrInputs = 1
        if self.items_ in self.constant:
            nrInputs = 0
        elif self.items_ in self.fx:
            nrInputs = 1
        elif self.items_ in self.fxy:
            nrInputs = 2
            # ugly hack to verify int property, should be improved.
            if self.items_ =='ROUND-N':
                if 'Y' in self.inputs:
                    self.inputs['Y'].prop_name = 'i_y'

        self.set_inputs(nrInputs)
        
        if 'X' in self.inputs:
            x = self.inputs['X'].sv_get(deepcopy=False)
            
        if 'Y' in self.inputs:
            y = self.inputs['Y'].sv_get(deepcopy=False)

        # outputs
        if 'float' in self.outputs and self.outputs['float'].links:
            result = []
            if nrInputs == 0:
                result = [[self.constant[self.items_]]]
            elif nrInputs == 1:
                result = self.recurse_fx(x, self.fx[self.items_])
            elif nrInputs == 2:
                result = self.recurse_fxy(x, y, self.fxy[self.items_])
            SvSetSocketAnyType(self, 'float', result)

    def set_inputs(self, n):
        if n == len(self.inputs):
            return
        if n < len(self.inputs):
            while n < len(self.inputs):
                self.inputs.remove(self.inputs[-1])
        if n > len(self.inputs):
            if 'X' not in self.inputs:
                self.inputs.new('StringsSocket', "X")
            if 'Y' not in self.inputs:
                self.inputs.new('StringsSocket', "Y")
            self.change_prop_type(None)

    # apply f to all values recursively
    def recurse_fx(self, l, f):
        if isinstance(l, (int, float)):
            return f(l)
        else:
            return [self.recurse_fx(i, f) for i in l]

    # match length of lists,
    # cases
    # [1,2,3] + [1,2,3] -> [2,4,6]
    # [1,2,3] + 1 -> [2,3,4]
    # [1,2,3] + [1,2] -> [2,4,5] , list is expanded to match length, [-1] is repeated
    # odd cases too.
    # [1,2,[1,1,1]] + [[1,2,3],1,2] -> [[2,3,4],3,[3,3,3]]
    def recurse_fxy(self, l1, l2, f):
        if (isinstance(l1, (int, float)) and isinstance(l2, (int, float))):
                return f(l1, l2)
                
        if (isinstance(l2, (list, tuple)) and isinstance(l1, (list, tuple))):
            fl = l2[-1] if len(l1) > len(l2) else l1[-1]
            res = []
            res_append = res.append
            for x, y in zip_longest(l1, l2, fillvalue=fl):
                res_append(self.recurse_fxy(x, y, f))
            return res
            
        if isinstance(l1, (list, tuple)) and isinstance(l2, (int, float)):
            return self.recurse_fxy(l1, [l2], f)
        if isinstance(l1, (int, float)) and isinstance(l2, (list, tuple)):
            return self.recurse_fxy([l1], l2, f)

    def update_socket(self, context):
        self.update()


def register():
    bpy.utils.register_class(ScalarMathNode)


def unregister():
    bpy.utils.unregister_class(ScalarMathNode)
