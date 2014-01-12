import bpy, bmesh, mathutils
from mathutils import Vector, Matrix
from node_s import *
from util import *

class ListItemNode(Node, SverchCustomTreeNode):
    ''' List item '''
    bl_idname = 'ListItemNode'
    bl_label = 'List item'
    bl_icon = 'OUTLINER_OB_EMPTY'
    
    level = bpy.props.IntProperty(name = 'level_to_count', default=2, min=0, update=updateNode)
    item = bpy.props.IntProperty(name = 'item', default=0, min=0, update=updateNode)
    
    def draw_buttons(self, context, layout):
        layout.prop(self, "level", text="level")
        layout.prop(self, "item", text="item")
        
    def init(self, context):
        self.inputs.new('StringsSocket', "Data", "Data")
        self.outputs.new('StringsSocket',"Item","Item")
        self.outputs.new('StringsSocket',"Other","Other")

    def update(self):
        # достаём два слота - вершины и полики
        if 'Item' in self.outputs and self.outputs['Item'].links or \
                'Other' in self.outputs and self.outputs['Other'].links:
            if not self.outputs['Item'].node.socket_value_update:
                self.outputs['Item'].node.update()
            if not self.outputs['Other'].node.socket_value_update:
                self.outputs['Other'].node.update()
            if 'Data' in self.inputs and self.inputs['Data'].links:
                if not self.inputs['Data'].node.socket_value_update:
                    self.inputs['Data'].node.update()
                if type(self.inputs['Data'].links[0].from_socket) == StringsSocket:
                    data = eval(self.inputs['Data'].links[0].from_socket.StringsProperty)
                elif type(self.inputs['Data'].links[0].from_socket) == VerticesSocket:
                    data = eval(self.inputs['Data'].links[0].from_socket.VerticesProperty)
                elif type(self.inputs['Data'].links[0].from_socket) == MatrixSocket:
                    data = eval(self.inputs['Data'].links[0].from_socket.MatrixProperty)
                
                
                if 'Item' in self.outputs and self.outputs['Item'].links:
                    out = self.count(data, self.level, self.item, True)
                    self.outputs['Item'].StringsProperty = str(out)  
                if 'Other' in self.outputs and self.outputs['Other'].links:
                    out = self.count(data, self.level, self.item, False)
                    self.outputs['Other'].StringsProperty = str(out)
            
    def count(self, data, level, item, itself):
        if level:
            out = []
            for obj in data:
                out.append(self.count(obj, level-1, item, itself))
                
        elif type(data) == tuple:
            if item > len(data)-1:
                item = len(data)-1
            if itself:
                out = [data[item]]
            else:
                out = [data[:item]+data[item+1:]]
        elif type(data) == list:
            if item > len(data)-1:
                item = len(data)-1
            if itself:
                out = [data[item]]
            else:
                data.pop(item)
                out = [data]
        else:
            out = None
        return out
            

    def update_socket(self, context):
        self.update()

def register():
    bpy.utils.register_class(ListItemNode)   
    
def unregister():
    bpy.utils.unregister_class(ListItemNode)

if __name__ == "__main__":
    register()