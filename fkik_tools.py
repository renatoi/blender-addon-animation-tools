#====================== BEGIN GPL LICENSE BLOCK ======================
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
#======================= END GPL LICENSE BLOCK ========================

bl_info = {
    "name": "FKIK Tools",
    "author": "Renato Iwashima",
    "location": "View3D > Tools Panel",
    "version": (0, 0, 1),
    "blender": (2, 7, 8),
    "warning": "Match IK will not align the pole target rotation",
    "description": "Tools for FK and IK Animation",
    "wiki_url": "https://github.com/renatoi/blender-addon-fkiktools/blob/master/README.md",
    "category": "Animation"
}

import bpy

from mathutils import Matrix, Vector
from math import acos, pi, radians
from bpy.types import Operator, Panel, PropertyGroup, Scene

context = bpy.context

def get_pose_matrix(global_matrix, pb):
    restInv = pb.bone.matrix_local.inverted()
    if pb.parent:
        parInv = pb.parent.matrix.inverted()
        parRest = pb.parent.bone.matrix_local
        return restInv * (parRest * (parInv * global_matrix))
    else:
        return restInv * global_matrix


def get_global_matrix(mat, pb):
    global_matrix = pb.bone.matrix_local * mat
    if pb.parent:
        parMat = pb.parent.matrix
        parRest = pb.parent.bone.matrix_local
        return parMat * (parRest.inverted() * global_matrix)
    else:
        return global_matrix
    
def insert_location(pb, mat):
    pb.location = mat.to_translation()
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='POSE')

def match_pose_translation(pose_bone, src, use_tail):
    src_matrix = Matrix.Translation(src.tail) if use_tail else src.matrix
    pose_matrix = get_pose_matrix(src_matrix, pose_bone)
    insert_location(pose_bone, pose_matrix)

#########################################
## FKIK Tools ##
#########################################

def get_abone_ik_constraint():
    return next((constraint for constraint in bpy.context.active_pose_bone.constraints if constraint.type == 'IK'), None)

def signed_angle(vector_u, vector_v, normal):
    # Normal specifies orientation
    angle = vector_u.angle(vector_v)
    if vector_u.cross(vector_v).angle(normal) < 1:
        angle = -angle
    return angle

def get_pole_angle(last_ik_bone, ik_bone, pole_location):
    pole_normal = (ik_bone.tail - last_ik_bone.head).cross(pole_location - last_ik_bone.head)
    projected_pole_axis = pole_normal.cross(last_ik_bone.tail - last_ik_bone.head)
    return signed_angle(last_ik_bone.x_axis, projected_pole_axis, last_ik_bone.tail - last_ik_bone.head)

def adjust_ik_pole_angle():
    ik_constraint = get_abone_ik_constraint()
    ik_bone = bpy.context.active_pose_bone
    last_ik_bone = ik_bone.parent_recursive[ik_constraint.chain_count - 2]
    pole_bone = bpy.context.object.pose.bones[ik_constraint.pole_subtarget]
    ik_constraint.pole_angle = get_pole_angle(last_ik_bone, ik_bone, pole_bone.matrix.translation)

def adjust_ik_pole_location():
    ik_constraint = get_abone_ik_constraint()
    ik_bone = bpy.context.active_pose_bone
    pole_bone = bpy.context.object.pose.bones[ik_constraint.pole_subtarget]
    match_pose_translation(pole_bone, ik_bone, False)

def copy_pose(pose):
    context = bpy.context
    ik_constraint = get_abone_ik_constraint()

    if pose == 'IK':
        # pretty sure there are better ways to do this
        ik_bone = context.active_pose_bone
        ik_handle = context.object.pose.bones[ik_constraint.subtarget]
        match_pose_translation(ik_handle, ik_bone, True)
        adjust_ik_pole_location()
        adjust_ik_pole_angle()
        bpy.context.scene.fkik_tools.fkik_enum = pose
    else:
        bones = [context.active_pose_bone]
        bones_m = [context.active_pose_bone.matrix.copy()]
        for x in range(ik_constraint.chain_count - 1):
            parent = bones[-1].parent
            bones.append(parent)
            bones_m.append(parent.matrix.copy())

        bpy.context.scene.fkik_tools.fkik_enum = pose

        for x in range(ik_constraint.chain_count):
            bones[x].matrix = bones_m[x]

def fkik_enum_get(self):
    ik_constraint = get_abone_ik_constraint()
    if ik_constraint.influence > 0:
        return 1
    else:
        return 0

def fkik_enum_set(self, value):
    ik_constraint = get_abone_ik_constraint()
    ik_constraint.influence = value

class FKIKToolsProperties(PropertyGroup):
    fkik_enum = bpy.props.EnumProperty(
        name = "FKIK Enum",
        items = (
            ('FK', "FK", "Sets 0% IK influence", 0),
            ('IK', "IK", "Sets 100% IK influence", 1)
        ),
        default = 'FK',
        get = fkik_enum_get,
        set = fkik_enum_set
    )

class AdjustIkPoleLocationOperator(Operator):
    bl_idname = "fk_ik_tools.adjust_ik_pole_location"
    bl_label = "Adjust IK pole angle"
    bl_description = "Adjusts pole location of IK constraint so it is perpendicular to the head of the IK bone in FK mode"

    def execute(self, context):
        adjust_ik_pole_location()
        self.report({'INFO'}, "Pole location of IK constraint is now perpendicular to the head of the FK bone in FK mode")
        return {'FINISHED'}

class AdjustIkPoleAngleOperator(Operator):
    bl_idname = "fk_ik_tools.adjust_ik_pole_angle"
    bl_label = "Adjust IK pole angle"
    bl_description = "Adjusts pole angle of IK constraint so it doesn't move when switching between FK/IK"

    def execute(self, context):
        adjust_ik_pole_angle()
        self.report({'INFO'}, "Pole angle of IK constraint now matches FK bone position")
        return {'FINISHED'}

class MatchIkOperator(Operator):
    bl_idname = "fk_ik_tools.match_ik"
    bl_label = "Match IK"
    bl_description = "Make IK pose match FK pose and make IK have 100% influence"

    def execute(self, context):
        copy_pose('IK')
        self.report({'INFO'}, "IK pose now matches FK pose")
        return {'FINISHED'}

class MatchFkOperator(Operator):
    bl_idname = "fk_ik_tools.match_fk"
    bl_label = "Match FK"
    bl_description = "Make FK pose match IK pose and make IK have 0% influence"

    def execute(self, context):
        copy_pose('FK')
        adjust_ik_pole_angle()
        self.report({'INFO'}, "FK pose now matches IK pose")
        return {'FINISHED'}

class FKIKToolsPanel(Panel):
    """FK/IK tools in the Viewport Toolbar"""
    bl_label = "FK/IK Tools"
    bl_idname = "OBJECT_PT_ikfk_tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = 'Tools'
    bl_context = "posemode"
    
    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE' or len(context.selected_pose_bones) != 1:
            return None

        ik_constraint = next((x for x in context.active_pose_bone.constraints if x.type == 'IK'), None)
        return (ik_constraint is not None)
    
    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        row = col.row(align=True)
        row.operator('fk_ik_tools.match_fk', text="Match FK", icon='BACK')
        row.operator('fk_ik_tools.match_ik', text="Match IK", icon='FORWARD')
        
        col = layout.column(align=True)
        row = col.row()
        row.prop(context.scene.fkik_tools, "fkik_enum", expand=True)

        if context.scene.fkik_tools.fkik_enum == 'FK':
            col = layout.column(align=True)
            row = col.row()
            row.operator('fk_ik_tools.adjust_ik_pole_angle', text="Adjust IK Pole Angle", icon='MAN_ROT')

            col = layout.column(align=True)
            row = col.row()
            row.operator('fk_ik_tools.adjust_ik_pole_location', text="Adjust IK Pole Location", icon='MANIPUL')

def register():
    bpy.utils.register_module(__name__)
    Scene.fkik_tools = bpy.props.PointerProperty(type=FKIKToolsProperties)
    
def unregister():
    bpy.utils.unregister_module(__name__)

if __name__ == "__main__":
    register()
